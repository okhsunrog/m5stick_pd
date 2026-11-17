//! BLE central link to `oxifoc-bridge`, ported from `oxifoc-remote/src/remote.rs`.
//!
//! Same connection parameters, retry policy, GATT session and ergot setup as the
//! nRF52840 remote, so the two radios can be compared on the same bridge. The
//! stick has no throttle; it only holds the link and serves device info.

use core::cell::Cell;
use core::pin::pin;
use core::sync::atomic::{AtomicU32, Ordering};

use defmt::{error, info, warn};
use embassy_futures::join::join4;
use embassy_futures::select::{Either, select, select3};
use embassy_time::{Duration, Instant, Timer};
use ergot::NetStack;
use ergot::interface_manager::profiles::direct_edge::{DirectEdge, EdgeFrameProcessor};
use ergot::interface_manager::utils::framed_stream;
use ergot::interface_manager::{FrameProcessor, InterfaceState, Profile};
use oxifoc_core::icd::{HardwareInfo, HardwareInfoEndpoint};
use static_cell::StaticCell;
use trouble_host::prelude::*;

use crate::transport;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 3; // Signal + ATT + safety margin

/// Address of the bridge peripheral (must match oxifoc-bridge).
const BRIDGE_ADDR_BYTES: [u8; 6] = [0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff];

/// Same retry policy as oxifoc-remote.
const IMMEDIATE_RETRIES: u32 = 3;
const RETRY_BACKOFF_MIN: Duration = Duration::from_millis(50);
const RETRY_BACKOFF_MAX: Duration = Duration::from_millis(1000);

/// Link statistics shown on the display.
pub struct LinkStats {
    /// Connection attempts that reached LE Connection Complete.
    pub attempts: AtomicU32,
    /// Links that dropped before the GATT client came up (0x3E and similar).
    pub not_established: AtomicU32,
    /// Ergot sessions that became active.
    pub sessions: AtomicU32,
    /// Sessions that ended with a link loss.
    pub losses: AtomicU32,
    /// Last link-lost to session-active time, ms (0 = none yet).
    pub last_recovery_ms: AtomicU32,
    /// 1 while a session is active.
    pub active: AtomicU32,
}

pub static STATS: LinkStats = LinkStats {
    attempts: AtomicU32::new(0),
    not_established: AtomicU32::new(0),
    sessions: AtomicU32::new(0),
    losses: AtomicU32::new(0),
    last_recovery_ms: AtomicU32::new(0),
    active: AtomicU32::new(0),
};

pub async fn run<C: Controller>(controller: C) {
    // Unique static-random address from the factory MAC. HCI is little endian.
    let mac = esp_hal::efuse::base_mac_address();
    let mut bytes = [0; 6];
    bytes.copy_from_slice(mac.as_bytes());
    bytes.reverse();
    bytes[5] |= 0xc0;
    let address = Address::random(bytes);
    info!("BLE address: {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let ble_stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .build();
    let mut central = ble_stack.central();
    let mut runner = ble_stack.runner();
    let bridge_addr = Address::random(BRIDGE_ADDR_BYTES);

    let stack: &'static transport::Stack = {
        static STACK_CELL: StaticCell<transport::Stack> = StaticCell::new();
        STACK_CELL.init(NetStack::new_with_profile(DirectEdge::new_target(
            framed_stream::Sink::new(transport::BLE_OUTQ.framed_producer(), transport::BLE_MTU),
        )))
    };
    info!("Ergot stack initialized (DirectEdge target)");

    let _ = join4(
        async {
            loop {
                if let Err(_e) = runner.run().await {
                    error!("[ble] runner error, restarting");
                }
            }
        },
        info_server(stack),
        stack
            .services()
            .device_info_handler::<2>(&ergot::well_known::DeviceInfo {
                name: Some("Oxifoc Remote Stick".try_into().unwrap_or_default()),
                description: Some("ESP32 link test".try_into().unwrap_or_default()),
                unique_id: 0,
            }),
        async {
            let mut backoff = Duration::from_ticks(0);
            let mut failures = 0u32;
            let mut lost_at: Option<Instant> = None;
            loop {
                if backoff.as_ticks() > 0 {
                    Timer::after(backoff).await;
                }
                info!("[ble] connecting to bridge...");

                let connect_config = ConnectConfig {
                    connect_params: RequestedConnParams {
                        min_connection_interval: Duration::from_millis(15),
                        max_connection_interval: Duration::from_millis(20),
                        supervision_timeout: Duration::from_millis(1000),
                        ..Default::default()
                    },
                    scan_config: ScanConfig {
                        filter_accept_list: &[bridge_addr],
                        ..Default::default()
                    },
                };

                match central.connect(&connect_config).await {
                    Ok(conn) => {
                        STATS.attempts.fetch_add(1, Ordering::Relaxed);
                        info!("[ble] connected to bridge");

                        let client =
                            match GattClient::<_, DefaultPacketPool, 10>::new(&ble_stack, &conn)
                                .await
                            {
                                Ok(c) => c,
                                Err(_e) => {
                                    STATS.not_established.fetch_add(1, Ordering::Relaxed);
                                    warn!("[ble] link not established, retrying");
                                    failures += 1;
                                    backoff = retry_backoff(failures, backoff);
                                    continue;
                                }
                            };

                        let active = Cell::new(false);
                        let _ = select3(
                            client.task(),
                            nus_session(stack, &client, &active, &mut lost_at),
                            connection_events(&ble_stack, &conn),
                        )
                        .await;
                        STATS.active.store(0, Ordering::Relaxed);

                        stack.manage_profile(|im| {
                            let _ = im.set_interface_state((), InterfaceState::Inactive);
                        });
                        // Never replay frames enqueued for the old connection.
                        let consumer = transport::BLE_OUTQ.framed_consumer();
                        while let Ok(frame) = consumer.read() {
                            frame.release();
                        }
                        if active.get() {
                            STATS.losses.fetch_add(1, Ordering::Relaxed);
                            lost_at = Some(Instant::now());
                            failures = 0;
                            backoff = Duration::from_ticks(0);
                        } else {
                            failures += 1;
                            backoff = retry_backoff(failures, backoff);
                        }
                        info!("[ble] disconnected, will reconnect");
                    }
                    Err(_e) => {
                        warn!("[ble] connect failed, retrying");
                        failures += 1;
                        backoff = retry_backoff(failures, backoff);
                    }
                }
            }
        },
    )
    .await;
}

fn retry_backoff(failures: u32, current: Duration) -> Duration {
    if failures <= IMMEDIATE_RETRIES {
        Duration::from_ticks(0)
    } else if current.as_ticks() == 0 {
        RETRY_BACKOFF_MIN
    } else {
        (current * 2).min(RETRY_BACKOFF_MAX)
    }
}

/// Log link-layer events and answer parameter requests until the link drops.
async fn connection_events<C: Controller>(
    stack: &Stack<'_, C, DefaultPacketPool>,
    conn: &Connection<'_, DefaultPacketPool>,
) {
    loop {
        match conn.next().await {
            ConnectionEvent::Disconnected { reason } => {
                info!("[ble] link lost: {:?}", reason);
                return;
            }
            ConnectionEvent::RequestConnectionParams(request) => {
                warn!("[ble] peer requested connection parameters, rejecting");
                if request.reject(stack).await.is_err() {
                    warn!("[ble] parameter request reply failed");
                }
            }
            event => info!("[ble] {:?}", event),
        }
    }
}

/// GATT session: discover the OxiFOC service, subscribe, run the frame loop.
async fn nus_session<C: Controller>(
    stack: &'static transport::Stack,
    client: &GattClient<'_, C, DefaultPacketPool, 10>,
    active: &Cell<bool>,
    lost_at: &mut Option<Instant>,
) {
    let service_uuid = Uuid::from(oxifoc_core::ble::SERVICE_UUID);
    let rx_uuid = Uuid::from(oxifoc_core::ble::RX_UUID);
    let tx_uuid = Uuid::from(oxifoc_core::ble::TX_UUID);

    let services = match client.services_by_uuid(&service_uuid).await {
        Ok(s) => s,
        Err(_e) => {
            error!("[nus] service discovery failed");
            return;
        }
    };
    let Some(service) = services.first() else {
        error!("[nus] OxiFOC service not found");
        return;
    };
    let rx_char: Characteristic<[u8]> = match client.characteristic_by_uuid(service, &rx_uuid).await
    {
        Ok(c) => c,
        Err(_e) => {
            error!("[nus] RX characteristic not found");
            return;
        }
    };
    let tx_char: Characteristic<[u8]> = match client.characteristic_by_uuid(service, &tx_uuid).await
    {
        Ok(c) => c,
        Err(_e) => {
            error!("[nus] TX characteristic not found");
            return;
        }
    };
    let mut listener = match client.subscribe(&tx_char, false).await {
        Ok(l) => l,
        Err(_e) => {
            error!("[nus] subscribe failed");
            return;
        }
    };

    stack.manage_profile(|im| {
        let _ = im.set_interface_state((), InterfaceState::edge_link_local());
    });
    transport::STATE_NOTIFY.wake_all();

    info!("[nus] active, running bidirectional loop");
    active.set(true);
    STATS.sessions.fetch_add(1, Ordering::Relaxed);
    STATS.active.store(1, Ordering::Relaxed);
    if let Some(t) = lost_at.take() {
        let ms = t.elapsed().as_millis() as u32;
        STATS.last_recovery_ms.store(ms, Ordering::Relaxed);
        info!("[nus] recovered {} ms after link loss", ms);
    }

    let mut processor = EdgeFrameProcessor::new();
    let consumer = transport::BLE_OUTQ.framed_consumer();
    loop {
        match select(listener.next(), consumer.wait_read()).await {
            Either::First(notif) => {
                if processor.process_frame(notif.as_ref(), &stack, ()) {
                    transport::STATE_NOTIFY.wake_all();
                }
            }
            Either::Second(grant) => {
                let result = client
                    .write_characteristic_without_response(&rx_char, grant.as_ref())
                    .await;
                grant.release();
                if result.is_err() {
                    warn!("[nus] write error, disconnecting");
                    break;
                }
            }
        }
    }
}

async fn info_server(stack: &'static transport::Stack) {
    let device_info = HardwareInfo {
        bootstrap_magic: oxifoc_core::types::ICD_BOOTSTRAP_MAGIC,
        proto_version: oxifoc_core::types::ICD_PROTO_VERSION,
        capabilities: 0,
        reserved: [0; 8],
        hw: "M5StickC Plus Remote".try_into().unwrap_or_default(),
        sw: "oxifoc-remote-stick-0.1.0".try_into().unwrap_or_default(),
        mcu: "ESP32".try_into().unwrap_or_default(),
        uuid: heapless::String::new(),
        foc_freq_hz: 0,
        max_current_a: 0.0,
        calib: oxifoc_core::types::BoardCalib::default(),
    };

    let server = stack
        .endpoints()
        .bounded_server::<HardwareInfoEndpoint, 2>(Some("hardware_info"));
    let server = pin!(server);
    let mut hdl = server.attach();
    info!("[info] server running");
    loop {
        let _ = hdl
            .serve(|_req: &()| {
                let info = device_info.clone();
                async move { info }
            })
            .await;
    }
}
