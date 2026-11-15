use defmt::{info, warn};
use embassy_futures::{join::join, select::select};
use embassy_time::Timer;
use trouble_host::prelude::*;

const NAME: &str = "OxiFOC Remote Stick";

#[gatt_server]
struct Server {
    diagnostics: DiagnosticsService,
}

/// BLE bring-up service; these values do not control a motor.
#[gatt_service(uuid = "c5a10000-7d3e-4f71-9a52-3b64e80f2401")]
struct DiagnosticsService {
    /// Echo register for read/write/notification checks, little endian.
    #[characteristic(uuid = "c5a10001-7d3e-4f71-9a52-3b64e80f2401", read, write, notify)]
    echo: u32,
    /// Monotonic uptime in seconds, updated once per second while connected.
    #[characteristic(uuid = "c5a10002-7d3e-4f71-9a52-3b64e80f2401", read, notify)]
    uptime: u32,
}

pub async fn run<C: Controller>(controller: C) {
    // Derive a unique static-random address from the factory MAC. HCI is little endian.
    let mac = esp_hal::efuse::base_mac_address();
    let mut bytes = [0; 6];
    bytes.copy_from_slice(mac.as_bytes());
    bytes.reverse();
    bytes[5] |= 0xc0;
    let address = Address::random(bytes);
    info!("BLE address: {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, 1, 2> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .build();
    let mut runner = stack.runner();
    let mut peripheral = stack.peripheral();
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: NAME,
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

    let mut adv_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(NAME.as_bytes()),
        ],
        &mut adv_data,
    )
    .unwrap();

    join(
        async {
            if let Err(e) = runner.run().await {
                panic!("BLE runner failed: {:?}", defmt::Debug2Format(&e));
            }
        },
        async {
            loop {
                let advertiser = peripheral
                    .advertise(
                        &Default::default(),
                        Advertisement::ConnectableScannableUndirected {
                            adv_data: &adv_data[..len],
                            scan_data: &[],
                        },
                    )
                    .await
                    .unwrap();
                info!("BLE advertising as {}", NAME);
                let conn = advertiser
                    .accept()
                    .await
                    .unwrap()
                    .with_attribute_server(&server)
                    .unwrap();
                info!("BLE connected");
                select(events(&server, &conn), uptime(&server, &conn)).await;
            }
        },
    )
    .await;
}

async fn events<P: PacketPool>(server: &Server<'_>, conn: &GattConnection<'_, '_, P>) {
    let echo = server.diagnostics.echo;
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                info!("BLE disconnected: {:?}", reason);
                return;
            }
            GattConnectionEvent::Gatt { event } => {
                let (reply, echo_value) = match event {
                    GattEvent::Write(event) if event.handle() == echo.handle => {
                        match event.value(&echo) {
                            Ok(value) => (event.accept(), Some(value)),
                            Err(_) => (
                                event.reject(AttErrorCode::INVALID_ATTRIBUTE_VALUE_LENGTH),
                                None,
                            ),
                        }
                    }
                    _ => (event.accept(), None),
                };
                match reply {
                    Ok(reply) => reply.send().await,
                    Err(e) => {
                        warn!("GATT request rejected: {:?}", e);
                        continue;
                    }
                }
                if let Some(value) = echo_value {
                    info!("BLE echo write: {}", value);
                    if let Err(e) = echo.notify(conn, &value, false).await {
                        warn!("BLE echo notification failed: {:?}", e);
                    }
                }
            }
            _ => {}
        }
    }
}

async fn uptime<P: PacketPool>(server: &Server<'_>, conn: &GattConnection<'_, '_, P>) {
    loop {
        let seconds = embassy_time::Instant::now().as_secs() as u32;
        if let Err(e) = server.diagnostics.uptime.notify(conn, &seconds, true).await {
            warn!("BLE uptime notification failed: {:?}", e);
            return;
        }
        Timer::after_secs(1).await;
    }
}
