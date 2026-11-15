# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "bleak>=3.1.0",
# ]
# ///

"""Run with `uv run scripts/ble_smoke.py` against the physical peripheral."""

import argparse
import asyncio
import json
import struct
import time

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakGATTProtocolError, BleakGATTProtocolErrorCode

NAME = "OxiFOC Remote Stick"
SERVICE = "c5a10000-7d3e-4f71-9a52-3b64e80f2401"
ECHO = "c5a10001-7d3e-4f71-9a52-3b64e80f2401"
UPTIME = "c5a10002-7d3e-4f71-9a52-3b64e80f2401"


def emit(**data):
    print(json.dumps(data), flush=True)


def decode(data):
    assert len(data) == 4, f"Expected u32, got {data.hex()}"
    return struct.unpack("<I", data)[0]


async def run(args):
    started = time.monotonic()
    total_writes = 0
    total_ticks = 0
    previous_tick = None
    for cycle in range(args.cycles):
        device = await BleakScanner.find_device_by_filter(
            lambda device, adv: adv.local_name == NAME
            and (args.address is None or device.address.upper() == args.address.upper()),
            timeout=20,
        )
        assert device is not None, f"{NAME} not found"
        emit(event="discovered", address=device.address, cycle=cycle + 1)
        echoes = asyncio.Queue()
        ticks = asyncio.Queue()
        async with BleakClient(device, timeout=20) as client:
            assert client.is_connected
            assert client.services.get_service(SERVICE) is not None
            for uuid, required in [(ECHO, {"read", "write", "notify"}), (UPTIME, {"read", "notify"})]:
                characteristic = client.services.get_characteristic(uuid)
                assert characteristic is not None
                assert required <= set(characteristic.properties)
            name = bytes(await client.read_gatt_char("00002a00-0000-1000-8000-00805f9b34fb")).decode()
            assert name == NAME, name
            emit(event="connected", name=name)
            await client.start_notify(ECHO, lambda _, data: echoes.put_nowait(decode(data)))
            await client.start_notify(UPTIME, lambda _, data: ticks.put_nowait(decode(data)))
            for i in range(args.writes):
                value = [0, 1, 0xFFFFFFFF, 0x12345678][i % 4] ^ (cycle << 8)
                await client.write_gatt_char(ECHO, struct.pack("<I", value), response=True)
                echoed = await asyncio.wait_for(echoes.get(), 5)
                assert echoed == value, (echoed, value)
                total_writes += 1
            # BlueZ can deliver read-induced Value changes through the notification
            # callback too. Stop echo notifications before checking readback.
            await client.stop_notify(ECHO)
            read_back = decode(await client.read_gatt_char(ECHO))
            assert read_back == value, (read_back, value)
            # The fixed-size register must reject malformed writes and retain its value.
            for malformed in [b"\x01", bytes(5)]:
                try:
                    await client.write_gatt_char(ECHO, malformed, response=True)
                except BleakGATTProtocolError as error:
                    assert error.code == BleakGATTProtocolErrorCode.INVALID_ATTRIBUTE_VALUE_LENGTH, error
                    emit(event="invalid_length_rejected", length=len(malformed), error=str(error))
                else:
                    raise AssertionError("Malformed u32 write was accepted")
            assert decode(await client.read_gatt_char(ECHO)) == value
            deadline = time.monotonic() + args.seconds
            observed = []
            while time.monotonic() < deadline:
                tick = await asyncio.wait_for(ticks.get(), 5)
                if previous_tick is not None:
                    assert tick > previous_tick, (previous_tick, tick)
                previous_tick = tick
                observed.append(tick)
            assert len(observed) >= max(2, args.seconds - 2), observed
            total_ticks += len(observed)
            read_tick = decode(await client.read_gatt_char(UPTIME))
            assert read_tick >= observed[-1]
            emit(event="cycle_passed", cycle=cycle + 1, writes=args.writes, notifications=len(observed), uptime=read_tick)
            await client.stop_notify(UPTIME)
        emit(event="disconnected", cycle=cycle + 1)
        await asyncio.sleep(1)
    emit(result="PASS", cycles=args.cycles, writes=total_writes, uptime_notifications=total_ticks, seconds=round(time.monotonic() - started, 2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--address", help="Restrict discovery to a specific BLE address")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--writes", type=int, default=20)
    parser.add_argument("--seconds", type=int, default=15)
    args = parser.parse_args()
    assert args.cycles > 0 and args.writes > 0 and args.seconds >= 2
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
