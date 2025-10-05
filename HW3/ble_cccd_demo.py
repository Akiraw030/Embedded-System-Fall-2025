#!/usr/bin/env python3
"""
ble_cccd_demo.py

Run with the same python that has bluepy installed, e.g.:
sudo /home/Akiraw/bluepy-env/bin/python3 ble_cccd_demo.py
"""

import sys, time
from bluepy.btle import Scanner, DefaultDelegate, Peripheral, UUID, BTLEGattError

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print("Discovered device", dev.addr, "(" + dev.addrType + ")")
        elif isNewData:
            print("Received new data from", dev.addr)

class NotifyDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
    def handleNotification(self, cHandle, data):
        print(f">>> Notification/Indication from handle 0x{cHandle:x}: {data}")

def scan_devices(timeout=6.0):
    scanner = Scanner().withDelegate(ScanDelegate())
    devices = scanner.scan(timeout)
    # convert to list to ensure subscriptable
    devices = list(devices)
    if not devices:
        print("No devices found.")
        return []
    print("\nScan results:")
    for i, d in enumerate(devices):
        name = d.getValueText(9) or d.getValueText(8) or "<no name>"
        print(f"{i}: {d.addr} ({d.addrType}) RSSI={d.rssi} dB name='{name}'")
    return devices

def list_device_gatt(dev):
    print("\n--- Services & Characteristics ---")
    for svc in dev.getServices():
        print(f"Service: {svc.uuid} handles: {svc.hndStart}-{svc.hndEnd}")
        for ch in svc.getCharacteristics():
            try:
                props = ch.propertiesToString()
            except Exception:
                props = "<couldn't get properties>"
            print(f"  Char: UUID={ch.uuid} handle=0x{ch.getHandle():x} props={props}")
            try:
                for desc in ch.getDescriptors():
                    print(f"    Desc: handle=0x{desc.handle:x} uuid={desc.uuid}")
            except Exception:
                pass

def find_cccd_descriptor_for_char(dev, char):
    for desc in char.getDescriptors():
        try:
            if str(desc.uuid).lower().endswith("2902") or desc.uuid == UUID(0x2902):
                return desc
        except Exception:
            continue
    # fallback: try handle+1 (common but not guaranteed)
    possible = char.getHandle() + 1
    try:
        # try to read; if read returns something, we can try to use this handle for writing
        _ = dev.readCharacteristic(possible)
        return possible
    except Exception:
        return None

def write_cccd(dev, char, value_bytes):
    desc = find_cccd_descriptor_for_char(dev, char)
    if desc is None:
        print("No CCCD (0x2902) found for this characteristic.")
        return False
    if isinstance(desc, int):
        handle = desc
    else:
        handle = desc.handle
    print(f"Writing CCCD to handle 0x{handle:x}: {value_bytes.hex()}")
    dev.writeCharacteristic(handle, value_bytes, withResponse=True)
    print("Write done.")
    return True

def main():
    devices = scan_devices(timeout=8.0)
    if not devices:
        return

    # allow auto-select by name: type 'name:AKIRAW' or a numeric index
    selection = input("\nEnter device number to connect (or 'name:AKIRAW' to auto-select): ").strip()
    if selection.lower().startswith("name:"):
        wanted = selection.split(":",1)[1].strip()
        idx = None
        for i, d in enumerate(devices):
            name = d.getValueText(9) or d.getValueText(8) or ""
            if wanted and wanted in name:
                idx = i
                break
        if idx is None:
            print(f"No device found with name containing '{wanted}'. Exiting.")
            return
    else:
        try:
            idx = int(selection)
        except Exception:
            print("Invalid input. Please enter a number or name:<device_name>")
            return

    if idx < 0 or idx >= len(devices):
        print(f"Index {idx} out of range (0..{len(devices)-1}). Exiting.")
        return

    devinfo = devices[idx]
    addr = devinfo.addr
    addrType = getattr(devinfo, "addrType", "random")  # fallback to random
    print(f"Connecting to {addr} ({addrType}) ...")
    p = None
    try:
        p = Peripheral(addr, addrType)
        p.withDelegate(NotifyDelegate())
        time.sleep(0.3)  # small wait for discovery

        list_device_gatt(p)

        ch_uuid_in = input("\nEnter characteristic UUID to set CCCD for (example 0000fff0 or 0xfff0) [empty -> 0xFFF1]: ").strip()
        if not ch_uuid_in:
            target_uuid = UUID(0xfff1)
        else:
            try:
                if ch_uuid_in.startswith("0x") or ch_uuid_in.isdigit():
                    target_uuid = UUID(int(ch_uuid_in, 0))
                else:
                    target_uuid = UUID(ch_uuid_in)
            except Exception:
                print("Invalid UUID format.")
                return

        ch_list = p.getCharacteristics(uuid=target_uuid)
        if not ch_list:
            print("Characteristic not found with UUID:", target_uuid)
            print("All characteristics:")
            for ch in p.getCharacteristics():
                try:
                    print("  ", ch.uuid, "handle=0x%X" % ch.getHandle(), ch.propertiesToString())
                except Exception:
                    print("  ", ch.uuid, "handle=0x%X" % ch.getHandle())
            return

        ch = ch_list[0]
        print("Selected characteristic:", ch.uuid, "handle=0x%X" % ch.getHandle(), "props:", ch.propertiesToString())

        print("\n--- CCCD Demo ---")
        print("1) Write 0x0002 to CCCD (enable Indications)")
        print("2) Write 0x0001 to CCCD (enable Notifications)")
        print("3) Write 0x0000 to CCCD (disable)")
        choice = input("Choose (1/2/3) [1]: ").strip() or "1"
        if choice == "1":
            val = bytes([0x02, 0x00])
        elif choice == "2":
            val = bytes([0x01, 0x00])
        else:
            val = bytes([0x00, 0x00])

        if not write_cccd(p, ch, val):
            print("Failed to write CCCD. Exiting.")
            return

        print("\nListening for notifications/indications for 60 seconds...")
        start = time.time()
        while time.time() - start < 60:
            if p.waitForNotifications(1.0):
                continue
        print("Done listening.")

    except BTLEGattError as e:
        print("BTLEGattError:", e)
    except Exception as e:
        print("Exception:", e)
    finally:
        if p:
            try:
                p.disconnect()
            except Exception:
                pass
        print("Disconnected.")

if __name__ == "__main__":
    main()
