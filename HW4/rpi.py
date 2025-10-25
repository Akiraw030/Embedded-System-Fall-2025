import asyncio
import sys
import struct
import argparse
from bleak import BleakClient, BleakScanner

# STM32 上 'BlueNRG' 的名稱
TARGET_NAME = "BlueNRG"

# -----------------------------------------------------------------
# *關鍵：使用從掃描結果中發現的實際 UUID*
# -----------------------------------------------------------------
# Service UUID from scan: 000000c0-496d-6441-734d-4847646e7242
ACCEL_SERVICE_UUID = "000000c0-496d-6441-734d-4847646e7242"

# Char (a) Data UUID from scan: 000000c1-496d-6441-734d-4847646e7242
ACCEL_DATA_CHAR_UUID = "000000c1-496d-6441-734d-4847646e7242"

# Char (b) Freq UUID from scan: 000000c2-496d-6441-734d-4847646e7242
ACCEL_FREQ_CHAR_UUID = "000000c2-496d-6441-734d-4847646e7242"

def notification_handler(sender, data):
    """
    Callback when BLE notification is received (from accelerometer characteristic).
    'data' is a 6-byte array.
    """
    # 'data' contains 3 int16_t (signed short), little-endian
    # < = little-endian
    # h = int16_t (short)
    try:
        x, y, z = struct.unpack('<hhh', data)
        # The unit 'mg' is just indicative, refer to STM32 BSP for actual scaling
        print(f"Accel Data (X, Y, Z): {x}, {y}, {z} (raw value)")
    except struct.error:
        print(f"Error unpacking data: {data.hex()}")

async def user_input_loop(client, freq_char):
    """
    A concurrent task that listens for user input to change the frequency.
    Runs in parallel with the notification handler.
    
    NOTE: This uses asyncio.to_thread, which requires Python 3.9+
    """
    print("\n--- Real-time Frequency Control Activated ---")
    print("Enter a new frequency (1-20 Hz) and press Enter to update.")
    print("Type 'q' and press Enter to quit the program.")
    
    while client.is_connected:
        try:
            # Run the blocking input() function in a separate thread
            # This prevents it from blocking our main asyncio event loop
            user_input = await asyncio.to_thread(input, "New Frequency (Hz) > ")
            
            # Check connection status again after input
            if not client.is_connected:
                print("Client disconnected while waiting for input.")
                break

            if user_input.lower() == 'q':
                print("Quit command received. Exiting...")
                break # This will break the loop and end the program gracefully

            # --- Process the new frequency ---
            try:
                frequency_hz = int(user_input)
                
                # Check if frequency is within the safe range
                if not (1 <= frequency_hz <= 20):
                    print(f"Error: Frequency {frequency_hz}Hz is outside the safe range (1-20Hz).")
                    print("Please enter a value between 1 and 20.")
                    continue # Ask for input again

                # Calculate and pack the new period
                period_ms = int(1000 / frequency_hz)
                period_bytes = struct.pack('<H', period_ms)

                print(f"--> Writing new period: {period_ms} ms ({frequency_hz} Hz)...")
                
                # Write the new value to the characteristic
                await client.write_gatt_char(freq_char, period_bytes, response=True)
                
                print(f"--> Successfully wrote new period: {period_ms} ms")

            except ValueError:
                print(f"Invalid input: '{user_input}'. Please enter a number (e.g., 5, 10) or 'q'.")
            except Exception as e:
                print(f"Error while writing new frequency: {e}")
        
        except asyncio.CancelledError:
            print("Input task cancelled.")
            break
        except Exception as e:
            print(f"An error occurred in the input loop: {e}")
            break

async def run_ble_client(initial_frequency_hz): # Renamed parameter
    """
    Main function: Scan, connect, interact.
    """
    device = None
    print(f"Scanning for BLE device (Name: '{TARGET_NAME}') ...")

    # 1. Scan for device (Same as before)
    try:
        device = await BleakScanner.find_device_by_name(TARGET_NAME, timeout=10.0)
    except Exception as e:
        print(f"Error during scan: {e}")
        return

    if device is None:
        print(f"* Error: Device named '{TARGET_NAME}' not found. *")
        # ... (Rest of the error/discovery logic is same as before) ...
        print("Scanning again to list all devices...")
        try:
            devices = await BleakScanner.discover(timeout=5.0)
            if not devices:
                print("No BLE devices found nearby.")
            else:
                print("Nearby devices found:")
                for d in devices:
                    print(f" - {d.name} ({d.address})")
        except Exception as e:
            print(f"Error during discovery scan: {e}")
        return

    print(f"\n*** Found target device: {device.name} ({device.address}) ***\n")

    # 2. Connect to device
    try:
        async with BleakClient(device) as client:
            print(f"Successfully connected to {device.name} (MAC: {device.address})")

            # 3. Verify service and characteristics exist (Same as before)
            try:
                service = client.services.get_service(ACCEL_SERVICE_UUID)
                if not service:
                    print(f"Error: Service UUID not found: {ACCEL_SERVICE_UUID}")
                    # ... (rest of service error logic) ...
                    return

                data_char = service.get_characteristic(ACCEL_DATA_CHAR_UUID)
                freq_char = service.get_characteristic(ACCEL_FREQ_CHAR_UUID)

                if not data_char or not freq_char:
                    print(f"Error: One or more characteristic UUIDs not found.")
                    # ... (rest of char error logic) ...
                    return

                print("Service and Characteristic UUIDs verified.")

            except Exception as e:
                print(f"Error finding service/characteristics: {e}")
                return

            # 4. Write *Initial* Sampling Period (Char b)
            try:
                # Check if initial frequency is valid
                if not (1 <= initial_frequency_hz <= 20):
                    print(f"Error: Initial frequency {initial_frequency_hz}Hz is outside the safe range (1-20Hz).")
                    return

                period_ms = int(1000 / initial_frequency_hz)
                period_bytes = struct.pack('<H', period_ms)

                print(f"Attempting to write initial period: {period_ms} ms ({initial_frequency_hz} Hz)")
                await client.write_gatt_char(freq_char, period_bytes, response=True)
                print(f"Successfully wrote initial sampling period: {period_ms} ms")

            except Exception as e:
                print(f"Error writing initial frequency characteristic: {e}")
                return

            # 5. Subscribe to accelerometer data notifications (Char a)
            try:
                print("\nSubscribing to accelerometer data notifications...")
                await client.start_notify(data_char, notification_handler)
                print("Subscription successful. Waiting for data...")

            except Exception as e:
                print(f"Error subscribing to notifications: {e}")
                return

            # 6. MODIFICATION: Run the concurrent input loop
            #    This function will now run until the user types 'q'
            #    or the connection is lost.
            await user_input_loop(client, freq_char)

            # 7. Cleanup (after user_input_loop finishes)
            print("Stopping notifications...")
            try:
                await client.stop_notify(data_char)
            except Exception as e:
                # Don't crash if stopping fails (e.g., if already disconnected)
                print(f"Note: Error stopping notifications (might be disconnected): {e}")

    except Exception as e:
        print(f"\nAn unexpected error occurred during connection or operation: {e}")
    finally:
        print("\n--- Operation finished, connection closed ---")


if __name__ == "__main__":

    # --- Argument Parsing (Same as before) ---
    parser = argparse.ArgumentParser(description="BLE Accelerometer client for 'BlueNRG'.")
    parser.add_argument(
        "-f",
        "--frequency",
        type=int,
        default=10, # Default to 10Hz if not specified
        help="Set *initial* accelerometer sampling frequency (Hz). Safe range: 1-20 Hz. (Default: 10Hz)"
    )
    args = parser.parse_args()
    # --- End Argument Parsing ---

    try:
        print(f"--- Attempting connection with *initial* frequency: {args.frequency} Hz ---")
        # Call the modified main function
        asyncio.run(run_ble_client(args.frequency))
    except KeyboardInterrupt:
        print("\nUser interrupted program (Ctrl+C).")
        sys.exit(0)