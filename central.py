import asyncio
import struct
import csv
from datetime import datetime
from bleak import BleakScanner, BleakClient

# BLE UUID
SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"
CONTROL_CHAR_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"
BUTTON_CHAR_UUID = "19B10003-E8F2-537E-4F6C-D104768A1214"
DEVICE_NAME = "XIAO nRF52840"
DEVICE_ADDRESS = "C7:60:B6:73:0F:87" 

# CSVファイル
CSV_FILE = "button_press_log.csv"

def button_press_callback(sender, data):
    press_time = struct.unpack("<I", data)[0]
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"Button pressed at: {press_time} ms (Received at: {current_time})")
    with open(CSV_FILE, mode="a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([current_time, press_time])

async def main():
    # CSVファイルのヘッダ作成（初回のみ）
    with open(CSV_FILE, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Received Time", "Button Press Time (ms)"])

    address = DEVICE_ADDRESS
    if not address:
        print("Exiting due to device not found.")
        return

    print("\nStarting BLE communication. Press Ctrl+C to stop.")
    
    async with BleakClient(address) as client:
        print(f"Connected to {address}")
        
        await client.start_notify(BUTTON_CHAR_UUID, button_press_callback)
        print("Subscribed to button press notifications.")

        command = "0"
        try:
            while True:
                command = "1" if command == "0" else "0"
                command_byte = bytes([int(command)])
                await client.write_gatt_char(CONTROL_CHAR_UUID, command_byte)
                print(f"Sent command: {command}")
                await asyncio.sleep(1.0)
        except KeyboardInterrupt:
            print("\nStopped by user (Ctrl+C).")
            await client.stop_notify(BUTTON_CHAR_UUID)

if __name__ == "__main__":
    asyncio.run(main())