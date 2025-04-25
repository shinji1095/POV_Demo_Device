import asyncio 
import struct 
import csv 
from datetime import datetime 
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Bool, String 
from bleak import BleakClient, BleakScanner 
from queue import Queue, Empty

class BluetoothCentralNode(Node): 
    def __init__(self): 
        super().__init__('bluetooth_central_node')

        # BLE configuration
        self.service_uuid = "19B10000-E8F2-537E-4F6C-D104768A1214"
        self.control_char_uuid = "19B10002-E8F2-537E-4F6C-D104768A1214"
        self.button_char_uuid = "19B10003-E8F2-537E-4F6C-D104768A1214"
        self.device_address = "C7:60:B6:73:0F:87"  # XIAO nRF52840 address
        self.device_name = "XIAO nRF52840"
        self.csv_file = "button_press_log.csv"

        # Queue for thread-safe communication
        self.notification_queue = Queue()

        # ROS 2 subscriber and publisher
        self.subscription = self.create_subscription(
            Bool,
            '/pov_demo/blt/command',
            self.command_callback,
            10
        )
        self.publisher = self.create_publisher(
            String,
            '/pov_demo/blt/button',
            10
        )

        # Timer to process notifications in the main thread
        self.timer = self.create_timer(0.01, self.process_notifications)

        # Initialize CSV file
        with open(self.csv_file, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Received Time", "Button Press Time (ms)"])

        # BLE client
        self.client = None
        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.connect_ble())

    async def find_device(self):
        self.get_logger().info("Scanning for devices...")
        devices = await BleakScanner.discover(timeout=10.0)
        for device in devices:
            if device.name == self.device_name or device.address == self.device_address:
                self.get_logger().info(f"Found {self.device_name} at {device.address}")
                return device.address
        self.get_logger().error(f"Could not find {self.device_name}")
        return None

    async def connect_ble(self):
        try:
            # デバイスをスキャンしてアドレスを取得
            address = await self.find_device()
            if not address:
                self.get_logger().error("Exiting due to device not found.")
                return

            self.client = BleakClient(address)
            await self.client.connect()
            self.get_logger().info(f"Connected to XIAO nRF52840 at {address}")
            await self.client.start_notify(self.button_char_uuid, self.button_press_callback)
            self.get_logger().info("Subscribed to button press notifications")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to BLE device: {str(e)}")
            self.client = None

    def button_press_callback(self, sender, data):
        try:
            press_time = struct.unpack("<I", data)[0]
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            # キューにデータを追加（スレッドセーフ）
            self.notification_queue.put((press_time, current_time))
        except Exception as e:
            # ここではログを出力しない（スレッドセーフではないため）
            pass

    def process_notifications(self):
        # メインスレッドでキューからデータを処理
        try:
            while True:
                press_time, current_time = self.notification_queue.get_nowait()
                self.get_logger().info(f"Button pressed at: {press_time} ms (Received at: {current_time})")

                # Log to CSV
                with open(self.csv_file, mode="a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([current_time, press_time])

                # Publish the press time to ROS 2 topic as a string
                msg = String()
                msg.data = str(press_time)  # 例: "12345"
                self.publisher.publish(msg)
                self.get_logger().info(f"Published press time '{press_time}' to /pov_demo/blt/button")
        except Empty:
            pass  # キューが空の場合は何もしない

    def command_callback(self, msg):
        if self.client is None or not self.client.is_connected:
            self.get_logger().warn("BLE client not connected, cannot send command")
            return

        # Convert Bool to command (True -> 1, False -> 0)
        command = "1" if msg.data else "0"
        command_byte = bytes([int(command)])

        # Run async write in the event loop
        try:
            self.loop.run_until_complete(
                self.client.write_gatt_char(self.control_char_uuid, command_byte)
            )
            self.get_logger().info(f"Sent command: {command}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {str(e)}")

    def destroy_node(self):
        if self.client and self.client.is_connected:
            self.loop.run_until_complete(self.client.disconnect())
            self.get_logger().info("Disconnected from BLE device")
        super().destroy_node()

def main(args=None): 
    rclpy.init(args=args) 
    node = BluetoothCentralNode() 
    try: 
        rclpy.spin(node) 
    except KeyboardInterrupt: 
        node.get_logger().info("Node stopped by user") 
    finally: 
        node.destroy_node() 
        rclpy.shutdown()

if __name__ == '__main__': 
    main()