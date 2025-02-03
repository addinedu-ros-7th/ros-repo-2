import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

class Battery:
    def __init__(self, min_value=2396, max_value=3215):
        self.min_value = min_value
        self.max_value = max_value
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS.ADS1115(self.i2c)
        self.chan = AnalogIn(self.ads, ADS.P0)

    def calculate_percentage(self, value):
        if value < self.min_value:
            return 0.0
        elif value > self.max_value:
            return 100.0
        else:
            percentage = (value - self.min_value) / (self.max_value - self.min_value) * 100
            return percentage

    def get_battery(self, sample_count=10):
        values = []

        for _ in range(sample_count):
            values.append(self.chan.value)
            time.sleep(0.1)

        avg_value = sum(values) / len(values)
        percentage = round(self.calculate_percentage(avg_value), 1)

        return percentage

    def clean(self):
        self.i2c.deinit()

    def __del__(self):
        self.clean()

class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')
        self.publisher = self.create_publisher(Float32, 'battery_percentage', 10)
        self.timer = self.create_timer(1.0, self.publish_battery_percentage)  # 1초마다 실행
        self.battery = Battery()
        self.get_logger().info("Battery Node has started.")

    def publish_battery_percentage(self):
        try:
            percentage = self.battery.get_battery()
            msg = Float32()
            msg.data = percentage
            self.publisher.publish(msg)
            self.get_logger().info(f'Published battery percentage: {percentage}%')
        except Exception as e:
            self.get_logger().error(f"Error reading battery percentage: {e}")

    def destroy(self):
        del self.battery
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down battery node.")
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
