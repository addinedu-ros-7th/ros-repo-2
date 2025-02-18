import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from .pinkylib import Pinky
from .battery import Battery

from rcl_interfaces.msg import SetParametersResult

class PinkyBringup(Node):
 
    def __init__(self):
        super().__init__('pinky_bringup')
 
        self.pinky = Pinky()
        self.battery = Battery()

        self.pinky.enable_motor()
        self.pinky.start_motor()
 
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.battery_publisher = self.create_publisher(
            Float32,
            '/pinky_battery_present',
            10
        )
        self.timer = self.create_timer(5.0, self.battery_callback)

        self.declare_parameter('motor_ratio', 1.0) # 왼쪽 모터 출력이 오른쪽 모터 출력에 비례해 조정되는 파라미터 
        self.motor_ratio = self.get_parameter('motor_ratio').value
        self.pinky.set_ratio(self.motor_ratio)
     
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("pinky is ready!!")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'motor_ratio':
                self.motor_ratio = param.value
                self.pinky.set_ratio(self.motor_ratio)

        self.get_logger().info(f"set L motor ratio {self.motor_ratio * 100} %")
        
        return SetParametersResult(successful=True)

    def battery_callback(self):
        msg = Float32()
        msg.data = self.battery.get_battery()

        self.battery_publisher.publish(msg)
 
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x 
        angular_z = msg.angular.z / 5

        # 좌우 회전
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z

        set_l = self.custom_map(left_speed)
        set_r = self.custom_map(right_speed)
 
        self.pinky.move(set_l, set_r)

    def custom_map(self, value):
        if value == 0:
            return 0
        elif value > 0:
            return 25 + ((value * 20) / 0.5)
        else:
            return -25 + ((value * 20) / 0.5)

    def destroy_node(self):
        # PWM 정지 및 GPIO 정리
        self.pinky.disable_motor()
        self.pinky.stop_motor()
        self.pinky.clean()
        
def main(args=None):
    rclpy.init(args=args)
    pinky_bringup_node = PinkyBringup()
     
    try:
        rclpy.spin(pinky_bringup_node)
    except KeyboardInterrupt:
        pass
    finally:
        pinky_bringup_node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()
