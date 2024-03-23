import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from Rosmaster_Lib import Rosmaster
import math

class YahboomCarDriver(Node):
    CONTROL_COMMAND_TOPIC = '/control/command/control_cmd'

    def __init__(self, node_name, control_command_topic=CONTROL_COMMAND_TOPIC):
        super().__init__(node_name)

        self.car = Rosmaster()
        self.control_command_subscription = self.create_subscription(
            AckermannControlCommand,
            control_command_topic,
            self.control_command_callback,
            10
        )

    def control_command_callback(self, msg):
        self.process_steering_angle(msg.lateral.steering_tire_angle)
        self.process_car_motion(msg.longitudinal.speed)
        self.update_colorful_lamps(msg.longitudinal.speed)

    def process_steering_angle(self, angle_rad):
        input_min = -0.7   # Minimum value of the input range
        input_max = 0.7    # Maximum value of the input range
        output_min = 0     # Minimum value of the output range
        output_max = 180   # Maximum value of the output range

        mapped_value = (angle_rad - input_min) * (output_max - output_min) / (input_max - input_min) + output_min

        self.get_logger().info(f"Received Steering Angle (rad): {angle_rad}")
        self.get_logger().info(f"Calculated Steering Angle (deg): {mapped_value}")

        # Assuming set_pwm_servo takes the adjusted_degrees
        self.car.set_pwm_servo(1, mapped_value)



    def process_car_motion(self, speed):
        self.get_logger().info(f"Received Speed (m/s): {speed}")
        # self.car.set_car_motion(speed, 0.0, 0.0)

    def update_colorful_lamps(self, speed):
        if speed < 0:
            self.car.set_colorful_lamps(0xff, 255, 255, 255)  # Set color to white
        elif speed == 0:
            self.car.set_colorful_lamps(0xff, 255, 0, 0)  # Set color to red
        else:
            self.car.set_colorful_lamps(0xff, 0, 0, 0)  # Turn off lamps

def main():
    rclpy.init()
    driver = YahboomCarDriver('r2ware_control_node')  
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop everything and turn off LEDs
        driver.car.set_car_motion(0.0, 0.0, 0.0)
        driver.car.set_colorful_lamps(0xff, 0, 0, 0)
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
