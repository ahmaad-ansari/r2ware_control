import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from Rosmaster_Lib import Rosmaster

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
        self.process_motor_speed(msg.longitudinal.speed)
        self.update_colorful_lamps(msg.longitudinal.speed)

    def process_steering_angle(self, angle_rad):
        # Ensure the steering angle is within the range of -0.07 to 0.07
        steering_angle = max(min(angle_rad, 0.07), -0.07)
        steering_angle = -steering_angle
        steering_angle_degrees = ((steering_angle + 0.07) / 0.14) * 180.0

        self.get_logger().info(f"Received Steering Angle (rad): {angle_rad}")
        self.get_logger().info(f"Calculated Steering Angle (deg): {steering_angle_degrees}")
        self.car.set_pwm_servo(1, steering_angle_degrees)

    def process_motor_speed(self, speed):
        mapped_speed = self.map_speed_to_motor(speed)
        self.get_logger().info(f"Received Speed (m/s): {speed}")
        self.get_logger().info(f"Calculated Motor Speed (%): {mapped_speed}")
        self.car.set_motor(0, mapped_speed, 0, mapped_speed)

    def map_speed_to_motor(self, speed):
        if speed > 0:
            return max(min(speed, 100), 35)
        elif speed < 0:
            return min(max(speed, -100), -35)
        else:
            return 0

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
        # Stop everything, set motors to 0, and turn off LEDs
        driver.car.set_motor(0, 0, 0, 0)
        driver.car.set_colorful_lamps(0xff, 0, 0, 0)
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
