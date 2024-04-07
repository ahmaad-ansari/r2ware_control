import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import GearCommand, HazardLightsCommand, TurnIndicatorsCommand
from Rosmaster_Lib import Rosmaster
import math

class YahboomCarDriver(Node):
    
    def __init__(self, node_name):
        super().__init__(node_name)

        self.car = Rosmaster()

        self.brake = False
        self.left_turn = False
        self.right_turn = False
        self.reverse = False
        self.emergency = False
        self.park = False

        self.control_command_subscription = self.create_subscription(AckermannControlCommand, '/control/command/control_cmd', self.control_command_callback, 10)
        self.gear_command_subscription = self.create_subscription(GearCommand, '/control/command/gear_cmd', self.gear_command_callback, 10)
        self.hazard_lights_subscription = self.create_subscription(HazardLightsCommand, '/control/command/hazard_lights_cmd', self.hazard_lights_callback, 10)
        self.turn_indicators_subscription = self.create_subscription(TurnIndicatorsCommand, '/control/command/turn_indicators_cmd', self.turn_indicators_callback, 10)

    def control_command_callback(self, msg):
        self.process_car_motion(msg.longitudinal.speed, msg.lateral.steering_tire_angle)
        self.control_led_strip(brake=self.brake, left_turn=self.left_turn, right_turn=self.right_turn, emergency=self.emergency, park=self.park)

    def gear_command_callback(self, msg):
        # Update global variables based on GearCommand message
        if msg.command == autoware_auto_vehicle_msgs.msg.GearCommand_Constants.REVERSE:
            self.reverse = True
        elif msg.command == autoware_auto_vehicle_msgs.msg.GearCommand_Constants.PARK:
            self.park = True
        else:
            self.reverse = False
            self.park = False

    def hazard_lights_callback(self, msg):
        # Update global variables based on HazardLightsCommand message
        if msg.command == autoware_auto_vehicle_msgs.msg.HazardLightsCommand_Constants.ENABLE:
            self.emergency = True
        else:
            self.emergency = False

    def turn_indicators_callback(self, msg):
        # Update global variables based on TurnIndicatorsCommand message
        if msg.command == autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand_Constants.ENABLE_LEFT:
            self.left_turn = True
            self.right_turn = False
        elif msg.command == autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand_Constants.ENABLE_RIGHT:
            self.left_turn = False
            self.right_turn = True
        else:
            self.left_turn = False
            self.right_turn = False

    def process_car_motion(self, speed, angle_rad):
        self.get_logger().info(f"Received Speed (m/s): {speed}")

        autoware_input_min_rad = -0.7
        autoware_input_max_rad = 0.7

        yahboom_output_min = -0.45
        yahboom_output_max = 0.45

        steering_angle = ((angle_rad - autoware_input_min_rad) / (autoware_input_max_rad - autoware_input_min_rad)) * (yahboom_output_max - yahboom_output_min) + yahboom_output_min

        self.car.set_car_motion(speed, steering_angle, 0.0)

    def control_led_strip(self, brake=False, left_turn=False, right_turn=False, reverse=False, emergency=False, park=False):
        # Define default colors for each LED
        colors = [(0, 0, 0)] * LED_STRIP_LENGTH  # Initialize all LEDs to off

        # Set colors based on signals
        if left_turn:
            led_ids_left_turn = [0, 1]
            for led_id in led_ids_left_turn:
                colors[led_id] = (255, 80, 0)  # Set LEDs to yellow for left turn
        if brake:
            led_ids_brake = [3, 4, 5, 6, 7, 8, 9, 10]
            for led_id in led_ids_brake:
                colors[led_id] = (255, 0, 0)  # Set LEDs to red for brake
        if reverse:
            led_ids_reverse = [2, 11]
            for led_id in led_ids_reverse:
                colors[led_id] = (255, 255, 255)  # Set LEDs to green for reverse
        if right_turn:
            led_ids_right_turn = [12, 13]
            for led_id in led_ids_right_turn:
                colors[led_id] = (255, 80, 0)  # Set LEDs to yellow for right turn
        if emergency:
            led_ids_emergency = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
            for led_id in led_ids_emergency:
                colors[led_id] = (255, 0, 0)
        if park:
            led_ids_park = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
            for led_id in led_ids_park:
                colors[led_id] = (0, 255, 0)

        # Set colors on the LED strip
        for i, color in enumerate(colors):
            bot.set_colorful_lamps(i, color[0], color[1], color[2])

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
