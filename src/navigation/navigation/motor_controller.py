import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import smbus2
import time

# This node handles the transmission of motor speed commands to the motor controller (ESP32) via I2C.
# Scaled speed control values from either teleoperation or autonomous control are sent as two
# signed bytes (int8, -100..100). The ESP32 maps these to ESC PWM microseconds (1000..2000).
# Publishers:
# Subscribers: 'set_motor_speeds', 'emergency_stop'
class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # I2C address of the motor controller (ESP32) and bus number on the Pi 5
        self.I2C_address = 0x55
        self.I2C_bus = 13
        self.bus = smbus2.SMBus(self.I2C_bus)

        # Subscription to the 'set_motor_speeds' topic
        self.speed_subscription = self.create_subscription(
            Float32MultiArray,
            'set_motor_speeds',
            self.set_motor_speeds,
            10)

        # Subscription to emergency stop topic
        # If emergency stop is triggered, the node will be destroyed and both motors will be stopped
        self.emergency_stop = self.create_subscription(
            Bool,
            'emergency_stop',
            self.destroy_node,
            10
        )

    # Convert a speed in [-1.0, 1.0] to a signed int8 in [-100, 100]
    def convert_speed(self, speed):
        scaled = int(round(speed * 100))
        if scaled > 100:
            scaled = 100
        elif scaled < -100:
            scaled = -100
        return scaled

    # Convert a signed int to an unsigned byte (two's complement) for I2C transmission
    def to_byte(self, value):
        return value & 0xFF

    # Processes the incoming speed command from the 'set_motor_speeds' topic
    # Input: [Float32MultiArray] msg containing the speed values for left and right motors
    def set_motor_speeds(self, msg):
        if len(msg.data) < 2:
            return
        # Check if the message is correctly formatted and scaled speed values are valid in the range -1 to 1
        if (-1 <= msg.data[0] <= 1 and -1 <= msg.data[1] <= 1):
            left = self.convert_speed(msg.data[0])
            right = self.convert_speed(msg.data[1])
            self.send_value(left, right)

    # Send the scaled speed values to the motor controller via I2C
    # Input: [int] left_value, [int] right_value in range [-100, 100]
    def send_value(self, left_value, right_value):
        byte_list = [self.to_byte(left_value), self.to_byte(right_value)]
        try:
            # Write two signed bytes: [left_speed, right_speed]
            self.bus.write_i2c_block_data(self.I2C_address, 0, byte_list)
        except Exception as e:
            self.get_logger().info(f"Failed to send value: {e}")

    # Stop the motors by sending zero speed to both
    def stop_motors(self):
        self.send_value(0, 0)

    # Destroy the node when the emergency stop is triggered
    # Stops the motors before shutting down
    def destroy_node(self, msg=None):
        self.stop_motors()
        time.sleep(0.1)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorControllerNode()
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        motor_controller.get_logger().info("Stopping motors")
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
