import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import sys
import tty
import termios
import time
import threading

# This node provides the teleoperation interface for the robot.
# It reads keyboard inputs via tty/termios raw mode (works over SSH) and publishes
# corresponding motor speed commands on the range -1 to 1.
# Publishers: 'set_motor_speeds'
# Subscribers: 'emergency_stop'
# Controls: w=forward, s=reverse, a=left, d=right, space=stop, q=quit
class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')

        # Publisher for motor speeds
        # Message type: Float32MultiArray
        self.speed_publisher = self.create_publisher(Float32MultiArray, 'set_motor_speeds', 10)

        # Initializes left and right motor speed values
        self.left_value = 0.0
        self.right_value = 0.0

        # Initializes variable to store the last key pressed
        self.last_input = ''

        # Flag to signal the input thread to stop
        self.running = True

        # Subscription to emergency stop topic
        # If emergency stop is triggered, the node will be destroyed
        self.emergency_stop = self.create_subscription(
            Bool,
            'emergency_stop',
            self.destroy_node,
            10
        )

        # Save original terminal settings so we can restore on exit
        self.old_settings = termios.tcgetattr(sys.stdin)

        # Start keyboard reading in a background thread
        self.input_thread = threading.Thread(target=self.read_keys, daemon=True)
        self.input_thread.start()

        self.get_logger().info('Teleop ready. Controls: w/s=fwd/rev, a/d=left/right, space=stop, q=quit')

    def read_keys(self):
        try:
            tty.setraw(sys.stdin.fileno())
            while self.running:
                key = sys.stdin.read(1)
                if not key:
                    break
                if key == ' ':
                    self.process_key('space')
                elif key in ('w', 'a', 's', 'd', 'q'):
                    self.process_key(key)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    # Process key inputs and update motor speeds accordingly
    # 'w' for forward, 's' for backward, 'a' for left, 'd' for right, 'space' for stop
    # If the same key is pressed consecutively, increase the speed at which the robot moves in that direction
    # If a different key is pressed, set the speed to a default value
    # Speed increments of 0.1 per keypress, capped at ±1.0
    def process_key(self, key):
        # Key 'w': forward linear movement
        if key == 'w':
            # If 'w' is pressed after another linear speed command ('w' or 's'), increase speed in the forward direction
            if key == self.last_input or self.last_input == 's':
                self.left_value = min(1.0, self.left_value + 0.1)
                self.right_value = min(1.0, self.right_value + 0.1)
            # If 'w' is pressed after a turn command ('a' or 'd'), set speed to a default forward value
            else:
                self.left_value = 0.2
                self.right_value = 0.2
        # Key 's': backward linear movement
        elif key == 's':
            # If 's' is pressed after another linear speed command ('w' or 's'), increase speed in the backward direction
            if key == self.last_input or self.last_input == 'w':
                self.left_value = max(-1.0, self.left_value - 0.1)
                self.right_value = max(-1.0, self.right_value - 0.1)
            # If 's' is pressed after a turn command ('a' or 'd'), set speed to a default backward value
            else:
                self.left_value = -0.2
                self.right_value = -0.2
        # Key 'a': turn left
        elif key == 'a':
            # If 'a' is pressed after another turn command ('a' or 'd'), increase speed in the left direction
            if key == self.last_input or self.last_input == 'd':
                self.left_value = max(-1.0, self.left_value - 0.1)
                self.right_value = min(1.0, self.right_value + 0.1)
            # If 'a' is pressed after a linear speed command ('w' or 's'), set speed to a default left turn value
            else:
                self.left_value = -0.2
                self.right_value = 0.2
        # Key 'd': turn right
        elif key == 'd':
            # If 'd' is pressed after another turn command ('a' or 'd'), increase speed in the right direction
            if key == self.last_input or self.last_input == 'a':
                self.left_value = min(1.0, self.left_value + 0.1)
                self.right_value = max(-1.0, self.right_value - 0.1)
            # If 'd' is pressed after a linear speed command ('w' or 's'), set speed to a default right turn value
            else:
                self.left_value = 0.2
                self.right_value = -0.2
        # Space: stop the robot
        elif key == 'space':
            self.left_value = 0.0
            self.right_value = 0.0
        # Key 'q': quit
        elif key == 'q':
            self.left_value = 0.0
            self.right_value = 0.0
            self.publish_speed()
            self.running = False
            raise SystemExit

        # Store the last key pressed to check for consecutive inputs of the same movement type (linear or rotational)
        self.last_input = key
        # Publish the updated motor speeds
        self.publish_speed()
        # Print speeds to terminal (write raw since terminal is in raw mode)
        status = f'\r\nL: {self.left_value:.1f}  R: {self.right_value:.1f}\r\n'
        sys.stdout.write(status)
        sys.stdout.flush()

    # Publish the motor speed values to the 'set_motor_speeds' topic
    # The values are in the range -1 to 1, where 0 is stop, positive values are forward, and negative values are backward
    def publish_speed(self):
        msg = Float32MultiArray()
        msg.data = [self.left_value, self.right_value]
        self.speed_publisher.publish(msg)

    # Destroy the node when the emergency stop is triggered
    # msg is optional so this also works when called from shutdown (no args)
    def destroy_node(self, msg=None):
        self.running = False
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        time.sleep(0.1)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    teleop = Teleop()
    try:
        rclpy.spin(teleop)
    except SystemExit:
        pass
    finally:
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
