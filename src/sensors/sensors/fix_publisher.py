import time
import rclpy
import gpsd

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool

# This node reads and publishes GPS information for the robot.
# Using GPSD, the node interprets lat,long, and uncertainty data and publishes
# Publishers: 'get_GPS'
# Subscribers: 'emergency_stop'

class GPSFixPub(Node):
    def __init__(self):
        super().__init__('gps_fix_publisher')

        # Connect to the local gpsd daemon (gpsd-py3)
        gpsd.connect()

        # Timer to read GPS data
        self.create_timer(0.05, self.read_gpsd)

        # Publisher for GPS data
        # Message type: NavSatFix
        self.pub = self.create_publisher(NavSatFix, 'get_GPS', 10)

        # Subscription to emergency stop topic
        # If emergency stop is triggered, the node will be destroyed
        self.emergency_stop = self.create_subscription(
            Bool,
            'emergency_stop',
            self.destroy_node,
            10
        )

        # Creates fix object to be modified with each new data point
        self.fix = NavSatFix()
        self.fix.header.frame_id = 'fix_pub'

    
    # Read GPS on timer using gpsd-py3
    # gpsd.get_current() returns a GpsResponse object with mode/lat/lon and
    # position_precision() for horizontal/vertical error estimates.
    def read_gpsd(self):
        try:
            packet = gpsd.get_current()
        except Exception as e:
            self.get_logger().warn(f"Failed to read GPS: {e}")
            return

        # mode: 0=no mode, 1=no fix, 2=2D fix, 3=3D fix
        if packet.mode >= 2:
            self.fix.status.status = NavSatStatus.STATUS_FIX
            self.fix.latitude = packet.lat
            self.fix.longitude = packet.lon
            self.fix.header.stamp = self.get_clock().now().to_msg()

            # Use horizontal/vertical error precision if available
            try:
                xerr, yerr = packet.position_precision()
                self.fix.position_covariance[0] = float(xerr)
                self.fix.position_covariance[4] = float(yerr)
                self.fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            except Exception:
                self.fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            self.pub.publish(self.fix)
        else:
            self.fix.status.status = NavSatStatus.STATUS_NO_FIX
            self.fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

    # Destroy the node when the emergency stop is triggered
    def destroy_node(self,msg):
        time.sleep(0.1)
        super().destroy_node()

# ROS STARTUP
def main(args=None):
    rclpy.init(args=args)
    gps_pub = GPSFixPub()

    rclpy.spin(gps_pub)

    rclpy.shutdown()