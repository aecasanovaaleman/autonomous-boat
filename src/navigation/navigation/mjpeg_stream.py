import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

# This node subscribes to 'camera/image/compressed' and serves the frames
# as an MJPEG stream over HTTP on port 8080 at /stream.
# No rosbridge or web_video_server required.
#
# Publishers: ''
# Subscribers: 'camera/image/compressed'

HTTP_PORT = 8080

# Shared state for the latest JPEG frame
_frame_lock = threading.Lock()
_latest_frame = None


class MJPEGStreamHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'close')
            self.end_headers()

            while True:
                with _frame_lock:
                    frame = _latest_frame

                if frame is not None:
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(frame)}\r\n'.encode())
                        self.wfile.write(b'\r\n')
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                        self.wfile.flush()
                    except BrokenPipeError:
                        break
                    except ConnectionResetError:
                        break

                # Serve at roughly 10 FPS to match the camera publisher rate
                threading.Event().wait(0.1)
        else:
            self.send_response(404)
            self.end_headers()

    # Suppress default request logging to avoid cluttering the terminal
    def log_message(self, format, *args):
        pass


class MJPEGStreamNode(Node):
    def __init__(self):
        super().__init__('mjpeg_stream')

        self.image_sub = self.create_subscription(
            CompressedImage,
            'camera/image/compressed',
            self.image_callback,
            10
        )

        # Start the HTTP server in a daemon thread
        self.server = HTTPServer(('0.0.0.0', HTTP_PORT), MJPEGStreamHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()

        self.get_logger().info(f'MJPEG stream serving on http://0.0.0.0:{HTTP_PORT}/stream')

    def image_callback(self, msg):
        global _latest_frame
        with _frame_lock:
            _latest_frame = bytes(msg.data)

    def destroy_node(self):
        self.server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MJPEGStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down MJPEG stream')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
