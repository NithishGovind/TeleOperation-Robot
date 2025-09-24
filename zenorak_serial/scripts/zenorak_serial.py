#!/usr/bin/env python3
# This script acts as a bridge between ROS 2 and an Arduino.
# It subscribes to the 'zenorak_teleop_cmd' topic and forwards
# the received string messages directly to the Arduino via serial.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # ROS 2 standard String message
import serial                   # PySerial library for communicating with Arduino


class TeleopBridge(Node):
    """
    A ROS 2 node that bridges teleoperation commands to Arduino.

    Workflow:
    1. Subscribe to the ROS 2 topic 'zenorak_teleop_cmd'.
    2. When a message arrives (e.g., "f25"), forward it to Arduino over serial.
    3. Arduino receives the command and drives motors accordingly.
    """
    
    def __init__(self):
        super().__init__('zenorak_serial')  # Name of the ROS 2 node

        # === Serial Setup ===
        # Attempt to connect to Arduino on /dev/ttyACM0 at 115200 baud.
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Connected to Arduino on /dev/ttyACM0")
        except Exception as e:
            # If connection fails, log an error but allow node to run
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.arduino = None  # Avoid crashing; allow graceful shutdown

        # === ROS Subscriber ===
        # Subscribe to 'zenorak_teleop_cmd' topic and call self.callback
        # every time a message is received.
        self.sub = self.create_subscription(
            String,               # Message type
            'zenorak_teleop_cmd', # Topic name
            self.callback,        # Callback function
            10                    # QoS history depth (number of messages to queue)
        )

    def callback(self, msg):
        """
        Called whenever a message is received on the subscribed topic.
        It extracts the command string and sends it to the Arduino.

        Parameters:
            msg: std_msgs.msg.String
        """
        command = msg.data.strip()  # Remove leading/trailing whitespace
        self.get_logger().info(f"Received teleop: {command}")  # Log for debugging

        if self.arduino:  # Only send if serial is connected
            try:
                # Send command to Arduino with a newline character.
                # Arduino's `readStringUntil('\n')` expects newline to know message end.
                self.arduino.write((command + "\n").encode())
                self.get_logger().info(f"Sent to Arduino: {command}")
            except Exception as e:
                # Catch errors during serial write (e.g., Arduino disconnected)
                self.get_logger().error(f"Error sending to Arduino: {e}")


def main(args=None):
    """
    Entry point for the ROS 2 node.
    Initializes ROS 2, creates the TeleopBridge node, spins it, and cleans up.
    """
    rclpy.init(args=args)        # Initialize ROS 2 Python client
    node = TeleopBridge()        # Instantiate our bridge node
    try:
        rclpy.spin(node)         # Keep node running, processing callbacks
    except KeyboardInterrupt:
        pass                     # Handle Ctrl+C gracefully
    finally:
        # Cleanup: close serial and destroy node
        if node.arduino:
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()  # Run the main function
