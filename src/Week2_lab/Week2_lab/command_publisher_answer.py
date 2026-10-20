import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CommandPublisher(Node):

    def __init__(self):
        super().__init__('command_publisher')

        # Create a publisher that sends Twist messages
        # to the cmd_vel topic
        self.publisher_ = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Call timer_callback every 0.5 seconds
        timer_period = 0.5
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )

        self.get_logger().info('Command Publisher started')


    def timer_callback(self):

        # Create a Twist message
        msg = Twist()

        # -------------------------
        # Linear velocity
        # -------------------------
        # Positive x = move forward
        # Negative x = move backward

        msg.linear.x = 0.2
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        # -------------------------
        # Angular velocity
        # -------------------------
        # Positive z = turn left
        # Negative z = turn right

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        # Publish the velocity command
        self.publisher_.publish(msg)

        # Display the command in the terminal
        self.get_logger().info(
            f'Publishing -> Linear X: {msg.linear.x} m/s, '
            f'Angular Z: {msg.angular.z} rad/s'
        )


def main(args=None):

    # Start ROS 2
    rclpy.init(args=args)

    # Create the node
    command_publisher = CommandPublisher()

    try:
        # Keep the node running
        rclpy.spin(command_publisher)

    except KeyboardInterrupt:
        # Allows Ctrl+C to stop the program cleanly
        pass

    finally:
        # Stop the robot before shutting down
        stop_msg = Twist()
        command_publisher.publisher_.publish(stop_msg)

        # Destroy the node and shut down ROS 2
        command_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()