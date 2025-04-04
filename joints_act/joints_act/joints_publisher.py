import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class joints_publisher(Node):

    def __init__(self):
        super().__init__('joints_publisher')

        # Publisher
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Create a Timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

        # Initialise Message to be published
        self.ctrlJoints = JointState()
        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.name = ["wheel_r_joint", "wheel_l_joint"]  # Dos nombres separados como strings
        self.ctrlJoints.position = [0.0, 0.0]  # Dos posiciones
        self.ctrlJoints.velocity = [1.0, 1.0]  # Dos velocidades
        self.ctrlJoints.effort = [0.0, 0.0]  # Dos esfuerzos

        # Initialise Variables
        self.i = 0.0
        self.sign = 1

    # Timer Callback
    def timer_cb(self):
        time = self.get_clock().now().nanoseconds / 1e9

        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.position[0] = np.sin(time)
        self.ctrlJoints.position[1] = np.cos(time)  

        self.publisher.publish(self.ctrlJoints)

def main(args=None):
    rclpy.init(args=args)
    node = joints_publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()