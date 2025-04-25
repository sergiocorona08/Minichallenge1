import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import transforms3d
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

        #Drone Initial Pose
        self.intial_pos_x = 1.0
        self.intial_pos_y = 1.0
        self.intial_pos_z = 0.0
        self.intial_pos_yaw = np.pi/2
        self.intial_pos_pitch = 0.0
        self.intial_pos_roll = 0.0

        #Angular velocity for the pose change and propellers
        self.omega = 0.5
        self.omega_prop = 50

        #Define Transformations
        self.define_TF()

        #Create Transform Boradcasters
        self.tf_br_base = TransformBroadcaster(self)
        self.tf_br_base_footprint = TransformBroadcaster(self)


    # Timer Callback
    def timer_cb(self):

        time = self.get_clock().now().nanoseconds / 1e9

        #TRASLACION COMPLETA DEL ROBOT
        #Create Trasnform Messages
        self.base_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_link_tf.transform.translation.x = self.intial_pos_x + 0.5*np.cos(self.omega*time) #MOVIMIENTO DE TRASLACION EN X
        self.base_link_tf.transform.translation.y = self.intial_pos_y + 0.5*np.sin(self.omega*time) #MOVIMIENTO DE TRASLACION EN Y
        self.base_link_tf.transform.translation.z = self.intial_pos_z    #MOVIMIENTO DE TRASLACION EN Z, EN ESTE CASO SE MANTIENE
        q = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw+self.omega*time)       
        self.base_link_tf.transform.rotation.x = q[1]
        self.base_link_tf.transform.rotation.y = q[2]
        self.base_link_tf.transform.rotation.z = q[3]
        self.base_link_tf.transform.rotation.w = q[0]

        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.position[0] = np.sin(time)
        self.ctrlJoints.position[1] = np.cos(time)  #ESTO SIMPLEMENTE MUEVE LAS RUEDAS

        self.tf_br_base.sendTransform(self.base_link_tf)

        self.publisher.publish(self.ctrlJoints)

    def define_TF(self):

        #Create Trasnform Messages
        self.base_link_tf = TransformStamped()
        self.base_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_link_tf.header.frame_id = 'odom'
        self.base_link_tf.child_frame_id = 'base_link'
        self.base_link_tf.transform.translation.x = self.intial_pos_x
        self.base_link_tf.transform.translation.y = self.intial_pos_y
        self.base_link_tf.transform.translation.z = self.intial_pos_z
        q = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw)       
        self.base_link_tf.transform.rotation.x = q[1]
        self.base_link_tf.transform.rotation.y = q[2]
        self.base_link_tf.transform.rotation.z = q[3]
        self.base_link_tf.transform.rotation.w = q[0]
 
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