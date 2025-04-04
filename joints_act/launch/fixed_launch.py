import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
#Todo EL PATH A DONDE ESTA LA UBICACION
    urdf_file_name = 'fixed_ex.urdf'

    urdf = os.path.join(
        get_package_share_directory('joints_act'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    robot_state_pub_node = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'robot_description': robot_desc}],
                            arguments=[urdf]
                            )
    

    Mydynamic_transform_node = Node(
                            name='joints_publisher',
                            package='joints_act',
                            executable='joints_publisher',
                            output='screen',
                            )


    l_d = LaunchDescription([
                              robot_state_pub_node,
                              Mydynamic_transform_node])

    return l_d