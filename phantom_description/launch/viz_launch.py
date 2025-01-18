from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
import os, xacro
"""
    This launchfile will bringup the robot using rviz2 to show 
    the urdf model. It will create two nodes
    - robot_state_publisher: 
        => take the joint_states and the urdf to publish the robot_description
    - joint_state_publisher 
        => Publish joint_states based in the urdf. 

"""
def generate_launch_description(): 
    pkg_path = get_package_share_directory('phantom_description')
    # First node call rviz 
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',  
        arguments=[
             '-d', os.path.join(pkg_path, 'config', 'rviz_px100.rviz')
         ]
        )
    
   

    # Now read the urdf 
<<<<<<< Updated upstream
    doc = xacro.process_file(os.path.join(pkg_path,'urdf','px_100','px100.urdf.xacro' ), mappings={})
=======
    doc = xacro.process_file(os.path.join(pkg_path,'urdf', 'turtlebot_arm.urdf.xacro' ), mappings={
        'phantom_x_100': 'false'
    })
    
>>>>>>> Stashed changes
    robot_desc = doc.toprettyxml(indent='  ')
    # urdfModelPath= os.path.join(pkg_path, 'urdf', 'turtlebot_arm.urdf')
    
    # with open(urdfModelPath,'r') as infp:
    #     robot_desc = infp.read()

    params = {
        'robot_description': robot_desc
    }
    
    robot_state_pub = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        parameters=[params]
    )

    joint_state_pub = Node(
        package='joint_state_publisher_gui', 
        executable='joint_state_publisher_gui'
        # parameters=[params]
    )


    nodes = [
        rviz,
        robot_state_pub,
        joint_state_pub
    ]

    return LaunchDescription(nodes)