from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():
    
    pkg_dir = get_package_share_directory('phantom_coppelia')
    scene_path = os.path.join(pkg_dir, "scenes", "spheres.ttt")
    cmd_str = "/Applications/coppeliaSim.app/Contents/MacOS/coppeliaSim"
    
    # Make sure each part of the command is passed as a separate list element
    coppelia_node = ExecuteProcess(
        cmd=[[cmd_str, " -f", scene_path]],  # Correct command structure
        shell=True
    )
   
    return LaunchDescription([coppelia_node])