import launch
import os, sys, yaml
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

DEFAULT_DRONE_ID=1
#DEFAULT_ENV='phys'

def generate_launch_description():
    ## LAUNCH ARGUMENTS
    #TODO: Note this doesn't work when passed from higher-level launch file
    launch_arg_drone_id = DeclareLaunchArgument(
      'drone_id', default_value=str(DEFAULT_DRONE_ID)
    )
    # launch_arg_sim_phys = DeclareLaunchArgument(
    #   'env', default_value=str(DEFAULT_ENV)
    # )

    # Get arguments  
    drone_id = LaunchConfiguration('drone_id')

    ## GET PARAMETERS
    config = None

    config = os.path.join(
    get_package_share_directory('multi_drone_slung_load'),
    'config',
    'phys.yaml'
    ) 

    # Set up launch description to launch logging node with arguments
    launch_description = [
        launch_arg_drone_id,
        Node(
            package='highbay_vicon_px4',
            executable='highbay_to_px4',
            namespace=PythonExpression(["'/px4_' + str(", drone_id, ")"]),
            name='mocap_to_px4',
            output='screen',
            parameters=[config]
        )]

    ## LAUNCH
    return LaunchDescription(launch_description)

    