import launch
import os, sys, yaml
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

DEFAULT_DEVICE_ROLE='drone'
DEFAULT_DEVICE_ID=1
#DEFAULT_ENV='phys'

def generate_launch_description():
    ## LAUNCH ARGUMENTS
    #TODO: Note this doesn't work when passed from higher-level launch file
    # launch_arg_drone_id = DeclareLaunchArgument(
    #   'drone_id', default_value=str(DEFAULT_DRONE_ID)
    # )

    vehicle_type_env = os.environ.get('DEVICE_ROLE', DEFAULT_DEVICE_ROLE) # Note must first set environment variable with export DEVICE_ROLE=drone
    launch_arg_device_role = DeclareLaunchArgument(
      'device_role', default_value=str(vehicle_type_env)
    )
    #vehicle_type = LaunchConfiguration('vehicle_type')

    # Get correct device ID and set corresponding namespace
    device_id_env = None

    if str(vehicle_type_env) == 'drone':
        device_id_env = os.environ.get('DRONE_ID', DEFAULT_DEVICE_ID) # Note must first set environment variable with export DRONE_ID=1
        device_id = LaunchConfiguration('device_id')

        ns = PythonExpression(["'/px4_' + str(", device_id, ")"])
    elif str(vehicle_type_env) == 'load':
        device_id_env = os.environ.get('LOAD_ID', DEFAULT_DEVICE_ID) # Note must first set environment variable with export LOAD_ID=1
        device_id = LaunchConfiguration('device_id')
        
        ns = PythonExpression(["'/load_' + str(", device_id, ")"])

    launch_arg_device_id = DeclareLaunchArgument(
    'device_id', default_value=str(device_id_env)
    )

    ## GET PARAMETERS
    config = None

    config = os.path.join(
    get_package_share_directory('multi_drone_slung_load'),
    'config',
    'phys.yaml'
    ) 

    # Set up launch description to launch logging node with arguments
    launch_description = [
        launch_arg_device_role,
        launch_arg_device_id,
        Node(
            package='highbay_vicon_px4',
            executable='highbay_to_px4',
            namespace=ns, #PythonExpression(["'/px4_' + str(", drone_id, ")"]),
            name='mocap_to_px4',
            output='screen',
            parameters=[config]
        )]

    ## LAUNCH
    return LaunchDescription(launch_description)

    