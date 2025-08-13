import launch
import os, sys, yaml
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ## LAUNCH ARGUMENTS
    #TODO: Note this doesn't work when passed from higher-level launch file
    launch_arg_device_id = DeclareLaunchArgument(
      'device_id', default_value='1'
    )

    launch_arg_device_role = DeclareLaunchArgument(
      'device_role', default_value='drone'
    )

    launch_arg_sim_phys = DeclareLaunchArgument(
      'env', default_value='phys'
    )
    
    #id = LaunchConfiguration('device_id')
    #device_role = LaunchConfiguration('device_role')

    env = 'phys'
    for arg in sys.argv:
        if arg.startswith("env:="):
            env = arg.split(":=")[1]

    device_role = 'drone'
    for arg in sys.argv:
        if arg.startswith("device_role:="):
            device_role = arg.split(":=")[1]

    id = '1'
    for arg in sys.argv:
        if arg.startswith("device_id:="):
            id = arg.split(":=")[1]
    
    
    # Get correct device ID and set corresponding namespace
    if device_role == 'drone':
        ns = PythonExpression(["'/px4_' + str(", id, ")"])
    elif device_role == 'load':
        ns = PythonExpression(["'/load_' + str(", id, ")"])


    ## GET PARAMETERS
    config = None

    if env=="sim":
      config = os.path.join(
        get_package_share_directory('multi_drone_slung_load'),
        'config',
        'sim.yaml'
        )
    elif env=="phys":
       config = os.path.join(
        get_package_share_directory('multi_drone_slung_load'),
        'config',
        'phys.yaml'
        ) 

    #print(f"Launching with device role: {device_role}, ID: {id}, Environment: {env}")

    # Set up launch description to launch logging node with arguments
    launch_description = [
        launch_arg_device_role,
        launch_arg_device_id,
        launch_arg_sim_phys,
        Node(
            package='highbay_vicon_px4',
            executable='ground_truth_to_px4',
            namespace=ns, #PythonExpression(["'/px4_' + str(", drone_id, ")"]),
            name='ground_truth_to_px4',
            output='screen',
            parameters=[config]
        )]

    ## LAUNCH
    return LaunchDescription(launch_description)

    