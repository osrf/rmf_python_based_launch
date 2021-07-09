import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackagePrefix
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    map_package = launch.actions.DeclareLaunchArgument(
        'map_package',
        default_value=['rmf_demos_maps'],
        description='Name of the map package')
    map_name = launch.actions.DeclareLaunchArgument(
        'map_name',
        description='Name of the rmf_demos map to simulate')
    use_ignition = launch.actions.DeclareLaunchArgument(
        'use_ignition',
        default_value=['false'],
        description='Use Ignition for simulation instead of GazebO')
    use_crowdsim = launch.actions.DeclareLaunchArgument(
        'use_crowdsim',
        default_value=['false'],
        description='Use Crowdsim to simulate crowds')

    gazebo_version = launch.actions.DeclareLaunchArgument(
        'gazebo_version',
        default_value=['11'],
        description='Version of Gazebo to use')
    world_path = launch.actions.SetLaunchConfiguration('world_path', [
        FindPackageShare(launch.substitutions.LaunchConfiguration('map_package')),
        '/maps/',
        launch.substitutions.LaunchConfiguration('map_name'),
        '/',
        launch.substitutions.LaunchConfiguration('map_name'),
        '.world'
        ])
    model_path = launch.actions.SetLaunchConfiguration('model_path', [
        FindPackageShare(launch.substitutions.LaunchConfiguration('map_package')),
        '/maps/',
        launch.substitutions.LaunchConfiguration('map_name'),
        '/models:',
        FindPackageShare('rmf_demos_assets'),
        '/models:/usr/share/gazebo-',
        launch.substitutions.LaunchConfiguration('gazebo_version'),
        '/models',
        ])
    resource_path = launch.actions.SetLaunchConfiguration('resource_path', [
        FindPackageShare('rmf_demos_assets'),
        ':/usr/share/gazebo-',
        launch.substitutions.LaunchConfiguration('gazebo_version'),
        ])
    plugin_path = launch.actions.SetLaunchConfiguration('plugin_path', [
        FindPackagePrefix('rmf_robot_sim_gazebo_plugins'),
        '/lib:',
        FindPackagePrefix('rmf_building_sim_gazebo_plugins'),
        '/lib:/usr/share/gazebo-',
        launch.substitutions.LaunchConfiguration('gazebo_version'),
        ])

    menge_resource_path = launch.actions.SetLaunchConfiguration(
        'menge_resource_path',
        '')

    gazebo_command = [launch.substitutions.FindExecutable(name='gzserver'),
        '--verbose',
        '-s', 'libgazebo_ros_factory.so',
        '-s', 'libgazebo_ros_init.so',
        launch.substitutions.LaunchConfiguration('world_path')]
    gazebo = launch.actions.ExecuteProcess(cmd=gazebo_command, shell=True,
        additional_env={
            'GAZEBO_MODEL_PATH':
                launch.substitutions.LaunchConfiguration('model_path'),
            'GAZEBO_RESOURCE_PATH':
                launch.substitutions.LaunchConfiguration('resource_path'),
            'GAZEBO_PLUGIN_PATH':
                launch.substitutions.LaunchConfiguration('plugin_path'),
            'GAZEBO_MODEL_DATABASE_URI': '',
            'MENGE_RESOURCE_PATH':
                launch.substitutions.LaunchConfiguration('menge_resource_path')
            }
        )

    gazebo_client_command = [
        launch.substitutions.FindExecutable(name='gzclient'),
        '--verbose',
        launch.substitutions.LaunchConfiguration('world_path')
        ]
    gazebo_client = launch.actions.ExecuteProcess(
        cmd=gazebo_client_command,
        additional_env={
            'GAZEBO_MODEL_PATH':
                launch.substitutions.LaunchConfiguration('model_path'),
            'GAZEBO_RESOURCE_PATH':
                launch.substitutions.LaunchConfiguration('resource_path'),
            'GAZEBO_PLUGIN_PATH':
                launch.substitutions.LaunchConfiguration('plugin_path'),
            }
        )

    return launch.LaunchDescription([
        map_package,
        map_name,
        use_ignition,
        use_crowdsim,
        gazebo_version,
        world_path,
        model_path,
        resource_path,
        plugin_path,
        menge_resource_path,
        gazebo,
        gazebo_client
    ])
