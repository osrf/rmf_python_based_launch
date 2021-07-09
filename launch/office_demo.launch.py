import launch
import launch_ros.actions
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    use_ignition = launch.actions.DeclareLaunchArgument(
        'use_ignition',
        default_value=['false'],
        description='Use Ignition for simulation instead of GazebO')
    gazebo_version = launch.actions.DeclareLaunchArgument(
        'gazebo_version',
        default_value=['11'],
        description='Version of Gazebo to use')
    use_sim_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value=['true'],
        description='Use the /clock topic for time to sync with simulation')
    failover_mode = launch.actions.DeclareLaunchArgument(
        'failover_mode',
        default_value=['false'],
        description='Enable failover mode for the fleet adapter')

    common = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([ThisLaunchFileDir(),
            '/common.launch.py']),
        launch_arguments={
            'use_sim_time':
                launch.substitutions.LaunchConfiguration('use_sim_time'),
            'viz_config_file': [
                FindPackageShare('rmf_demos'), '/include/office/office.rviz'],
            'config_file': [
                FindPackageShare('rmf_demos_maps'),
                '/office/office.building.yaml'],
            'dashboard_config_file': [
                FindPackageShare('rmf_demos_dashboard_resources'),
                    '/office/dashboard_config.json']}.items())

    simulation = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([ThisLaunchFileDir(),
            '/simulation.launch.py']),
        launch_arguments={
            'map_name': 'office',
            'use_ignition':
                launch.substitutions.LaunchConfiguration('use_ignition'),
            'gazebo_version':
                launch.substitutions.LaunchConfiguration('gazebo_version')
            }.items())

    fleet_name = 'tinyRobot'
    tiny_robot_fleet = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([ThisLaunchFileDir(),
            '/include/adapters/tinyRobot_adapter.launch.py']),
        launch_arguments={
            'fleet_name': fleet_name,
            'use_sim_time':
                launch.substitutions.LaunchConfiguration('use_sim_time'),
            'nav_graph_file': [
                FindPackageShare('rmf_demos_maps'),
                '/maps/office/nav_graphs/0.yaml']
            }.items())

    state_aggregator = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            #FindPackageShare('rmf_fleet_adapter'),
            ThisLaunchFileDir(),
            '/robot_state_aggregator.launch.py']),
        launch_arguments={
            'robot_prefix': fleet_name,
            'fleet_name': fleet_name,
            'use_sim_time':
                launch.substitutions.LaunchConfiguration('use_sim_time'),
            'failover_mode':
                launch.substitutions.LaunchConfiguration('failover_mode'),
            }.items())

    return launch.LaunchDescription([
        use_ignition,
        gazebo_version,
        use_sim_time,
        failover_mode,
        common,
        simulation,
        tiny_robot_fleet,
        state_aggregator,
        ])
