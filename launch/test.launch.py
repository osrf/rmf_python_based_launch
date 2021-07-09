import sys

import launch
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def main(argv=sys.argv[1:]):
    ld = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                'office_demo.launch.py'
            ]),
            launch_arguments={
                'map_name': 'office',
                'use_ignition': 'false',
                'gazebo_version': '11',
            }.items()
        )
    ])

    print('Launch introspection:')
    print('')
    print(launch.LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Launching')
    print('')

    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    sys.exit(main())
