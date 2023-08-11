ros action相关package创建流程与注意点

1. ament_python与ament_cmake编译方式的不同

   ```
   #ament_cmake 需要
   CMakeLists.txt

   package.xml


     <buildtool_depend>ament_cmake</buildtool_depend>
     <buildtool_depend>rosidl_default_generators</buildtool_depend>
     <exec_depend>builtin_interfaces</exec_depend>
     <exec_depend>rosidl_default_runtime</exec_depend>
     <member_of_group>rosidl_interface_packages</member_of_group>
     <export>
       <build_type>ament_cmake</build_type>
     </export>



   ```

   ```
   #ament_python需要

   package.xml
   setup.py
   setup.cfg


     <depend>rclpy</depend>
     <depend>std_msgs</depend>
     <depend>navigation_action_msg</depend>
     <export>
       <build_type>ament_python</build_type>
     </export>



   ```
2. setup.cfg  与setup.py
   setup.cfg 注意包名称

   ```

   [develop]
   script_dir=$base/lib/navigation_action
   [install]
   install_scripts=$base/lib/navigation_action




   ```

   setup.py 注意包名称， 文件路径 和python文件的执行点entry_points
   python文件与ros执行程序的配置

   ```
    #!/usr/bin/env python3

    import glob
    import os

    from setuptools import find_packages
    from setuptools import setup

    package_name = 'navigation_action'
    share_dir = 'share/' + package_name

    setup(
        name=package_name,
        version='0.6.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            (share_dir, ['package.xml']),
            (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
            (share_dir + '/param', glob.glob(os.path.join('param', '*.yaml'))),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        author='Pyo, Darby Lim',
        author_email='passionvirus@gmail.com, routiful@gmail.com',
        maintainer='Pyo',
        maintainer_email='passionvirus@gmail.com',
        keywords=['ROS'],
        classifiers=[
            'Intended Audience :: Developers',
            'License :: OSI Approved :: Apache Software License',
            'Programming Language :: Python',
            'Topic :: Software Development',
        ],
        description='ROS 2 rclpy example package for the topic, service, action',
        license='Apache License, Version 2.0',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'action_client = script.action_client:main',
                'action_server = script.action_server:main',            
            ],
        },
    )
   ```
3. 文件夹下面加入 ____init____.py 空文件
4. launch文件与参数文件配置

```
    import os

    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node


    def generate_launch_description():
        param_dir = LaunchConfiguration(
            'param_dir',
            default=os.path.join(
                get_package_share_directory('navigation_action'),
                'param',
                'test.yaml'))

        return LaunchDescription([
            DeclareLaunchArgument(
                'param_dir',
                default_value=param_dir,
                description='Full path of parameter file'),

            Node(
                package='navigation_action',
                executable='action_client',
                name='action_client',
                parameters=[param_dir],
                output='screen'),
        ])



```

```
在ROS 2的launch_ros.actions.Node中，Node类用于描述一个要启动的ROS节点。以下是一些常用的参数及其简要描述：

    package (必需):
        描述：要从中运行节点的ROS包的名称。
        类型：字符串

    executable (必需):
        描述：要运行的可执行文件的名称。
        类型：字符串

    name:
        描述：启动的节点的名称。
        类型：字符串

    namespace:
        描述：节点的命名空间。
        类型：字符串

    output:
        描述：指定节点的输出应该如何处理。例如，可以设置为'screen'以将输出打印到控制台。
        类型：字符串

    parameters:
        描述：一个或多个参数文件的路径，或直接是参数字典。
        类型：列表或字典

    remappings:
        描述：主题、服务和参数的重新映射。
        类型：列表

    arguments:
        描述：传递给可执行文件的任何额外命令行参数。
        类型：列表

    env:
        描述：设置环境变量。
        类型：字典

    cwd:
        描述：设置节点的当前工作目录。
        类型：字符串

    cmd:
        描述：完整的命令行，用于替代包和可执行文件的组合。
        类型：列表

    launch-prefix:
        描述：在启动节点之前，可以在命令行前加上一个前缀，例如使用gdb或valgrind进行调试。
        类型：字符串

    respawn:
        描述：如果节点退出，则自动重新启动它。
        类型：布尔值

    respawn_delay:
        描述：如果respawn=True，此值指定在重新启动之前等待的秒数。
        类型：浮点数

    emulate_tty:
        描述：模拟TTY输出模式，这有助于彩色化日志输出。
        类型：布尔值

```

5. 带参数项的launch程序配置参数解析

 注意sys.argv具体包含的参数， 正常ros2 run topic_service_action_rclpy_example checker -g 50

 换roslaunch的方式会带有（--ros-args -r）等更多的参数， 通过argv=sys.argv[1:3]限制
 checker --goal_total_sum 50 --ros-args -r __node:=checker --params-file /home/robot/ros2_ws/action_demo_ws/install

```
import argparse
import sys

import rclpy

from topic_service_action_rclpy_example.checker.checker import Checker

def main(argv=sys.argv[1:3]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-g',
        '--goal_total_sum',
        type=int,
        default=50,
        help='Target goal value of total sum')  
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')
    args = parser.parse_args(args=argv)
    print("+++args.argv::: ", args)

    rclpy.init(args=args.argv)
    try:
        checker = Checker()
        checker.send_goal_total_sum(args.goal_total_sum)
        try:
            rclpy.spin(checker)
        except KeyboardInterrupt:
            checker.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            checker.arithmetic_action_client.destroy()
            checker.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()




```

参考https://github.com/robotpilot/ros2-seminar-examples.git
