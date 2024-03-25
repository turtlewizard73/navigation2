# #! /usr/bin/env python3
# Copyright 2022 Enrico Sutera
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    benchmark_dir = os.getcwd()
    #metrics_py = os.path.join(benchmark_dir, 'metrics.py')
    config = os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')
    benchmark_config = os.path.join(benchmark_dir, 'controller_benchmark.yaml')
    map_file = os.path.join(benchmark_dir,'maps/10by10_empty.yaml')
    lifecycle_nodes = ['map_server', 'planner_server', 'controller_server']

    static_transform_one = Node(
         package = 'tf2_ros',
         executable = 'static_transform_publisher',
         output = 'screen',
         parameters=[{'use_sim_time': True}],
         arguments = ["0", "0", "0", "0", "0", "0", "odom", "map"])

    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'yaml_filename': map_file},
                    {'topic_name': "map"}])

    start_planner_server_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[config,
                    {'use_sim_time': True}])

    start_controller_server_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[config,benchmark_config,
                    {'use_sim_time': True}], )

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}])

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '',
                          'ues_sim_time' : 'True',
                          'use_namespace': 'False'}.items())

    world_dir = os.path.join(benchmark_dir,'worlds')
    world_path = os.path.join(world_dir,'maze_5x5.world')
    

    gazebo_models_path = os.path.join(benchmark_dir,'models')
    try:
        os.environ["GAZEBO_MODEL_PATH"] = os.environ["GAZEBO_MODEL_PATH"] + ":" + gazebo_models_path
    except:
        os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    tb3_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(benchmark_dir, 'tb3_simulation_only.launch.py')),
        launch_arguments={'namespace': '',
                          'use_sim_time' : 'True',
                          'use_rviz' : 'False',
                          'headless' : 'False', #Will be true
                          'world' : world_path
                          }.items())
                          
    metrics_cmd = ExecuteProcess(
       cmd=['python3', 'metrics.py',
             '--ros-args', '-p', 'use_sim_time:=true',
             '-p' ,'map_name:=maze_5x5'],
       cwd=[benchmark_dir], output='screen')

    ld = LaunchDescription()
    ld.add_action(static_transform_one)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_planner_server_cmd)
    ld.add_action(start_controller_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(tb3_cmd)
    ld.add_action(metrics_cmd)
    return ld
