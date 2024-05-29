# Controller Benchmark

Bechmarking scripts require the following python packages to be installed:

```
pip install transforms3d
pip install tabulate
```
Also, it may be convenient to install `turtlebot3_gazebo`, which contains turtlebot meshes. It's not required, but gazebo will look for them and take its time.
To avoid this, install it and remember to source the path in the terminal from which bringup is launched:
```bash
apt install ros-$ROS_DISTRO-turtlebot3_gazebo
# 
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models 
```
To use the suite, modify local parameter file `controller_benchmark.yaml` to include desired controller plugin.

Then execute the benchmarking:

- `ros2 launch ./controller_benchmark_bringup.py` to launch part of the nav2 stack.
- `python3 metrics.py --ros-args -p use_sim_time:=true`  to launch the benchmark script
- `python3 ./process_data.py` to take the metric files and process them into key results
