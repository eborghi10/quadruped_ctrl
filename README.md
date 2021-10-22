# quadruped_robot

### MIT mini cheetah simulation in pybullet

MIT mini cheetah use customized simulator and lcm framework, which is not a popular way to do the robot development. Now, we extract the algorithm and do the simulation using ros and pybullet. This can be simple to deploy the system into different custom robot or plantform, and easy to learn the algorithm.

<img src="https://github.com/Derek-TH-Wang/quadruped_ctrl/blob/master/quadruped_balance.gif" alt="show" />
<img src="https://github.com/Derek-TH-Wang/quadruped_ctrl/blob/master/vision.png" alt="show" />
<img src="https://github.com/Derek-TH-Wang/quadruped_ctrl/blob/master/rviz.png" alt="show" />

### System requirements

Ubuntu 18.04, ROS Melodic

### Dependency

Use Logitech gamepad to control robot

```bash
git clone https://github.com/Derek-TH-Wang/gamepad_ctrl.git
```

### Build

```bash
cd {your workspace}
catkin make
source devel/setup.bash
```

### Terrain

You can modify the ```config/quadruped_ctrl_cinfig.yaml/terrain``` to deploy different terrains, there are four terrains supported in the simulator now, for example:

```no-lang
"plane"
"stairs"
"random1"
"random2"
"racetrack"
```

### Running

Run the gamepad node to control robot:

```bash
roslaunch gamepad_ctrl gamepad_ctrl.launch
```

Alternately, you can run control the robot with the keyboard:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Run the controller in simulator:

```bash
roslaunch quadruped_ctrl quadruped_ctrl.launch
```

switch the camera on / off:
camera set `True` or `False` in `config/quadruped_ctrl_config.yaml`, then launch the rviz to see the point cloud:

```bash
roslaunch quadruped_ctrl vision.launch
```

also can switch the gait type:

```bash
rosservice call /gait_type "cmd: 1"
```

gait type:

```no-lang
0:trot
1:bunding
2:pronking
3:random
4:standing
5:trotRunning
6:random2
7:galloping
8:pacing
9:trot (same as 0)
10:walking
11:walking2
```
