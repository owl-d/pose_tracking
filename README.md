# pose_tracking

pose tracking using only IMU sensor

with kalman filter algorithm

test with teleop key input


1. 6DoF : orientation tracking with gyro, acc sensors

2. 9DoF : orientation tracking with gyro, acc, mag sensors


```bash
[Arduino IDE] update
```

```bash
$ roscore
```

```bash
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

```bash
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

```bash
$ rviz
```
