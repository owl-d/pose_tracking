# pose_tracking

pose tracking using only IMU sensor

with kalman filter algorithm

test with teleop key input


```bash
[Arduino IDE] update
  1. 6DoF : orientation tracking with gyro, acc sensors
  2. 9DoF : orientation tracking with gyro, acc, mag sensors
```

```bash
$ roscore
```

```bash
#수동 컨트롤 : teleop 입력
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

#자동 컨트롤 : 시나리오 돌리기
$ roslaunch pose_tracking turtle_pose.launch scenario:=set_vel2.py

#USB 포트로 연결
$ rosrun rosserial_python serial_node.py __name:=opencr _port:=/dev/ttyACM0 _baud:=115200
```

```bash
$ rviz
```
