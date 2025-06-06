user@waffle5:~ $ rosrun image_transport republish compressed in:=/raspicam_node/image/compressed raw out:=/raspicam_node/image_raw
/opt/ros/kinetic/bin/rosrun: line 71: find: command not found
[rosrun] Couldn't find executable named republish below /opt/ros/kinetic/share/image_transport
/opt/ros/kinetic/bin/rosrun: line 78: find: command not found
user@waffle5:~ $ 



rosparam get /raspicam_node


rosparam list | grep raspicam_node


rosparam set /raspicam_node/enable_raw true



rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.025 image:=/raspicam_node/image_raw camera:=/raspicam_node

rosrun image_transport republish compressed in:=/raspicam_node/image/compressed raw out:=/raspicam_node/image_raw


# ROS & Turtlebot Setup

This project involves developing and testing autonomous robotic systems using ROS 1 (Noetic) and Turtlebot robots. Below is an outline to help you configure your development environment and connect to the Turtlebot hardware.

## 💻 System Requirements

- **OS**: Ubuntu 20.04 LTS (recommended for ROS Noetic)
- **ROS Version**: ROS Noetic
- **Recommended Tools**: 
  - `terminator` for managing terminals
  - `teleop-twist-keyboard` for manual robot control
  - `rqt_image_view` and `rviz` for visualization

## 🧰 ROS Installation Resources

- [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Helpful Setup Guide](https://varhowto.com/install-ros-noetic-ubuntu-20-04)

## 🐢 Turtlebot Setup

### Network Details

- **SSID**: `deec-robots`
- **Password**: `shakeytherobot`
- **IP Range**: `192.168.28.11` to `192.168.28.15`
- **Credentials**: `user:user`

### Connecting to the Turtlebot

```bash
ssh user@192.168.28.<Turtlebot IP>
roscore
```

Then, in a new terminal:

```bash
ssh user@192.168.28.<Turtlebot IP>
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### Laptop Configuration

Install required packages:

```bash
sudo apt-get install ros-noetic-dynamixel-sdk \
                     ros-noetic-turtlebot3-msgs \
                     ros-noetic-turtlebot3 \
                     ros-noetic-teleop-twist-keyboard
```

Add to `~/.bashrc`:

```bash
export TURTLEBOT3_MODEL=waffle_pi
export TURTLEBOT3_NAME=waffle4
export TURTLEBOT3_IP=192.168.28.<Turtlebot IP>
export TURTLEBOT3_NUMBER=<last IP digits>
export ROS_MASTER_URI=http://192.168.28.<Turtlebot IP>:11311
export ROS_HOSTNAME=192.168.28.<your IP>
export ROS_IP=192.168.28.<your IP>
```
Para guardar as alteraçoes do`~/.bashrc`:

```bash
source ~/.bashrc
```



### Verifying Communication

```bash
rostopic list
rostopic echo /odom
rostopic hz /scan
rostopic pub /cmd_vel geometry_msgs/Twist '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.5]' 
```

### Manual Control

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### Camera Access

On the Turtlebot:

```bash
roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch
```

On the laptop:

```bash
rosrun rqt_image_view rqt_image_view
```
## calibrar a câmara
```
rosrun camera_calibration cameracalibrator.py \
  --size 9x6 \
  --square 0.024 \
  image:=/raspicam_node/image_raw \
  camera:=/raspicam_node
```

## 📚 Useful Links

- [Turtlebot3 Bringup](http://wiki.ros.org/turtlebot3_bringup)
- [Laser Scanner Driver](http://wiki.ros.org/hls_lfcd_lds_driver)
- [Sensor Message Tutorials](http://wiki.ros.org/sensor_msgs/Tutorials)
- [Foxglove (Visualization Tool)](https://foxglove.dev/)
- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
