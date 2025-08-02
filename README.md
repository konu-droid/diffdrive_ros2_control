# 🤖 DiffDrive ROS 2 Control Package

This package provides a `ros2_control` hardware interface, making it easy to connect various industrial motor drivers with your ROS 2 projects.

-----

## 🔌 Supported Motor Drivers

This package currently supports the following motor drivers:

  * **Roboteq**
  * **Moons Industries**
  * **ZlTech motors** - ZLAC8015

-----

## 📦 Getting Started

### 1\. Installation

1. Clone this repository into your ROS 2 workspace `src` folder:

   ```bash
   git clone https://github.com/konu-droid/diffdrive_ros2_control.git src/diffdrive_ros2_control
   ```

2. Build the workspace using `colcon`:

   ```bash
   colcon build
   ```

### 2\. Configure Serial Port Permissions

Your user needs permission to access the motor driver's serial port (e.g., `/dev/ttyUSB0`).

  * **Permanent (Recommended) Method:** For a permanent fix that works every time you plug in the device, set up **Udev rules**. See the [Udev Rules](https://www.google.com/search?q=%23-persistent-permissions-udev-rules) section below for an example.

  * **Temporary Method:** To grant permissions for your current session, run the following command. You may need to do this every time you reconnect the driver.

    ```bash
    # Replace ttyUSB0 with your actual device port
    sudo chmod 666 /dev/ttyUSB0
    ```

### 3\. Configure Your Robot

You must edit the configuration file to select your motor driver and specify its serial port.

✏️ **Edit this file:** [`diffbot.ros2_control.xacro`](https://www.google.com/search?q=/src/diffdrive_ros2_control/description/ros2_control/diffbot.ros2_control.xacro)

### 4\. Launch the Node

We've provided an example launch file with a simple URDF of a differential drive robot to get you started.

```bash
# Source your workspace
source install/setup.bash

# Run the launch file
ros2 launch diffdrive_ros2_control diffdrive.launch.py
```

-----

## 🔁 Topics

Once launched with the correct configuration, the package will expose the following standard ROS 2 topics for navigation.

### 📥 Subscriber

* `~/cmd_vel` — [`geometry_msgs/msg/TwistStamped`](http://docs.ros.org/en/foxy/api/geometry_msgs/msg/TwistStamped.html)

### 📤 Publishers

* `~/odom` — [`nav_msgs/msg/Odometry`](http://docs.ros.org/en/foxy/api/nav_msgs/msg/Odometry.html)
* `~/tf` — [`tf2_msgs/msg/TFMessage`](http://docs.ros.org/en/foxy/api/tf2_msgs/msg/TFMessage.html)

---


## 🕹️ Sending Commands

Try sending a velocity command with:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

Fire up `RViz2` to visualize the robot model and see the odometry being updated in real-time as your robot moves\!

-----


## 🛡️ Persistent Permissions (Udev Rules)

To avoid running `chmod` every time, you can create a `udev` rule. This will automatically grant the correct permissions to your motor driver whenever it's connected to the system.

An example rule file is provided in this repository. You can copy it to your system's rules directory:
[`udev.rules`](https://www.google.com/search?q=/irctc_udev.rules)

-----

## 📚 Additional Resources

This package uses the standard `diff_drive_controller`. For more details on its parameters and configuration options, check out the official documentation.

  * **[ROS 2 Control: diff\_drive\_controller](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)**

---

## 🤝 Contributing

We welcome contributions! If you have a new motor driver or an improvement, feel free to fork, improve, and create a pull request 🙌

---