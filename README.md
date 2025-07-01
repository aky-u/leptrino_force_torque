# leptrino_force_torque

Leptrino 6 DOF force-torque sensors (https://www.leptrino.co.jp/product/6axis-force-sensor)

## Connection

Connect the force sensor, serial converter and your PC. Pay attention to the SN numbers of sensor and converter, they should be the same.

Check your device name and give it a permission.

```bash
chmod 666 /dev/ttyACM0
```

## Build

In your workspace, build the package using the following command.

```bash
colcon build --symlink-install
```

## Node

Launch ROS 2 node using the following command.

```bash
ros2 launch leptrino_force_torque leptrino.launch.py comport:=/dev/ttyACM0
```

## ros2 control

This package also provides `ros2_control` hardware interface. See the example below.

```xml
<?xml version="1.0"?>
<robot name="leptrino_example" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Main URDF -->
  <link name="tool_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius=".08" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius=".08" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <ros2_control name="leptrino_force_torque" type="sensor">
    <hardware>
      <plugin>leptrino_force_torque/LeptrinoForceTorqueSensor</plugin>
      <param name="calib_len">100</param>
    </hardware>

    <sensor name="left_leptrino_fts" type="sensor">
      <state_interface name="force.x"/>
      <state_interface name="force.y"/>
      <state_interface name="force.z"/>
      <state_interface name="torque.x"/>
      <state_interface name="torque.y"/>
      <state_interface name="torque.z"/>
      <param name="frame_id">left_tool_link</param>
      <param name="fx_range">50</param>
      <param name="fy_range">50</param>
      <param name="fz_range">50</param>
      <param name="tx_range">0.5</param>
      <param name="ty_range">0.5</param>
      <param name="tz_range">0.5</param>
    </sensor>

  </ros2_control>
</robot>
```