# Realsense Gazebo description

Description: This ROS 2 package is designed to be used in unison with [orbbec_description](https://github.com/locusrobotics/OrbbecSDK_ROS2/tree/locus) `locus` branch and allows for easy definition of cameras that can be simulated in Gazebo Fortress and newer. It may support other versions of Gazebo but this has not been tested.

## Running Example Launch

This package includes a launch file to start Gazebo, bridge the data to ROS 2 and display the simulated camera data in Rviz.
NOTE: this package does not [depend on Rviz](package.xml#L16) so you may need to install it.

Once you have built this package and sourced your workspace you can run
```bash
ros2 launch orbbec_gz_description example_335Le_gazebo.launch.py
```

> Note: you can specify `headless:=false` and it will also open the Gazebo GUI.

Which should start a simulated camera in Gazebo with a few objects in front of it. In the Rviz window that launches you can see the RGB images streaming along with the point cloud in the main view.


# Example Usage in URDF

In your robots urdf.xacro include the desired Realsense model along with its Gazebo description.
```xml
<xacro:include filename="$(find orbbec_description)/urdf/gemini335Le.urdf.xacro" />
<xacro:include filename="$(find orbbec_gz_description)/urdf/gemini335Le.gazebo.xacro" />

```
Then call the xacros and specify the same `name` and other optional arguments.
```xml
<!-- URDF xacro-->
<xacro:sensor_d415 parent="world" name="$(arg camera_name)" ...>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:sensor_d415>
<!-- Gazebo xacro-->
<xacro:gazebo_d415 name="$(arg camera_name)" .../>
```

> Note: Gazebo plugins can only be included once so the xacros in this repo assume that a parent will include/run the required Sensors plugin when starting simulation.
This plugin can be started from your URDF or world.sdf file.
```xml
<gazebo>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
</gazebo>
```

you can also refer to the the [example.urdf.xacro](./urdf/example_335Le_gazebo.urdf.xacro) included.

## Gazebo only features

Gazebo offers a triggered based RGB camera that can be enabled by passing `triggered="true"` to the Gazebo description xacro.
Currently this feature does not appear to not work with the RGBD sensor, but will hopefully be added soon.
Switching the camera to only `trigger` when requested allows developers to better control the computation load required by each sensor.
Note triggering is not a feature the Realsense hardware offers unfortunately.

To trigger the camera from the command line you can publish on the Gazebo topic `camera_name/trigger`.
```bash
ign topic -t "/name/trigger" -m Boolean -p "data: true" -n 1
```
