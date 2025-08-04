# Gazebo Camera Descriptions

[![Format Status](https://github.com/locusrobotics/gz_camera_descriptions/actions/workflows/format.yaml/badge.svg)](https://github.com/locusrobotics/gz_camera_descriptions/actions/workflows/format.yaml)

### Getting Starteed

This package assumes that you want to simulate a camera with Gazebo. In the [package.xml](gz_camera_macros/package.xml) of the generic macros or even a specific camera like the [Realsense](../gz_camera_descriptions/realsense2_gz_description/package.xml) requires you have Gazebo installed.

In your robots URDF....

1. Include and run your camera's description xacro:
```xml
<xacro:include filename="$(find realsense2_description)/urdf/_d415.urdf.xacro" />
...
<xacro:sensor_d415 parent="my_robots_camera_mount_link" name="$(arg camera_name)">
  <origin xyz="0 0 0" rpy="0 0 0"/> <!-- This robot's camera mount is aligned with the camera base -->
</xacro:sensor_d415>
```

2. Selectivly include Gz xacro with `if(sim_gazebo)` if you don't want the definition included when running with hardware:
```xml
<xacro:if value="${sim_gazebo}">
  <xacro:include filename="$(find realsense2_gz_description)/urdf/_d415.gazebo.xacro" />
  <xacro:gazebo_d415 name="$(arg camera_name)" gz_topic_name="$(arg camera_name)" type="rgbd" fps="15"/>
</xacro:if>
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

3. When simulatting add to your robot's Gazebo launch file to start and configure the bridge `Node` so that you can see the simulated data as ROS topics:
```python
# Bridge
gazebo_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    parameters=[{"use_sim_time": True}],
    arguments=[
        "/wrist_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/wrist_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/wrist_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        "/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    ],
    remappings=[
        (
            "/wrist_camera/image",
            "/wrist/camera/color/image_raw",
        ),
        (
            "/wrist_camera/depth_image",
            "/wrist/camera/depth_registered/image_rect",
        ),
        (
            "/wrist_camera/points",
            "/wrist/camera/depth/color/points",
        ),
        (
            "/wrist_camera/camera_info",
            "/wrist/camera/color/camera_info",
        ),
        (
            "/wrist_camera/camera_info",
            "/wrist/camera/depth_registered/camera_info",
        )
    ],
    output="screen",
)
```

## Package Documentation:

- [Generic Gazebo Camera macros](gz_camera_macros/README.md)
- [Realsense2 Gazebo Camera macros](realsense2_gz_description/README.md)

## Contributing

pre-commit is a tool to automatically run formatting checks on each commit, which saves you from manually running them.
This repo requires formatting to pass before changes will be accepted.

Install pre-commit like this:

```
pip3 install pre-commit
```

Run this in the top directory of the repo to set up the git hooks:

```
pre-commit install
```
