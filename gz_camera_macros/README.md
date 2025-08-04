# Realsense Gazebo Camera Macro's

### Example Usage in URDF

In your robot or camera's urdf.xacro include the desired macro.
```xml
<xacro:include filename="$(find gz_camera_macros)/urdf/rgb_camera.gazebo.xacro" />
<xacro:include filename="$(find gz_camera_macros)/urdf/rgbd_camera.gazebo.xacro" />

```
Then call the desired macro and specify the same `name` and camera properties.
```xml
<xacro:if value="${type == 'rgbd'}">
        <xacro:gazebo_rgbd name="${name}"
                           fps="${fps}"
                           gz_topic_name="${gz_topic_name}"
                           image_width="${image_width}"
                           image_height="${image_height}"
                           h_fov="${realsense_h_fov}"
                           v_fov="${realsense_v_fov}"
                           min_depth="${min_depth}"
                           max_depth="${max_depth}"/>
    </xacro:if>
    <xacro:if value="${type == 'rgb'}">
        <xacro:gazebo_rgb name="${name}"
                        fps="${fps}"
                        gz_topic_name="${gz_topic_name}"
                        image_width="${image_width}"
                        image_height="${image_height}"
                        h_fov="${realsense_h_fov}"
                        v_fov="${realsense_v_fov}"
                        triggered="${triggered}"/>
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

you can also refer to the the [example.urdf.xacro](./urdf/example_d415_gazebo.urdf.xacro) included.

## Triggering RGB camera

Gazebo offers a triggered based RGB camera that can be enabled by passing `triggered="true"` to the Gazebo description xacro.
Currently this feature does not appear to not work with the RGBD sensor, but will hopefully be added soon.
Switching the camera to only `trigger` when requested allows developers to better control the computation load required by each sensor.
Note triggering on some hardware plarforms is not available.

To trigger the camera from the command line you can publish on the Gazebo topic `camera_name/trigger`.
```bash
ign topic -t "/name/trigger" -m Boolean -p "data: true" -n 1
```
