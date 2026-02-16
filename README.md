# gazebo_continuous_track

ROS 2 / Gazebo Harmonic port of continuous-track simulation plugins.

## Provided plugins

- `gazebo_continuous_track::ContinuousTrackSimpleSystem`
  - Library: `libgz_continuous_track_simple_system.so`
  - SDF schema: `<sprocket> + <track>`
- `gazebo_continuous_track::ContinuousTrackSystem`
  - Library: `libgz_continuous_track_system.so`
  - SDF schema: `<sprocket> + <trajectory>` (or `<track>` as fallback)
  - Note: pattern-based visual switching from the original ROS 1 implementation is not fully ported.

## Build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select gazebo_continuous_track
```

## Xacro macros

Existing macros are kept in `urdf_xacro/` and updated for ROS 2 / Gazebo Harmonic:

- `macros_track_simple_gazebo.urdf.xacro`
- `macros_track_gazebo.urdf.xacro`
- `macros_lugged_wheel_gazebo.urdf.xacro`
