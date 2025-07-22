# DiffBot - Independent ros2_control Package

**DiffBot** (*Differential Mobile Robot*) is a complete, independent ros2_control implementation for differential drive robots. This package is fully self-contained with no external dependencies, making it perfect for learning, prototyping, and production use.

## ✅ **COMPLETELY INDEPENDENT PACKAGE**

- **No External Dependencies**: All URDF, materials, RViz configs included
- **Self-Contained**: Works without ros2_control_demo_description or any external packages  
- **Production Ready**: Complete implementation ready for real hardware integration

## Features

- ✅ **Complete ros2_control Implementation**: Hardware interface, controllers, and state management
- ✅ **Differential Drive Kinematics**: Proper implementation of diff drive control
- ✅ **Independent URDF System**: Custom robot description with materials
- ✅ **RViz Visualization**: Local RViz configurations for robot visualization
- ✅ **Odometry Publishing**: Accurate pose estimation and tracking
- ✅ **Command Interface**: TwistStamped message support for velocity commands
- ✅ **Joint State Broadcasting**: Real-time wheel position and velocity feedback
- ✅ **Educational Framework**: Clean, well-documented code for learning ros2_control

## Quick Start

### 1. Build the Package
```bash
cd /home/robot/robot_ws
colcon build --packages-select ros2_control_diffbot_original
source install/setup.bash
```

### 2. Launch Options

**View Robot (No Control):**
```bash
ros2 launch ros2_control_diffbot_original view_robot.launch.py
```

**Full Control System:**
```bash
ros2 launch ros2_control_diffbot_original diffbot.launch.py
```

This starts:
- Controller manager with DiffBot hardware interface
- Differential drive controller (`diffbot_base_controller`)
- Joint state broadcaster
- RViz visualization with robot model
- Robot state publisher

### 3. Control the Robot

**Combined Movement (Recommended):**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, 
    twist: {linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}}" -r 10
```

**Forward Movement:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, 
    twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" -r 10
```

**Pure Rotation:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, 
    twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}" -r 10
```

### 4. Monitor Robot State

**Joint States:**
```bash
ros2 topic echo /joint_states
```

**Odometry:**
```bash
ros2 topic echo /diffbot_base_controller/odom
```

**Controller Status:**
```bash
ros2 control list_controllers
```

**Hardware Interfaces:**
```bash
ros2 control list_hardware_interfaces
```

## System Architecture

### Hardware Interface
- **DiffBotSystemHardware**: Implements `hardware_interface::SystemInterface`
- **State Interfaces**: Position and velocity for each wheel joint
- **Command Interfaces**: Velocity commands for each wheel
- **Lifecycle Management**: Proper activation/deactivation sequences

### Controllers
- **diff_drive_controller**: Converts TwistStamped commands to wheel velocities
- **joint_state_broadcaster**: Publishes joint states for visualization

### Topics
- `/cmd_vel` (geometry_msgs/TwistStamped): Robot velocity commands
- `/diffbot_base_controller/odom` (nav_msgs/Odometry): Robot pose and twist
- `/joint_states` (sensor_msgs/JointState): Wheel positions and velocities
- `/tf` and `/tf_static`: Transform tree for visualization

## Independent Files Structure
```
ros2_control_diffbot_original/
├── bringup/
│   ├── config/diffbot_controllers.yaml    # Controller configuration
│   ├── launch/diffbot.launch.py          # Main launch file
│   └── rviz/                             # Independent RViz configs
│       ├── diffbot.rviz                  # Full system visualization
│       └── diffbot_view.rviz             # Robot view only
├── description/
│   ├── launch/view_robot.launch.py      # Simplified robot viewer
│   └── urdf/                            # Independent robot description
│       ├── diffbot_description.urdf.xacro  # Complete robot model
│       ├── diffbot.materials.xacro         # Independent materials
│       ├── diffbot.ros2_control.xacro      # Control interfaces
│       └── diffbot.urdf.xacro              # Main robot file
├── hardware/                            # Hardware interface implementation
│   ├── diffbot_system.cpp
│   └── include/ros2_control_diffbot_original/
│       └── diffbot_system.hpp
└── test/test_urdf_xacro.py              # URDF validation tests
```

## Performance Characteristics

- **Control Frequency**: 10Hz (configurable in diffbot_controllers.yaml)
- **Joint State Publishing**: 10Hz
- **Odometry Publishing**: 10Hz  
- **Command Processing**: Real-time response
- **Wheel Separation**: 0.4m (configurable)
- **Wheel Radius**: 0.08m (configurable)

## Integration with VESC Hardware

This package serves as a reference for implementing real hardware interfaces. In our workspace, we have:

- `diff_vesc_can_ros2_pkg_cpp`: Production VESC CAN communication
- `modular_diffbot_control`: Real robot control implementation

### Key Learnings for VESC Integration

1. **Hardware Interface Pattern**: Study `hardware/diffbot_system.cpp` for proper lifecycle management
2. **Command Processing**: See how `write()` method handles velocity commands
3. **State Publishing**: Understand how `read()` method updates joint states
4. **Controller Configuration**: Review the YAML configuration patterns
5. **Launch File Structure**: Analyze the launch file organization

## Monitoring and Debugging

### Check System Status
```bash
# List all nodes
ros2 node list

# Check controller status
ros2 control list_controllers

# Check hardware components
ros2 control list_hardware_components

# Monitor topics
ros2 topic list
ros2 topic hz /joint_states
ros2 topic hz /diffbot_base_controller/odom
```

### Troubleshooting

**Robot not moving:**
- Check if controllers are active: `ros2 control list_controllers`
- Verify command reception: `ros2 topic echo /cmd_vel`
- Monitor joint states: `ros2 topic echo /joint_states`

**RViz not showing robot:**
- Ensure robot_description is published: `ros2 topic echo /robot_description`
- Check transforms: `ros2 run tf2_tools view_frames`

## Educational Value

This package demonstrates:

1. **ros2_control Framework**: Complete implementation from hardware interface to visualization
2. **Differential Drive Kinematics**: Mathematical model to wheel commands
3. **ROS2 Best Practices**: Proper package structure, lifecycle management, and communication patterns
4. **Real-time Control**: High-frequency control loops and state updates
5. **Simulation to Reality**: Bridge between simulated and real hardware
6. **Independent Package Design**: Self-contained, no external dependencies

## Related Packages in Workspace

- **diff_vesc_can_ros2_pkg_cpp**: Real VESC hardware interface with CAN communication
- **modular_diffbot_control**: Production robot control system

## Next Steps

1. **Study the Code**: Understand the hardware interface implementation
2. **Modify Parameters**: Experiment with wheel radius, base width, etc.
3. **Extend Functionality**: Add sensors, implement custom controllers
4. **Hardware Integration**: Use this as a template for real VESC implementation

## Documentation References

- [ros2_control Documentation](https://control.ros.org/)
- [diff_drive_controller Documentation](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [Writing a Hardware Interface](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html)

---

**Status**: ✅ Fully functional, tested, and completely independent  
**Dependencies**: None - completely self-contained package  
**Ready for**: Learning, prototyping, and production hardware integration
