# ros2_control_diffbot_original package from ros2_control


**DiffBot** (*Differential Mobile Robot*) is a simple mobile base with differential drive kinematics. The robot is essentially a box that moves according to differential drive principles, making it perfect for learning and prototyping.

## Features

- ✅ **Complete ros2_control Implementation**: Hardware interface, controllers, and state management
- ✅ **Differential Drive Kinematics**: Proper implementation of diff drive control
- ✅ **RViz Visualization**: Real-time robot visualization and control
- ✅ **Odometry Publishing**: Accurate pose estimation and tracking
- ✅ **Command Interface**: TwistStamped message support for velocity commands
- ✅ **Joint State Broadcasting**: Real-time wheel position and velocity feedback
- ✅ **Educational Framework**: Clean, well-documented code for learning ros2_control

## Architecture

### Hardware Interface
- **DiffBotSystemHardware**: Implements `hardware_interface::SystemInterface`
- **State Interfaces**: Position and velocity for each wheel joint
- **Command Interfaces**: Velocity commands for each wheel
- **Lifecycle Management**: Proper activation/deactivation sequences

### Controllers
- **diff_drive_controller**: Converts Twist commands to wheel velocities
- **joint_state_broadcaster**: Publishes joint states for visualization

### Topics
- `/cmd_vel` (geometry_msgs/TwistStamped): Robot velocity commands
- `/diffbot_base_controller/odom` (nav_msgs/Odometry): Robot pose and twist
- `/joint_states` (sensor_msgs/JointState): Wheel positions and velocities
- `/tf` and `/tf_static`: Transform tree for visualization

## Quick Start

### 1. Build the Package
```bash
cd /home/robot/robot_ws
colcon build --packages-select ros2_control_diffbot_original
source install/setup.bash
```

### 2. Launch the Demo
```bash
ros2 launch ros2_control_diffbot_original diffbot.launch.py
```

This will start:
- Controller manager
- Hardware interface (simulated)
- Differential drive controller
- Joint state broadcaster
- RViz visualization
- Robot state publisher

### 3. Control the Robot

**Forward Movement:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, 
    twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" -r 10
```

**Rotation:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, 
    twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}" -r 10
```

**Combined Movement (Arc):**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, 
    twist: {linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}}" -r 10
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

## Code Structure

```
ros2_control_diffbot_original/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package dependencies
├── README.md                   # This file
├── hardware/
│   ├── diffbot_system.cpp      # Hardware interface implementation
│   └── include/
│       └── ros2_control_diffbot_original/
│           └── diffbot_system.hpp
├── bringup/
│   ├── launch/
│   │   ├── diffbot.launch.py   # Main launch file
│   │   └── view_robot.launch.py
│   └── config/
│       └── diffbot_controllers.yaml
├── description/
│   └── urdf/
│       ├── diffbot_description.urdf.xacro
│       └── diffbot.ros2_control.xacro
└── ros2_control_diffbot_original.xml  # Plugin declaration
```

## Educational Value

This package demonstrates:

1. **ros2_control Framework**: Complete implementation from hardware interface to visualization
2. **Differential Drive Kinematics**: Mathematical model to wheel commands
3. **ROS2 Best Practices**: Proper package structure, lifecycle management, and communication patterns
4. **Real-time Control**: High-frequency control loops and state updates
5. **Simulation to Reality**: Bridge between simulated and real hardware

## Monitoring and Debugging

### Check System Status
```bash
# List all nodes
ros2 node list

# Check controller status
ros2 control list_controllers

# Monitor topics
ros2 topic list
ros2 topic hz /joint_states
ros2 topic hz /diffbot_base_controller/odom
```

### Performance Metrics
- **Control Frequency**: 100Hz (configurable)
- **Joint State Publishing**: 100Hz
- **Odometry Publishing**: 100Hz
- **Command Processing**: Real-time response

## Related Packages in Workspace

- **diff_vesc_can_ros2_pkg_cpp**: Real VESC hardware interface with CAN communication
- **modular_diffbot_control**: Production robot control system
- **ros2_control_demo_description**: Robot URDF descriptions

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

**Status**: ✅ Fully functional and tested  
