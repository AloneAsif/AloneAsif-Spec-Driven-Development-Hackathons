# Humanoid Model Loading Process (URDF/USD Formats)

## Overview

Loading humanoid models in Isaac Sim requires understanding both URDF (Universal Robot Description Format) and USD (Universal Scene Description) formats. This guide covers the process of importing, converting, and configuring humanoid robots for simulation.

## Understanding Humanoid Model Formats

### URDF Format
URDF is an XML-based format that describes robot structure:
- Links: Rigid bodies with mass and collision properties
- Joints: Constraints connecting links
- Materials: Visual appearance
- Transmissions: Motor interface definitions
- Gazebo extensions: Simulation-specific parameters

### USD Format
USD is a more comprehensive format that supports:
- Hierarchical scene composition
- Multiple representations (visual, collision, physics)
- Material definitions
- Animation and motion data
- Variant sets for different configurations

## Method 1: Direct USD Model Loading

### Using Existing USD Models
If your humanoid model is already in USD format:

- **Verify Model Structure**:
   ```bash
   # Check USD file structure
   usdview humanoid_model.usd
   ```

- **Load in Isaac Sim**:
   ```python
   import omni
   from omni.isaac.core import World
   from omni.isaac.core.utils.stage import add_reference_to_stage
   from omni.isaac.core.utils.nucleus import get_assets_root_path

   # Add reference to humanoid model
   add_reference_to_stage(
       usd_path="/path/to/humanoid_model.usd",
       prim_path="/World/HumanoidRobot"
   )
   ```

- **Configure Articulation**:
   ```python
   from omni.isaac.core.articulations import Articulation
   from omni.isaac.core.utils.prims import get_prim_at_path

   # Get the robot prim and configure as articulation
   world = World(stage_units_in_meters=1.0)
   humanoid_robot = world.scene.add(
       Articulation(
           prim_path="/World/HumanoidRobot",
           name="humanoid_robot",
           position=[0, 0, 1.0],  # Start 1m above ground
           orientation=[1, 0, 0, 0]
       )
   )
   ```

## Method 2: URDF to USD Conversion

### Using Isaac Sim's URDF Importer

- **Prepare URDF File**:
   Ensure your URDF file is properly structured:
   ```xml
   <!-- Example humanoid URDF snippet -->
   <?xml version="1.0"?>
   <robot name="humanoid_robot">
     <link name="base_link">
       <visual>
         <geometry>
           <capsule length="0.2" radius="0.1"/>
         </geometry>
       </visual>
       <collision>
         <geometry>
           <capsule length="0.2" radius="0.1"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="10.0"/>
         <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
       </inertial>
     </link>
     <joint name="hip_joint" type="revolute">
       <parent link="base_link"/>
       <child link="torso"/>
       <axis xyz="0 0 1"/>
       <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
     </joint>
   </robot>
   ```

- **Convert URDF to USD**:
   Using Isaac Sim's built-in converter:
   ```bash
   # Launch Isaac Sim with URDF importer
   ./isaac-sim.sh --exec "omni.isaac.urdf_importer" --no-window

   # Or use the Python API
   python -m omni.isaac.kit.tools.urdf_importer --config="path/to/config.json"
   ```

- **Command Line Conversion**:
   ```python
   from omni.isaac.urdf_importer import _urdf_importer
   import omni

   # Initialize URDF importer
   urdf_interface = _urdf_importer.acquire_urdf_interface()

   # Import URDF and convert to USD
   imported_robot = urdf_interface.parse_from_file(
       urdf_filename="path/to/humanoid.urdf",
       import_config=_urdf_importer.UrdfImportConfig()
   )

   # Save as USD
   urdf_interface.save_as_usd(
       imported_robot,
       usd_filename="output/humanoid_robot.usd"
   )
   ```

### Using External Tools

- **Using Omniverse Create**:
   - Open Omniverse Create
   - Import URDF using the URDF Importer extension
   - Export as USD

- **Using ROS Tools**:
   ```bash
   # If you have ROS with xacro support
   rosrun xacro xacro --inorder robot.urdf.xacro > robot.urdf

   # Then convert using Isaac Sim tools
   ```

## Method 3: Using the Isaac Sim URDF Importer GUI

- **Launch Isaac Sim** and open the URDF Importer:
   - Go to Window > Extensions
   - Search for "URDF Importer" and enable it
   - Click on "URDF Importer" in the toolbar

- **Configure Import Settings**:
   - Set URDF file path
   - Configure joint properties
   - Set collision and visual properties
   - Choose output USD path

- **Import and Validate**:
   - Click "Import" button
   - Check the imported model in the viewport
   - Verify joint ranges and limits

## Configuring Humanoid Models for Simulation

### Physics Properties
```python
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.core.utils.stage import get_current_stage
from pxr import UsdPhysics, PhysicsSchemaTools

# Set physics properties for humanoid links
stage = get_current_stage()

# Apply physics properties to each link
for link_name in ["torso", "left_upper_leg", "right_upper_leg", "left_lower_leg", "right_lower_leg"]:
    link_path = f"/World/HumanoidRobot/{link_name}"
    link_prim = stage.GetPrimAtPath(link_path)

    # Add rigid body API
    UsdPhysics.RigidBodyAPI.Apply(link_prim, "physics")

    # Set mass properties
    mass_api = UsdPhysics.MassAPI.Apply(link_prim)
    mass_api.CreateMassAttr().Set(5.0)  # kg
```

### Joint Configuration
```python
from omni.isaac.core.utils.stage import get_current_stage
from pxr import UsdSkel, UsdGeom

stage = get_current_stage()

# Configure humanoid skeleton joints
skeleton = UsdSkel.Skeleton.Define(stage, "/World/HumanoidRobot/Skeleton")
skeleton.CreateJointsAttr().Set([
    "pelvis", "torso", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle"
])

# Set joint transforms
bind_transforms = [Gf.Matrix4d(1)] * 8  # Identity matrices
skeleton.CreateBindTransformsAttr(bind_transforms)

# Set rest transforms
rest_transforms = [Gf.Matrix4d(1)] * 8
skeleton.CreateRestTransformsAttr(rest_transforms)
```

## Common Humanoid Model Configurations

### Basic Humanoid Structure
```
Humanoid Model
├── base_link (pelvis)
│   ├── torso
│   ├── left_hip
│   │   ├── left_upper_leg
│   │   └── left_lower_leg
│   └── right_hip
│       ├── right_upper_leg
│       └── right_lower_leg
├── left_shoulder
│   ├── left_upper_arm
│   └── left_lower_arm
└── right_shoulder
    ├── right_upper_arm
    └── right_lower_arm
```

### Typical Joint Limits
- Hip joints: ±90° (abduction/adduction), ±45° (flexion/extension)
- Knee joints: 0° to -120° (flexion only)
- Ankle joints: ±20° (pitch), ±15° (roll)
- Torso: ±30° in all directions

## Loading Multiple Humanoid Models

### Spawning Multiple Instances
```python
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

# Load multiple humanoid models at different positions
positions = [
    [0, 0, 1.0],
    [2, 0, 1.0],
    [0, 2, 1.0],
    [-2, 0, 1.0]
]

for i, pos in enumerate(positions):
    add_reference_to_stage(
        usd_path="/path/to/humanoid_model.usd",
        prim_path=f"/World/HumanoidRobot_{i}",
        position=pos
    )
```

## Troubleshooting Model Loading Issues

### Common Problems and Solutions

#### 1. Model Not Loading
**Symptoms**: Model appears invisible or missing
**Solutions**:
- Verify USD file path is correct
- Check file permissions
- Validate USD file structure with `usdview`
- Ensure Isaac Sim has read access to the file

#### 2. Physics Issues
**Symptoms**: Robot falls through floor or behaves erratically
**Solutions**:
- Check mass properties for all links
- Verify collision geometry is properly defined
- Ensure all joints have proper limits
- Validate center of mass placement

#### 3. Joint Configuration Problems
**Symptoms**: Joints don't move or move unexpectedly
**Solutions**:
- Verify joint types (revolute, prismatic, etc.)
- Check joint axis alignment
- Validate joint limits and ranges
- Ensure proper parent-child relationships

#### 4. Performance Issues
**Symptoms**: Low frame rates or simulation lag
**Solutions**:
- Reduce polygon count in visual meshes
- Simplify collision geometry
- Limit number of simultaneous models
- Optimize texture sizes

## Best Practices

### Model Optimization
- **Keep polygon counts reasonable**: `&lt;10K` polygons per robot for real-time performance
- **Use appropriate collision geometry**: Simplified shapes rather than visual mesh
- **Validate mass properties**: Total robot mass should be realistic (50-100kg for humanoid)
- **Set proper joint limits**: Avoid unlimited joints that can cause simulation instability

### File Organization
```
robot_models/
├── humanoid_base.usd
├── humanoid_with_sensors.usd
├── configs/
│   ├── walking_config.usd
│   └── standing_config.usd
└── assets/
    ├── meshes/
    └── materials/
```

### Validation Steps
- **Visual inspection**: Load in usdview to check geometry
- **Kinematic validation**: Verify joint ranges and connectivity
- **Physical validation**: Test with simple physics simulation
- **Sensor validation**: Ensure sensors are properly positioned and oriented

## Next Steps

After successfully loading humanoid models:
- Configure [Physics and Lighting](./physics-lighting-config.md)
- Set up [RGB Camera Sensors](./rgb-sensor-setup.md)
- Configure [Depth Sensors](./depth-sensor-config.md)
- Generate [Synthetic Datasets](./synthetic-data-generation.md)

## Additional Resources

- [Isaac Sim URDF Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_urdf_importer.html)
- [USD Robot Format Specification](https://github.com/PixarAnimationStudios/USD/tree/release/extras/usd_robots)
- [Humanoid Robot Kinematics Guide](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_bridges/isaac_ros_cumotion/index.html)