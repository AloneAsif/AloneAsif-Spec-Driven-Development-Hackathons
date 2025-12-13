# Omniverse and USD Fundamentals for Robotics

## Introduction to Omniverse

NVIDIA Omniverse is a simulation and collaboration platform that provides a shared space for 3D design and simulation. For robotics applications, Omniverse serves as the foundation for Isaac Sim, enabling photorealistic simulation with physically accurate rendering and physics.

### Core Concepts

#### USD (Universal Scene Description)
USD is Pixar's scene description format that enables complex scene composition and asset interchange. In robotics simulation, USD files define:
- Robot geometry and kinematics
- Environment layouts
- Material properties
- Lighting conditions
- Animation and motion data

#### Key USD File Extensions
- **.usd/.usda**: Plain USD files (ASCII text format)
- **.usdc**: Compressed USD files (binary format)
- **.usdz**: Zipped USD files (for sharing)

### Omniverse Architecture for Robotics

```
Omniverse Platform
├── Isaac Sim (Robotics Simulation)
│   ├── Physics Engine (PhysX)
│   ├── Renderer (NVIDIA RTX)
│   ├── USD Scene Management
│   └── ROS 2 Integration
├── Asset Library
├── Collaboration Services
└── Extension Framework
```

## USD Fundamentals for Robotics

### Scene Hierarchy
In USD, everything is organized in a hierarchical structure called a "stage". The stage contains "primitives" (or "prims") that represent objects, lights, cameras, and other scene elements.

```python
# USD Stage Structure Example
/stage
├── /World
│   ├── /Robot
│   │   ├── /Base
│   │   ├── /Link1
│   │   └── /Link2
│   ├── /Environment
│   │   ├── /Floor
│   │   ├── /Walls
│   │   └── /Obstacles
│   └── /Sensors
│       ├── /Camera
│       └── /DepthSensor
```

### Robot Representation in USD

#### URDF to USD Conversion
Robot models typically start as URDF (Unified Robot Description Format) and are converted to USD for Isaac Sim:

```bash
# Example URDF to USD conversion
python -m omni.isaac.kit.tools.usd_importer input.urdf output.usd
```

#### USD Robot Structure
A typical robot in USD includes:
- **Rigid bodies**: Links with mass and collision properties
- **Joints**: Constraints defining degrees of freedom
- **Materials**: Visual appearance and physical properties
- **Sensors**: Camera, depth, IMU, etc.
- **Articulations**: Groups of joints for control

### USD Prim Types Relevant to Robotics

#### Geometry Primitives
- `Xform`: Transformation container (position, rotation, scale)
- `Mesh`: Surface geometry definition
- `Capsule`, `Sphere`, `Cylinder`: Basic shapes for collision
- `Cone`, `Cube`: Additional primitive shapes

#### Physics Primitives
- `PhysicsScene`: Physics simulation parameters
- `PhysicsMaterial`: Material properties (friction, restitution)
- `RigidBody`: Rigid body simulation
- `Joint`: Joint constraints (revolute, prismatic, fixed)

#### Sensor Primitives
- `Camera`: RGB camera sensor
- `DepthCamera`: Depth sensor
- `Imu`: Inertial measurement unit
- `Lidar`: LIDAR sensor

## Creating Robot Models in USD

### Basic Robot USD Structure
```usda
#usda 1.0

def Xform "Robot" (
    prepend apiSchemas = ["ArticulationRootAPI"]
)
{
    def Xform "base_link"
    {
        def Sphere "visual"
        {
            uniform token[] primvars:displayColor = [(0.8, 0.8, 0.8)]
        }
        def Sphere "collision"
        {
            # Collision geometry
        }
    }

    def Xform "link1"
    {
        # Additional links...
    }
}
```

### Material Definition
```usda
def Material "RobotMaterial"
{
    def Shader "diffuseShader" (interfaceName = "diffuse")
    {
        uniform token info_id = "OmniPBR"
        color3f inputs:diffuse_tint = (0.8, 0.8, 0.8)
        float inputs:metallic = 0.0
        float inputs:roughness = 0.5
    }

    token outputs:surface.connect = </RobotMaterial/diffuseShader.outputs:surface>
}
```

## Isaac Sim Specific USD Extensions

### Isaac Sim Prims
- `IsaacSensor`: Base sensor definition
- `IsaacArticulation`: Robot articulation controller
- `IsaacRobot`: Robot-specific properties
- `IsaacEnvironment`: Environment-specific properties

### Physics Configuration in USD
```usda
def PhysicsScene "PhysicsScene"
{
    float physics:defaultRestOffset = 0.001
    float physics:defaultContactOffset = 0.002
    float3 physics:gravity = (0, -9.81, 0)
}
```

## Asset Management in Omniverse

### Asset Structure
```
Assets/
├── Robots/
│   ├── Humanoid/
│   │   ├── Atlas.usd
│   │   └── Nadia.usd
│   └── Wheeled/
│       ├── TurtleBot3.usd
│       └── Jackal.usd
├── Environments/
│   ├── Indoor/
│   │   ├── Warehouse.usd
│   │   └── Office.usd
│   └── Outdoor/
│       ├── ParkingLot.usd
│       └── Garden.usd
└── Objects/
    ├── Furniture/
    └── Tools/
```

### Asset Referencing
USD supports referencing and instancing of assets:

```usda
# Referencing an external robot asset
def Xform "MyRobot" (
    prepend references = @./robots/humanoid/atlas.usd@</Atlas>
)
{
    # Local overrides can be added here
    float3 xformOp:translate = (0, 0, 0.5)
}
```

## Working with USD in Isaac Sim

### Loading USD Files
```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage

# Add USD reference to current stage
add_reference_to_stage(usd_path="/path/to/robot.usd", prim_path="/World/Robot")

# Create a new world and load robot
world = World(stage_units_in_meters=1.0)
robot = world.scene.add(
    Robot(
        prim_path="/World/Robot",
        usd_path="/path/to/robot.usd"
    )
)
```

### USD Tools and Utilities
- **usdview**: USD file viewer and inspector
- **usdcat**: USD file concatenation and conversion
- **usdzip**: USD file packaging tool
- **Isaac Sim USD Importer**: URDF to USD conversion

## Best Practices for Robotics USD

### Performance Optimization
1. **LOD (Level of Detail)**: Create simplified versions for distant objects
2. **Instancing**: Use instancing for repeated objects
3. **Texture atlasing**: Combine multiple textures into single files
4. **Polygon count**: Keep robot meshes under reasonable limits

### Collision Geometry
- Use simplified meshes for collision detection
- Ensure collision geometry is watertight
- Consider using primitive shapes where possible
- Validate joint limits in collision geometry

### Material Optimization
- Use physically based materials (OmniPBR)
- Share materials across similar objects
- Keep texture resolution appropriate for distance
- Use USD's material binding system effectively

## USD for Synthetic Data Generation

### USD for Dataset Creation
USD files are ideal for creating synthetic datasets because they provide:
- Perfect ground truth (geometry, pose, semantics)
- Flexible scene composition
- Consistent format across different scenarios
- Easy modification of lighting and materials

### USD Variants
Use USD variants for creating multiple robot configurations:

```usda
def Xform "Robot" (
    variants = {
        string robotType = "humanoid"
    }
)
{
    variantSet "robotType" = {
        "humanoid" {
            # Humanoid robot configuration
        }
        "wheeled" {
            # Wheeled robot configuration
        }
    }
}
```

## Troubleshooting USD Issues

### Common USD Problems
- **Invalid topology**: Check mesh geometry for non-manifold edges
- **Reference issues**: Verify asset paths are correct
- **Performance problems**: Check polygon counts and texture sizes
- **Physics errors**: Validate collision geometry and joint limits

### USD Validation
```bash
# Validate USD file structure
usdval robot.usd

# Check for errors in Isaac Sim
omni.kit.tools.usd_stage_inspector
```

## Next Steps

After understanding USD fundamentals:
1. Learn about [Humanoid Model Loading](./humanoid-model-loading.md)
2. Configure [Physics and Lighting](./physics-lighting-config.md)
3. Set up [Sensors for Data Generation](./synthetic-data-generation.md)

## Additional Resources

- [USD Documentation](https://graphics.pixar.com/usd/release/docs/index.html)
- [Isaac Sim USD Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_working_with_usd.html)
- [USD for Robotics](https://github.com/PixarAnimationStudios/USD/tree/release/extras/usd_robots)