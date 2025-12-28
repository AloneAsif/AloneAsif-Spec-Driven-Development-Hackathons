# Physics and Lighting Configuration in Isaac Sim

## Physics Configuration

### Physics Scene Setup

Isaac Sim uses NVIDIA PhysX as its physics engine, providing realistic physics simulation for robotics applications. Proper physics configuration is crucial for accurate simulation and stable humanoid locomotion.

#### Creating a Physics Scene
```python
import omni
from omni.isaac.core import World
from pxr import PhysxSchema, UsdPhysics, Gf
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim

# Create a physics scene with specific parameters
physics_scene = UsdPhysics.Scene.Define(world.stage, "/World/physicsScene")

# Set gravity (standard Earth gravity)
physics_scene.CreateGravityAttr().Set(Gf.Vec3f(0.0, -9.81, 0.0))

# Set solver parameters
physics_scene.CreateTimestepAttr().Set(1.0/60.0)  # 60 FPS
physics_scene.CreateMaxSubstepsAttr().Set(1)      # Number of substeps per frame
```

#### Physics Scene Parameters
- **Gravity**: Typically (0, -9.81, 0) for Earth gravity
- **Timestep**: Usually 1/60 to 1/240 seconds for stability
- **Max Substeps**: 1-8 depending on simulation complexity
- **Solver Type**: TGS (Truncated Generalized Solver) recommended

### Rigid Body Configuration

#### Setting Up Humanoid Links as Rigid Bodies
```python
from pxr import UsdPhysics, UsdGeom, Gf
from omni.isaac.core.utils.prims import get_prim_at_path

# Configure a humanoid link as a rigid body
def configure_rigid_body(prim_path, mass, linear_damping=0.05, angular_damping=0.1):
    link_prim = get_prim_at_path(prim_path)

    # Apply rigid body API
    rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(link_prim)
    rigid_body_api.CreateRigidBodyEnabledAttr().Set(True)

    # Set mass properties
    mass_api = UsdPhysics.MassAPI.Apply(link_prim)
    mass_api.CreateMassAttr().Set(mass)

    # Set damping (helps with stability)
    rigid_body_api.CreateLinearDampingAttr().Set(linear_damping)
    rigid_body_api.CreateAngularDampingAttr().Set(angular_damping)

# Configure humanoid body parts with appropriate masses
configure_rigid_body("/World/HumanoidRobot/torso", 10.0)
configure_rigid_body("/World/HumanoidRobot/left_upper_leg", 5.0)
configure_rigid_body("/World/HumanoidRobot/right_upper_leg", 5.0)
configure_rigid_body("/World/HumanoidRobot/left_lower_leg", 3.0)
configure_rigid_body("/World/HumanoidRobot/right_lower_leg", 3.0)
```

### Collision Configuration

#### Collision Geometry Best Practices
```python
from pxr import UsdPhysics, UsdGeom

def setup_collision_geometry(prim_path, geometry_type="capsule", size=(0.1, 0.3)):
    """
    Set up collision geometry for humanoid links
    geometry_type: "capsule", "sphere", "box", "mesh"
    size: depends on geometry type
    """
    link_prim = get_prim_at_path(prim_path)

    # Create collision approximation
    if geometry_type == "capsule":
        collision_geom = UsdGeom.Capsule.Define(world.stage, f"{prim_path}/collision")
        collision_geom.CreateRadiusAttr().Set(size[0])
        collision_geom.CreateHeightAttr().Set(size[1])
    elif geometry_type == "sphere":
        collision_geom = UsdGeom.Sphere.Define(world.stage, f"{prim_path}/collision")
        collision_geom.CreateRadiusAttr().Set(size[0])
    elif geometry_type == "box":
        collision_geom = UsdGeom.Cube.Define(world.stage, f"{prim_path}/collision")
        collision_geom.GetSizeAttr().Set(size[0])

    # Apply collision API
    collision_api = UsdPhysics.CollisionAPI.Apply(collision_geom.GetPrim())
    collision_api.CreateCollisionEnabledAttr().Set(True)

# Set up collision for humanoid parts
setup_collision_geometry("/World/HumanoidRobot/torso", "capsule", (0.15, 0.6))
setup_collision_geometry("/World/HumanoidRobot/upper_leg", "capsule", (0.08, 0.4))
setup_collision_geometry("/World/HumanoidRobot/lower_leg", "capsule", (0.07, 0.4))
```

#### Physics Materials
```python
from pxr import UsdShade, PhysxSchema

def create_physics_material(material_path, static_friction=0.5, dynamic_friction=0.5, restitution=0.1):
    """Create a physics material with specific properties"""
    material = PhysxSchema.PhysxMaterial.Define(world.stage, material_path)
    material.CreateStaticFrictionAttr().Set(static_friction)
    material.CreateDynamicFrictionAttr().Set(dynamic_friction)
    material.CreateRestitutionAttr().Set(restitution)
    return material

# Create different materials for different surfaces
floor_material = create_physics_material("/World/Looks/floor_material",
                                       static_friction=0.8, dynamic_friction=0.7, restitution=0.1)
humanoid_material = create_physics_material("/World/Looks/humanoid_material",
                                          static_friction=0.6, dynamic_friction=0.5, restitution=0.2)
```

### Joint Configuration for Humanoid Stability

#### Configuring Humanoid Joints
```python
from pxr import PhysxSchema, UsdPhysics, Gf

def configure_joint_limits(prim_path, joint_type, limits_low, limits_high):
    """Configure joint limits for humanoid stability"""
    joint_prim = get_prim_at_path(prim_path)

    if joint_type == "revolute":
        joint = UsdPhysics.RevoluteJoint.Get(world.stage, prim_path)
        joint.CreateLowerLimitAttr().Set(limits_low)
        joint.CreateUpperLimitAttr().Set(limits_high)
    elif joint_type == "spherical":
        # For more complex joints like hip joints
        joint = UsdPhysics.SphericalJoint.Get(world.stage, prim_path)

# Example joint configurations for humanoid
configure_joint_limits("/World/HumanoidRobot/hip_joint", "revolute", -0.78, 0.78)  # ±45°
configure_joint_limits("/World/HumanoidRobot/knee_joint", "revolute", 0.0, 2.09)    # 0 to 120°
configure_joint_limits("/World/HumanoidRobot/ankle_joint", "revolute", -0.35, 0.35) # ±20°
```

## Lighting Configuration

### Basic Lighting Setup

Isaac Sim provides realistic lighting for photorealistic rendering, essential for synthetic data generation.

#### Adding a Dome Light
```python
from omni.isaac.core.utils.prims import create_prim

# Create dome light for environment lighting
create_prim(
    prim_path="/World/DomeLight",
    prim_type="DomeLight",
    position=[0, 0, 0],
    attributes={
        "color": (0.9, 0.9, 0.9),      # Warm white
        "intensity": 3000,             # Light intensity
        "enableColorTemperature": False
    }
)
```

#### Adding Directional Lights
```python
# Add directional light to simulate sun
create_prim(
    prim_path="/World/DirectionalLight",
    prim_type="DistantLight",
    position=[0, 0, 10],
    orientation=[0.707, 0, -0.707, 0],  # Pointing downward at 45 degree angle
    attributes={
        "color": (0.95, 0.9, 0.8),      # Slightly warm
        "intensity": 500,
        "angle": 0.5
    }
)
```

#### Adding Spot Lights for Detail
```python
# Add spot lights for focused illumination
create_prim(
    prim_path="/World/SpotLight",
    prim_type="SpotLight",
    position=[2, 2, 3],
    attributes={
        "color": (1.0, 1.0, 1.0),
        "intensity": 1000,
        "innerConeAngle": 20,
        "outerConeAngle": 30,
        "radius": 0.1
    }
)
```

### Advanced Lighting Techniques

#### HDR Environment Maps
```python
# Use HDR environment map for realistic lighting
from pxr import UsdLux

def setup_hdr_environment(hdr_path):
    """Set up HDR environment for realistic lighting"""
    dome_light = UsdLux.DomeLight.Define(world.stage, "/World/EnvLight")
    dome_light.CreateTextureFileAttr().Set(hdr_path)
    dome_light.CreateColorAttr().Set((1.0, 1.0, 1.0))
    dome_light.CreateIntensityAttr().Set(1.0)

# Example HDR environment
setup_hdr_environment("path/to/hdr/environment.hdr")
```

#### Dynamic Lighting Configuration
```python
def configure_dynamic_lighting(day_time=True):
    """Configure lighting based on time of day"""
    if day_time:
        # Bright daylight configuration
        dome_light = get_prim_at_path("/World/DomeLight")
        dome_light.GetAttribute("inputs:intensity").Set(5000)
        dome_light.GetAttribute("inputs:color").Set((0.95, 0.92, 0.88))
    else:
        # Night time configuration
        dome_light = get_prim_at_path("/World/DomeLight")
        dome_light.GetAttribute("inputs:intensity").Set(500)
        dome_light.GetAttribute("inputs:color").Set((0.4, 0.45, 0.6))

# Example usage
configure_dynamic_lighting(day_time=True)
```

## Performance Optimization

### Physics Optimization Settings
```python
def optimize_physics_performance():
    """Optimize physics settings for better performance"""
    physics_scene = UsdPhysics.Scene.Get(world.stage, "/World/physicsScene")

    # Adjust solver settings for performance
    physics_scene.CreateMaxVelocityAttr().Set(1000.0)      # Max linear velocity
    physics_scene.CreateMaxAngularVelocityAttr().Set(1000.0)  # Max angular velocity
    physics_scene.CreateMaxDepenetrationVelocityAttr().Set(100.0)  # Max depenetration velocity

    # Enable GPU dynamics if available
    physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
    physx_scene_api.CreateEnableGpuDynamicParticlesAttr().Set(True)

# Apply optimizations
optimize_physics_performance()
```

### Rendering Optimization
```python
def configure_rendering_settings():
    """Configure rendering settings for optimal performance"""
    # Set rendering quality settings
    settings = {
        "rtx-defaults:pathtracingSamplesPerFrame": 1,
        "rtx-defaults:useInteractiveMode": True,  # Faster but less accurate
        "renderer:resyncOnSceneChanged": False,
        "renderer:useSceneCache": True
    }

    for setting, value in settings.items():
        omni.kit.app.get_app().get_setting_store().set(setting, value)

# Apply rendering optimizations
configure_rendering_settings()
```

## Environment Setup

### Creating Realistic Environments
```python
def create_realistic_environment():
    """Set up a realistic environment for humanoid simulation"""

    # Create ground plane with appropriate material
    create_prim(
        prim_path="/World/groundPlane",
        prim_type="Plane",
        position=[0, 0, 0],
        scale=[20, 1, 20],  # Large plane for humanoid movement
        attributes={
            "visibility": "inherited"
        }
    )

    # Apply physics properties to ground
    ground_prim = get_prim_at_path("/World/groundPlane")
    collision_api = UsdPhysics.CollisionAPI.Apply(ground_prim)
    collision_api.CreateCollisionEnabledAttr().Set(True)

    # Add physics material to ground
    material_path = "/World/Looks/GroundMaterial"
    ground_material = create_physics_material(
        material_path,
        static_friction=0.8,
        dynamic_friction=0.7,
        restitution=0.1
    )

    # Bind material to ground
    UsdShade.MaterialBindingAPI(ground_prim).Bind(ground_material)

# Create the environment
create_realistic_environment()
```

## Validation and Testing

### Physics Validation
```python
def validate_physics_setup():
    """Validate physics configuration"""
    print("Validating physics setup...")

    # Check if physics scene exists
    physics_scene = UsdPhysics.Scene.Get(world.stage, "/World/physicsScene")
    if not physics_scene:
        print("ERROR: Physics scene not found")
        return False

    # Check gravity setting
    gravity = physics_scene.GetGravityAttr().Get()
    expected_gravity = Gf.Vec3f(0.0, -9.81, 0.0)
    if gravity != expected_gravity:
        print(f"WARNING: Gravity is {gravity}, expected {expected_gravity}")

    # Check humanoid mass properties
    humanoid_links = [
        "/World/HumanoidRobot/torso",
        "/World/HumanoidRobot/left_upper_leg",
        "/World/HumanoidRobot/right_upper_leg"
    ]

    for link_path in humanoid_links:
        link_prim = get_prim_at_path(link_path)
        if link_prim:
            mass_api = UsdPhysics.MassAPI(link_prim)
            mass = mass_api.GetMassAttr().Get()
            if mass and mass > 0:
                print(f"  {link_path}: mass = {mass} kg")
            else:
                print(f"  {link_path}: mass not set or invalid")

    print("Physics validation complete")
    return True

# Validate the setup
validate_physics_setup()
```

### Lighting Validation
```python
def validate_lighting_setup():
    """Validate lighting configuration"""
    print("Validating lighting setup...")

    # Check for dome light
    dome_light = get_prim_at_path("/World/DomeLight")
    if dome_light:
        intensity = dome_light.GetAttribute("inputs:intensity").Get()
        print(f"  Dome light intensity: {intensity}")
    else:
        print("  WARNING: No dome light found")

    # Check for directional light
    dir_light = get_prim_at_path("/World/DirectionalLight")
    if dir_light:
        print("  Directional light found")
    else:
        print("  WARNING: No directional light found")

    print("Lighting validation complete")

# Validate lighting
validate_lighting_setup()
```

## Troubleshooting Common Issues

### Physics Issues
- **Humanoid falls through floor**: Check collision geometry and physics materials
- **Jittery movement**: Reduce timestep or increase damping
- **Unstable joints**: Verify joint limits and constraints
- **Performance problems**: Simplify collision geometry or reduce substeps

### Lighting Issues
- **Dark rendering**: Check if lights are enabled and have sufficient intensity
- **Artifacts**: Adjust rendering settings or light parameters
- **Performance**: Use fewer lights or reduce quality settings

## Best Practices

### Physics Best Practices
1. **Start simple**: Begin with basic physics and gradually add complexity
2. **Validate mass**: Ensure total humanoid mass is realistic (50-100kg)
3. **Use damping**: Apply appropriate damping to prevent oscillations
4. **Test stability**: Run simulation for extended periods to check stability

### Lighting Best Practices
1. **Use multiple light sources**: Combine dome, directional, and spot lights
2. **Match environment**: Adjust lighting to match your target environment
3. **Consider performance**: Balance quality with real-time performance needs
4. **Test different conditions**: Validate under various lighting conditions

## Next Steps

After configuring physics and lighting:
1. Set up [RGB Camera Sensors](./rgb-sensor-setup.md)
2. Configure [Depth Sensors](./depth-sensor-config.md)
3. Set up [Semantic Segmentation Sensors](./segmentation-sensor-setup.md)
4. Generate [Synthetic Datasets](./synthetic-data-generation.md)

## Additional Resources

- [Isaac Sim Physics Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_physics.html)
- [Lighting Guide for Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/programming_guide/advanced_tutorials/tutorial_advanced_lighting.html)
- [Performance Optimization Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/programming_guide/optimization/performance_optimization.html)