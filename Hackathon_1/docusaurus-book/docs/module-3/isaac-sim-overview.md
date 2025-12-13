# Isaac Sim & Synthetic Worlds

## Overview

This chapter introduces NVIDIA Isaac Sim for photorealistic humanoid simulation and synthetic data generation. Students will learn to set up Isaac Sim, load humanoid models, configure sensors, and generate synthetic datasets for perception tasks.

## Learning Objectives

By the end of this chapter, students will be able to:
- Install and configure Isaac Sim on Ubuntu 22.04
- Load and configure humanoid models in USD format
- Set up physics and lighting for realistic simulation
- Configure RGB, depth, and segmentation sensors
- Generate synthetic datasets for robotics perception
- Optimize simulation performance for real-time operation

## Isaac Sim Installation and Setup

### System Requirements

Before installing Isaac Sim, ensure your system meets the following requirements:

- **Operating System**: Ubuntu 22.04 LTS
- **GPU**: NVIDIA RTX GPU (RTX 3080 or higher recommended)
  - Minimum: RTX 2080 with 8GB+ VRAM
  - Recommended: RTX 3080/4090 with 12GB+ VRAM for optimal performance
- **CPU**: Multi-core processor (Intel i7 or AMD Ryzen 7 or equivalent)
- **RAM**: 16GB+ (32GB recommended for Isaac Sim)
- **Storage**: 50GB+ free space for Isaac Sim assets and dependencies
- **Software**: CUDA 11.8+, NVIDIA drivers, ROS 2 Humble

### Installation Process

1. **Verify Hardware and Software Requirements**:
   ```bash
   # Check GPU availability
   nvidia-smi

   # Verify CUDA installation
   nvcc --version

   # Check Ubuntu version
   lsb_release -a
   ```

2. **Download Isaac Sim**:
   - Go to https://developer.nvidia.com/isaac-sim
   - Sign in with your NVIDIA account
   - Download the latest version for Linux
   - Extract the downloaded archive:
   ```bash
   tar -xzf isaac_sim-2023.1.1.tar.gz
   cd isaac_sim-2023.1.1
   ```

3. **Install Isaac Sim**:
   ```bash
   # Run the setup script to install Isaac Sim
   bash setup.sh --add-python-installation
   ```

4. **Launch Isaac Sim**:
   ```bash
   # Basic launch
   ./isaac-sim.sh

   # Launch with specific parameters
   ./isaac-sim.sh --no-window --/renderer/enabled=False  # Headless mode
   ```

## Omniverse and USD Fundamentals

### USD (Universal Scene Description)

USD is Pixar's scene description format that enables complex scene composition and asset interchange. In robotics simulation, USD files define:
- Robot geometry and kinematics
- Environment layouts
- Material properties
- Lighting conditions
- Animation and motion data

### USD File Extensions
- **.usd/.usda**: Plain USD files (ASCII text format)
- **.usdc**: Compressed USD files (binary format)
- **.usdz**: Zipped USD files (for sharing)

### Basic USD Structure for Robotics
```
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

## Humanoid Model Loading

### Direct USD Model Loading

If your humanoid model is already in USD format:

1. **Verify Model Structure**:
   ```bash
   # Check USD file structure
   usdview humanoid_model.usd
   ```

2. **Load in Isaac Sim** using Python API:
   ```python
   import omni
   from omni.isaac.core import World
   from omni.isaac.core.utils.stage import add_reference_to_stage

   # Add reference to humanoid model
   add_reference_to_stage(
       usd_path="/path/to/humanoid_model.usd",
       prim_path="/World/HumanoidRobot"
   )
   ```

### URDF to USD Conversion

1. **Using Isaac Sim's URDF Importer**:
   ```bash
   # Launch Isaac Sim with URDF importer
   ./isaac-sim.sh --exec "omni.isaac.urdf_importer" --no-window
   ```

2. **Using Python API**:
   ```python
   from omni.isaac.urdf_importer import _urdf_importer
   import omni

   # Initialize URDF importer
   urdf_interface = _urdf_importer.acquire_urdf_interface()

   # Import and convert
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

## Physics and Lighting Configuration

### Physics Scene Setup

```python
from pxr import PhysxSchema, UsdPhysics, Gf
from omni.isaac.core import World

# Create a physics scene with specific parameters
physics_scene = UsdPhysics.Scene.Define(world.stage, "/World/physicsScene")

# Set gravity (standard Earth gravity)
physics_scene.CreateGravityAttr().Set(Gf.Vec3f(0.0, -9.81, 0.0))

# Set solver parameters
physics_scene.CreateTimestepAttr().Set(1.0/60.0)  # 60 FPS
physics_scene.CreateMaxSubstepsAttr().Set(1)      # Number of substeps per frame
```

### Rigid Body Configuration

```python
# Configure a humanoid link as a rigid body
def configure_rigid_body(prim_path, mass, linear_damping=0.05, angular_damping=0.1):
    from omni.isaac.core.utils.prims import get_prim_at_path
    from pxr import UsdPhysics

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
```

### Lighting Setup

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

## Sensor Configuration

### RGB Camera Setup

```python
def add_rgb_camera(prim_path, position, orientation, config=None):
    """
    Add an RGB camera to the scene
    """
    if config is None:
        config = {
            "resolution": {"width": 1920, "height": 1080},
            "focal_length": 24.0,
            "focus_distance": 10.0,
            "f_stop": 0.0,
            "horizontal_aperture": 36.0,
            "vertical_aperture": 20.25
        }

    from omni.isaac.core.utils.prims import create_prim

    camera_prim = create_prim(
        prim_path=prim_path,
        prim_type="Camera",
        position=position,
        orientation=orientation,
        attributes={
            "focalLength": config["focal_length"],
            "focusDistance": config["focus_distance"],
            "fStop": config["f_stop"],
            "horizontalAperture": config["horizontal_aperture"],
            "verticalAperture": config["vertical_aperture"],
            "clippingRange": (0.1, 1000000.0)
        }
    )

    return camera_prim

# Example: Add camera to humanoid head
rgb_camera = add_rgb_camera(
    prim_path="/World/HumanoidRobot/head/rgb_camera",
    position=[0, 0.1, 0],  # Slightly above the head
    orientation=[0.707, 0, 0.707, 0],  # Looking forward (rotated 90° around Y)
    config={
        "resolution": {"width": 1920, "height": 1080},
        "focal_length": 24.0
    }
)
```

### Depth Sensor Configuration

```python
def add_depth_sensor(prim_path, position, orientation, config=None):
    """
    Add a depth sensor to the scene
    """
    if config is None:
        config = {
            "resolution": {"width": 640, "height": 480},
            "focal_length": 24.0,
            "min_depth": 0.1,    # Minimum measurable distance in meters
            "max_depth": 10.0,   # Maximum measurable distance in meters
            "format": "R16f"     # 16-bit float format
        }

    from omni.isaac.core.utils.prims import create_prim

    # Create the depth sensor prim
    depth_sensor = create_prim(
        prim_path=prim_path,
        prim_type="Camera",  # Depth sensors are implemented as special cameras
        position=position,
        orientation=orientation,
        attributes={
            "focalLength": config["focal_length"],
            "horizontalAperture": 36.0,
            "verticalAperture": 20.25,
            "clippingRange": (config["min_depth"], config["max_depth"])
        }
    )

    # Add depth-specific attributes
    from omni.isaac.core.utils.prims import get_prim_at_path
    depth_prim = get_prim_at_path(prim_path)
    depth_prim.GetAttribute("depth:enabled").Set(True)
    depth_prim.GetAttribute("depth:min_range").Set(config["min_depth"])
    depth_prim.GetAttribute("depth:max_range").Set(config["max_depth"])
    depth_prim.GetAttribute("depth:format").Set(config["format"])

    return depth_sensor
```

### Semantic Segmentation Sensor Setup

```python
def add_segmentation_sensor(prim_path, position, orientation, config=None):
    """
    Add a semantic segmentation sensor to the scene
    """
    if config is None:
        config = {
            "resolution": {"width": 640, "height": 480},
            "focal_length": 24.0,
            "modality": "segmentation",  # segmentation, instance, panoptic
            "format": "rgb"  # rgb (color-coded) or id (integer IDs)
        }

    from omni.isaac.core.utils.prims import create_prim

    # Create the segmentation sensor as a special camera
    seg_sensor = create_prim(
        prim_path=prim_path,
        prim_type="Camera",
        position=position,
        orientation=orientation,
        attributes={
            "focalLength": config["focal_length"],
            "horizontalAperture": 36.0,
            "verticalAperture": 20.25,
            "clippingRange": (0.01, 1000000.0)
        }
    )

    # Add segmentation-specific attributes
    from omni.isaac.core.utils.prims import get_prim_at_path
    seg_prim = get_prim_at_path(prim_path)
    seg_prim.GetAttribute("segmentation:enabled").Set(True)
    seg_prim.GetAttribute("segmentation:modality").Set(config["modality"])
    seg_prim.GetAttribute("segmentation:format").Set(config["format"])
    seg_prim.GetAttribute("sensor:modality").Set("segmentation")

    return seg_sensor
```

### Assigning Semantic Labels to Objects

```python
def assign_semantic_label(prim_path, label, semantic_id, role="object"):
    """
    Assign semantic label to a prim
    """
    from omni.isaac.core.utils.prims import get_prim_at_path

    prim = get_prim_at_path(prim_path)

    # Set semantic attributes
    prim.GetAttribute("primvars:semantic:label").Set(label)
    prim.GetAttribute("primvars:semantic:id").Set(semantic_id)
    prim.GetAttribute("primvars:semantic:role").Set(role)

# Assign semantic labels to humanoid robot parts
assign_semantic_label("/World/HumanoidRobot/torso", "torso", 1, "robot")
assign_semantic_label("/World/HumanoidRobot/head", "head", 2, "robot")
assign_semantic_label("/World/HumanoidRobot/left_arm", "arm", 3, "robot")
assign_semantic_label("/World/HumanoidRobot/right_arm", "arm", 3, "robot")
assign_semantic_label("/World/HumanoidRobot/left_leg", "leg", 4, "robot")
assign_semantic_label("/World/HumanoidRobot/right_leg", "leg", 4, "robot")

# Assign labels to environment
assign_semantic_label("/World/groundPlane", "floor", 10, "environment")
assign_semantic_label("/World/obstacle_box", "obstacle", 20, "obstacle")
```

## Synthetic Dataset Generation

### Setting Up the Data Generation Pipeline

```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim, get_prim_at_path
from omni.isaac.synthetic_utils import DataWriter
import numpy as np
import os
from PIL import Image

class SyntheticDatasetGenerator:
    def __init__(self, output_dir, dataset_name="synthetic_dataset"):
        """
        Initialize synthetic dataset generator
        """
        self.output_dir = output_dir
        self.dataset_name = dataset_name
        self.data_writer = None
        self.scene_objects = []
        self.camera_configs = []

        # Create output directory structure
        self._create_dataset_structure()

    def _create_dataset_structure(self):
        """Create the standard dataset directory structure"""
        dirs_to_create = [
            f"{self.output_dir}/{self.dataset_name}/images",
            f"{self.output_dir}/{self.dataset_name}/depth",
            f"{self.output_dir}/{self.dataset_name}/segmentation",
            f"{self.output_dir}/{self.dataset_name}/metadata",
            f"{self.output_dir}/{self.dataset_name}/labels"
        ]

        for dir_path in dirs_to_create:
            os.makedirs(dir_path, exist_ok=True)

        print(f"Dataset structure created in: {self.output_dir}/{self.dataset_name}")

    def setup_scene(self, scene_config):
        """
        Set up the scene for data generation
        """
        # Add ground plane
        create_prim(
            prim_path="/World/groundPlane",
            prim_type="Plane",
            position=[0, 0, 0],
            scale=[20, 1, 20],
            attributes={"visibility": "inherited"}
        )

        # Add physics to ground
        from pxr import UsdPhysics
        ground_prim = get_prim_at_path("/World/groundPlane")
        collision_api = UsdPhysics.CollisionAPI.Apply(ground_prim)
        collision_api.CreateCollisionEnabledAttr().Set(True)

        # Add environment objects based on config
        for obj_config in scene_config.get("objects", []):
            self._add_scene_object(obj_config)

        # Set up lighting
        self._setup_lighting(scene_config.get("lighting", {}))

        print("Scene setup complete")

    def capture_frame(self, frame_id):
        """
        Capture a single frame of multi-modal data
        """
        import time
        from omni.synthetic_utils.sensors import CameraSensor

        # Ensure simulation is stepped
        world = World()
        world.step(render=True)

        # Capture RGB image
        rgb_sensor = CameraSensor(self.rgb_camera)
        rgb_data = rgb_sensor.get_rgb_data()

        # Capture depth image
        depth_sensor = CameraSensor(self.depth_camera)
        depth_data = depth_sensor.get_depth_data()

        # Capture segmentation image
        seg_sensor = CameraSensor(self.seg_camera)
        seg_data = seg_sensor.get_segmentation_data()

        # Get camera intrinsic parameters
        intrinsic_matrix = self._get_camera_intrinsics()

        # Create metadata
        metadata = {
            "frame_id": frame_id,
            "timestamp": time.time(),
            "camera_intrinsics": intrinsic_matrix.tolist(),
            "camera_position": self._get_camera_position(),
            "scene_objects": self.scene_objects.copy(),
            "capture_settings": {
                "rgb_resolution": (rgb_data.shape[1], rgb_data.shape[0]),
                "depth_resolution": (depth_data.shape[1], depth_data.shape[0]),
                "seg_resolution": (seg_data.shape[1], seg_data.shape[0])
            }
        }

        # Package the data
        frame_data = {
            "rgb": rgb_data,
            "depth": depth_data,
            "segmentation": seg_data,
            "metadata": metadata
        }

        return frame_data

    def save_frame_data(self, frame_data, frame_id):
        """
        Save captured frame data to the appropriate directories
        """
        import cv2
        from PIL import Image
        import json

        # Save RGB image
        rgb_path = f"{self.output_dir}/{self.dataset_name}/images/frame_{frame_id:06d}.png"
        rgb_image = Image.fromarray(frame_data["rgb"])
        rgb_image.save(rgb_path)

        # Save depth image
        depth_path = f"{self.output_dir}/{self.dataset_name}/depth/frame_{frame_id:06d}.tiff"
        depth_16bit = (frame_data["depth"] * 1000).astype(np.uint16)  # Scale to avoid precision loss
        depth_image = Image.fromarray(depth_16bit)
        depth_image.save(depth_path)

        # Save segmentation image
        seg_path = f"{self.output_dir}/{self.dataset_name}/segmentation/frame_{frame_id:06d}.png"
        seg_16bit = frame_data["segmentation"].astype(np.uint16)
        seg_image = Image.fromarray(seg_16bit)
        seg_image.save(seg_path)

        # Save metadata
        meta_path = f"{self.output_dir}/{self.dataset_name}/metadata/frame_{frame_id:06d}.json"
        with open(meta_path, 'w') as f:
            json.dump(frame_data["metadata"], f, indent=2)

        print(f"Saved frame {frame_id:06d}")
```

## Performance Optimization

### Rendering Optimization

```python
def configure_rendering_quality(quality_level="balanced"):
    """
    Configure rendering quality settings based on performance requirements
    """
    import omni

    settings_map = {
        "high": {
            "rtx-defaults:denoiserType": "Optix",  # Highest quality denoising
            "rtx-defaults:pathtracingSamplesPerFrame": 16,
            "rtx-defaults:useInteractiveMode": False,  # Full ray tracing
            "renderer:resyncOnSceneChanged": False,
            "renderer:useSceneCache": True
        },
        "balanced": {
            "rtx-defaults:denoiserType": "Optix",
            "rtx-defaults:pathtracingSamplesPerFrame": 4,
            "rtx-defaults:useInteractiveMode": True,
            "renderer:resyncOnSceneChanged": False,
            "renderer:useSceneCache": True
        },
        "performance": {
            "rtx-defaults:denoiserType": "None",  # No denoising for speed
            "rtx-defaults:pathtracingSamplesPerFrame": 1,
            "rtx-defaults:useInteractiveMode": True,  # Interactive mode
            "renderer:resyncOnSceneChanged": False,
            "renderer:useSceneCache": True
        }
    }

    if quality_level in settings_map:
        settings = settings_map[quality_level]
        app = omni.kit.app.get_app()
        setting_store = app.get_setting_store()

        for setting, value in settings.items():
            setting_store.set(setting, value)

        print(f"Rendering quality set to: {quality_level}")
    else:
        print(f"Invalid quality level: {quality_level}. Use 'high', 'balanced', or 'performance'")
```

### Physics Optimization

```python
def configure_physics_solver():
    """
    Configure physics solver for optimal performance
    """
    from pxr import UsdPhysics, PhysxSchema
    from omni.isaac.core import World

    world = World()
    stage = world.stage

    # Get physics scene
    physics_scene = UsdPhysics.Scene.Get(stage, "/World/physicsScene")
    if physics_scene:
        # Configure solver parameters
        physics_scene.CreateMaxSubstepsAttr().Set(1)  # Reduce substeps for performance
        physics_scene.CreateMaxVelocityAttr().Set(1000.0)
        physics_scene.CreateMaxAngularVelocityAttr().Set(1000.0)

        # Configure PhysX-specific settings
        physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
        physx_scene_api.CreateSolverTypeAttr().Set("TGS")  # TGS solver often performs better
        physx_scene_api.CreateBroadphaseTypeAttr().Set("MBP")  # Multi-box pruning

        print("Physics solver configured for performance")
    else:
        print("Physics scene not found - ensure physics is enabled")
```

### Sensor Optimization

```python
def optimize_sensor_performance(sensor_paths, optimization_config=None):
    """
    Optimize sensor settings for better performance
    """
    if optimization_config is None:
        optimization_config = {
            "resolution_scale": 0.5,      # Reduce resolution by 50%
            "max_fps": 30,               # Limit frame rate
            "enable_compression": True,  # Enable data compression
            "temporal_subsampling": 1    # Process every frame (no subsampling)
        }

    for sensor_path in sensor_paths:
        sensor_prim = get_prim_at_path(sensor_path)

        if sensor_prim:
            # Get current resolution and scale it
            current_width = sensor_prim.GetAttribute("customData:resolution:width").Get()
            current_height = sensor_prim.GetAttribute("customData:resolution:height").Get()

            if current_width and current_height:
                new_width = int(current_width * optimization_config["resolution_scale"])
                new_height = int(current_height * optimization_config["resolution_scale"])

                sensor_prim.GetAttribute("customData:resolution:width").Set(new_width)
                sensor_prim.GetAttribute("customData:resolution:height").Set(new_height)

            # Set performance attributes
            sensor_prim.GetAttribute("performance:max_fps").Set(optimization_config["max_fps"])
            sensor_prim.GetAttribute("performance:enable_compression").Set(optimization_config["enable_compression"])
            sensor_prim.GetAttribute("performance:temporal_subsampling").Set(optimization_config["temporal_subsampling"])

            print(f"Optimized sensor: {sensor_path} -> {new_width}x{new_height}, {optimization_config['max_fps']} FPS")
```

## Testing and Validation

### Basic Isaac Sim Test

To verify your Isaac Sim setup is working correctly:

1. **Launch Isaac Sim**:
   ```bash
   ./isaac-sim.sh
   ```

2. **Create a simple test scene**:
   - Create a new stage (Ctrl+N)
   - Add a simple primitive (Cube, Sphere, etc.)
   - Verify rendering works properly
   - Test physics by enabling physics and dropping objects

3. **Test with Humanoid Model**:
   - Load your humanoid model
   - Verify it appears correctly in the scene
   - Check that physics properties are applied
   - Test basic movement if applicable

### Synthetic Data Generation Test

To validate the synthetic data generation pipeline:

1. **Run a small dataset generation**:
   ```python
   # Create a simple generator instance
   generator = SyntheticDatasetGenerator("./test_dataset", "test_run")

   # Setup a simple scene
   scene_config = {
       "objects": [
           {"name": "test_cube", "type": "cube", "position": [0, 0.5, 0], "scale": [1, 1, 1],
            "semantic_label": "obstacle", "semantic_id": 1, "movable": False}
       ],
       "lighting": {
           "color": (0.9, 0.9, 0.9),
           "intensity": 3000,
           "directional_light": True
       }
   }

   generator.setup_scene(scene_config)
   # Generate a few test frames to verify the pipeline
   ```

2. **Check generated files**:
   - Verify files are created in the expected directories
   - Check that RGB, depth, and segmentation images are properly generated
   - Validate metadata files contain correct information

## Troubleshooting Common Issues

### Installation Issues
- **GPU Not Detected**: Ensure NVIDIA drivers are properly installed: `nvidia-smi`
- **CUDA Issues**: Verify CUDA installation: `nvcc --version`
- **Permission Issues**: Fix permissions if needed: `chmod +x isaac-sim.sh`

### Simulation Issues
- **Humanoid Falls Through Floor**: Check collision geometry and physics materials
- **Jittery Movement**: Reduce timestep or increase damping
- **Performance Problems**: Simplify collision geometry or reduce substeps

### Sensor Issues
- **Camera Not Publishing**: Verify ROS bridge extension is enabled and topics are set up
- **Low Frame Rate**: Reduce resolution or disable compression
- **Distorted Images**: Verify focal length and aperture settings

## Summary

This chapter covered the fundamentals of Isaac Sim for humanoid robot simulation:
- Installation and setup of Isaac Sim on Ubuntu 22.04
- USD fundamentals and humanoid model loading
- Physics and lighting configuration for realistic simulation
- RGB, depth, and segmentation sensor setup
- Synthetic dataset generation pipeline
- Performance optimization techniques

The next chapter will cover Isaac ROS for perception and VSLAM, building on the simulation foundation established here.