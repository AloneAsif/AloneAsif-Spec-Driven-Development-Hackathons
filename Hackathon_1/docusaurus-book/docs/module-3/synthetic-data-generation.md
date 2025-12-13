# Synthetic Dataset Generation Workflow

## Overview

Synthetic data generation in Isaac Sim enables the creation of large-scale, photorealistic datasets with perfect ground truth annotations for training perception algorithms. This workflow combines RGB, depth, and segmentation sensors to generate comprehensive multi-modal datasets for robotics applications.

## Dataset Generation Fundamentals

### Key Components of Synthetic Datasets
1. **RGB Images**: Color imagery for object detection and recognition
2. **Depth Maps**: 3D spatial information for depth estimation
3. **Segmentation Masks**: Pixel-level object classification
4. **Metadata**: Camera parameters, poses, and scene information
5. **Annotations**: Bounding boxes, keypoints, and other labels

### Benefits of Synthetic Data
- **Perfect Ground Truth**: Accurate annotations without manual labeling
- **Infinite Variability**: Control over lighting, objects, and scenarios
- **Safety**: Test in dangerous scenarios without risk
- **Cost-Effective**: Generate large datasets without manual effort
- **Edge Cases**: Create rare scenarios for robust training

## Setting Up the Data Generation Pipeline

### Creating a Data Generation Environment

```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.isaac.core.utils.prims import create_prim, get_prim_at_path
from omni.isaac.synthetic_utils import DataWriter
import numpy as np
import os
from PIL import Image

class SyntheticDatasetGenerator:
    def __init__(self, output_dir, dataset_name="synthetic_dataset"):
        """
        Initialize synthetic dataset generator

        Args:
            output_dir: Directory to save generated datasets
            dataset_name: Name of the dataset
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

        Args:
            scene_config: Dictionary with scene configuration
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

    def _add_scene_object(self, obj_config):
        """Add an object to the scene with semantic labeling"""
        obj_path = f"/World/{obj_config['name']}"

        # Create object based on type
        if obj_config["type"] == "cube":
            create_prim(
                prim_path=obj_path,
                prim_type="Cube",
                position=obj_config["position"],
                scale=obj_config["scale"],
                attributes={"visibility": "inherited"}
            )
        elif obj_config["type"] == "sphere":
            create_prim(
                prim_path=obj_path,
                prim_type="Sphere",
                position=obj_config["position"],
                scale=obj_config["scale"],
                attributes={"visibility": "inherited"}
            )
        elif obj_config["type"] == "cylinder":
            create_prim(
                prim_path=obj_path,
                prim_type="Cylinder",
                position=obj_config["position"],
                scale=obj_config["scale"],
                attributes={"visibility": "inherited"}
            )

        # Assign semantic label
        from pxr import UsdGeom
        obj_prim = get_prim_at_path(obj_path)
        obj_prim.GetAttribute("primvars:semantic:label").Set(obj_config["semantic_label"])
        obj_prim.GetAttribute("primvars:semantic:id").Set(obj_config["semantic_id"])

        self.scene_objects.append(obj_config)
        print(f"Added {obj_config['type']} object: {obj_config['name']}")

    def _setup_lighting(self, lighting_config):
        """Set up lighting for the scene"""
        # Add dome light
        create_prim(
            prim_path="/World/DomeLight",
            prim_type="DomeLight",
            attributes={
                "color": lighting_config.get("color", (0.9, 0.9, 0.9)),
                "intensity": lighting_config.get("intensity", 3000),
                "enableColorTemperature": False
            }
        )

        # Add directional light if specified
        if lighting_config.get("directional_light", False):
            create_prim(
                prim_path="/World/DirectionalLight",
                prim_type="DistantLight",
                position=[0, 0, 10],
                orientation=[0.707, 0, -0.707, 0],
                attributes={
                    "color": (0.95, 0.9, 0.8),
                    "intensity": 500,
                    "angle": 0.5
                }
            )

        print("Lighting setup complete")
```

### Configuring Multi-Modal Sensors

```python
    def setup_sensors(self, sensor_config):
        """
        Set up RGB, depth, and segmentation sensors

        Args:
            sensor_config: Dictionary with sensor configuration
        """
        # Add RGB camera
        rgb_config = sensor_config.get("rgb", {})
        self.rgb_camera = self._add_rgb_camera(
            "/World/sensors/rgb_camera",
            rgb_config.get("position", [2, 2, 2]),
            rgb_config.get("orientation", [0.354, 0.354, 0.146, 0.854]),  # Looking at origin
            rgb_config.get("resolution", {"width": 640, "height": 480})
        )

        # Add depth camera
        depth_config = sensor_config.get("depth", {})
        self.depth_camera = self._add_depth_sensor(
            "/World/sensors/depth_camera",
            depth_config.get("position", [2, 2, 2]),
            depth_config.get("orientation", [0.354, 0.354, 0.146, 0.854]),
            depth_config.get("resolution", {"width": 640, "height": 480}),
            depth_config.get("range", {"min": 0.1, "max": 10.0})
        )

        # Add segmentation camera
        seg_config = sensor_config.get("segmentation", {})
        self.seg_camera = self._add_segmentation_sensor(
            "/World/sensors/seg_camera",
            seg_config.get("position", [2, 2, 2]),
            seg_config.get("orientation", [0.354, 0.354, 0.146, 0.854]),
            seg_config.get("resolution", {"width": 640, "height": 480})
        )

        # Set up ROS bridge if needed
        if sensor_config.get("enable_ros_bridge", False):
            self._setup_ros_bridge()

        print("Multi-modal sensors setup complete")

    def _add_rgb_camera(self, prim_path, position, orientation, resolution):
        """Add RGB camera sensor"""
        from pxr import Gf
        camera_prim = create_prim(
            prim_path=prim_path,
            prim_type="Camera",
            position=position,
            orientation=orientation,
            attributes={
                "focalLength": 24.0,
                "horizontalAperture": 36.0,
                "verticalAperture": 20.25,
                "clippingRange": (0.01, 1000000.0)
            }
        )

        # Set resolution
        cam_prim = get_prim_at_path(prim_path)
        cam_prim.GetAttribute("customData:resolution:width").Set(resolution["width"])
        cam_prim.GetAttribute("customData:resolution:height").Set(resolution["height"])

        return prim_path

    def _add_depth_sensor(self, prim_path, position, orientation, resolution, depth_range):
        """Add depth sensor"""
        camera_prim = create_prim(
            prim_path=prim_path,
            prim_type="Camera",
            position=position,
            orientation=orientation,
            attributes={
                "focalLength": 24.0,
                "horizontalAperture": 36.0,
                "verticalAperture": 20.25,
                "clippingRange": (depth_range["min"], depth_range["max"])
            }
        )

        # Set depth-specific attributes
        depth_prim = get_prim_at_path(prim_path)
        depth_prim.GetAttribute("depth:enabled").Set(True)
        depth_prim.GetAttribute("depth:min_range").Set(depth_range["min"])
        depth_prim.GetAttribute("depth:max_range").Set(depth_range["max"])
        depth_prim.GetAttribute("customData:resolution:width").Set(resolution["width"])
        depth_prim.GetAttribute("customData:resolution:height").Set(resolution["height"])

        return prim_path

    def _add_segmentation_sensor(self, prim_path, position, orientation, resolution):
        """Add segmentation sensor"""
        camera_prim = create_prim(
            prim_path=prim_path,
            prim_type="Camera",
            position=position,
            orientation=orientation,
            attributes={
                "focalLength": 24.0,
                "horizontalAperture": 36.0,
                "verticalAperture": 20.25,
                "clippingRange": (0.01, 1000000.0)
            }
        )

        # Set segmentation-specific attributes
        seg_prim = get_prim_at_path(prim_path)
        seg_prim.GetAttribute("segmentation:enabled").Set(True)
        seg_prim.GetAttribute("segmentation:modality").Set("semantic")
        seg_prim.GetAttribute("segmentation:format").Set("rgb")
        seg_prim.GetAttribute("customData:resolution:width").Set(resolution["width"])
        seg_prim.GetAttribute("customData:resolution:height").Set(resolution["height"])

        return prim_path

    def _setup_ros_bridge(self):
        """Set up ROS bridge for sensor data publishing"""
        from omni.isaac.core.utils.extensions import enable_extension
        enable_extension("omni.isaac.ros2_bridge")

        # Configure RGB camera for ROS
        rgb_cam = get_prim_at_path(self.rgb_camera)
        rgb_cam.GetAttribute("ros2:topicName").Set("/camera/rgb/image_raw")
        rgb_cam.GetAttribute("ros2:modality").Set("rgb")
        rgb_cam.GetAttribute("ros2:enabled").Set(True)

        # Configure depth camera for ROS
        depth_cam = get_prim_at_path(self.depth_camera)
        depth_cam.GetAttribute("ros2:topicName").Set("/camera/depth/image_raw")
        depth_cam.GetAttribute("ros2:modality").Set("depth")
        depth_cam.GetAttribute("ros2:enabled").Set(True)

        # Configure segmentation camera for ROS
        seg_cam = get_prim_at_path(self.seg_camera)
        seg_cam.GetAttribute("ros2:topicName").Set("/camera/segmentation/image_raw")
        seg_cam.GetAttribute("ros2:modality").Set("segmentation_semantic")
        seg_cam.GetAttribute("ros2:enabled").Set(True)

        print("ROS bridge setup complete")
```

## Data Capture and Storage Pipeline

### Capturing Multi-Modal Data

```python
    def capture_frame(self, frame_id):
        """
        Capture a single frame of multi-modal data

        Args:
            frame_id: Unique identifier for the frame

        Returns:
            dict: Captured data with RGB, depth, segmentation, and metadata
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

    def _get_camera_intrinsics(self):
        """Get camera intrinsic matrix"""
        import numpy as np

        # Standard camera intrinsics (these would be retrieved from the camera prim in real implementation)
        fx = fy = 300.0  # Focal length in pixels
        cx = 320.0       # Principal point x
        cy = 240.0       # Principal point y

        intrinsic_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])

        return intrinsic_matrix

    def _get_camera_position(self):
        """Get camera position in world coordinates"""
        # This would be retrieved from the camera prim in real implementation
        return [2.0, 2.0, 2.0]

    def save_frame_data(self, frame_data, frame_id):
        """
        Save captured frame data to the appropriate directories

        Args:
            frame_data: Dictionary containing RGB, depth, segmentation, and metadata
            frame_id: Unique identifier for the frame
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

    def generate_dataset(self, num_frames=1000, frame_interval=0.1, capture_callback=None):
        """
        Generate synthetic dataset by capturing multiple frames

        Args:
            num_frames: Number of frames to capture
            frame_interval: Time interval between frames in seconds
            capture_callback: Optional callback function for each capture
        """
        import time

        print(f"Starting dataset generation: {num_frames} frames")

        for frame_id in range(num_frames):
            # Capture frame
            frame_data = self.capture_frame(frame_id)

            # Optionally modify scene between frames (for variety)
            if frame_id % 10 == 0:  # Every 10 frames, move some objects
                self._perturb_scene()

            # Save frame data
            self.save_frame_data(frame_data, frame_id)

            # Optional callback
            if capture_callback:
                capture_callback(frame_id, frame_data)

            # Wait for interval
            time.sleep(frame_interval)

            # Progress update
            if (frame_id + 1) % 100 == 0:
                print(f"Captured {frame_id + 1}/{num_frames} frames...")

        print(f"Dataset generation complete: {num_frames} frames saved to {self.output_dir}/{self.dataset_name}")

    def _perturb_scene(self):
        """Perturb the scene to create variety in the dataset"""
        import random
        import numpy as np

        # Move objects slightly for variety
        for obj in self.scene_objects:
            if obj.get("movable", False):
                current_pos = obj["position"]
                # Add small random offset
                new_pos = [
                    current_pos[0] + random.uniform(-0.1, 0.1),
                    current_pos[1] + random.uniform(-0.1, 0.1),
                    current_pos[2] + random.uniform(-0.1, 0.1)
                ]

                # Update object position
                obj_prim = get_prim_at_path(f"/World/{obj['name']}")
                obj_prim.GetAttribute("xformOp:translate").Set(new_pos)
                obj["position"] = new_pos
```

## Advanced Dataset Generation Techniques

### Variational Data Generation

```python
class AdvancedDatasetGenerator(SyntheticDatasetGenerator):
    def __init__(self, output_dir, dataset_name="advanced_synthetic_dataset"):
        super().__init__(output_dir, dataset_name)
        self.variation_params = {
            "lighting": {
                "intensity_range": (1000, 5000),
                "color_temperature_range": (3000, 8000)
            },
            "object_poses": {
                "position_jitter": 0.2,
                "rotation_jitter": 15.0  # degrees
            },
            "camera_poses": {
                "orbit_radius_range": (1.5, 3.0),
                "height_range": (1.0, 3.0)
            }
        }

    def setup_variational_scene(self, base_scene_config, variations=100):
        """
        Set up scene with automatic variations for data augmentation

        Args:
            base_scene_config: Base scene configuration
            variations: Number of scene variations to create
        """
        self.variations = variations
        self.base_config = base_scene_config

        # Create initial scene
        self.setup_scene(base_scene_config)

        print(f"Variational scene setup complete with {variations} potential variations")

    def generate_variational_dataset(self, num_frames_per_variation=10, total_frames=1000):
        """
        Generate dataset with automatic scene variations

        Args:
            num_frames_per_variation: Number of frames per scene variation
            total_frames: Total number of frames to generate
        """
        import random

        frames_per_variation = total_frames // self.variations
        remaining_frames = total_frames % self.variations

        print(f"Generating {total_frames} frames with {self.variations} scene variations")
        print(f"Approximately {frames_per_variation} frames per variation")

        frame_count = 0
        for var_idx in range(self.variations):
            # Apply scene variation
            self._apply_scene_variation(var_idx)

            # Capture frames for this variation
            frames_for_this_var = frames_per_variation
            if var_idx < remaining_frames:
                frames_for_this_var += 1  # Distribute remaining frames

            for frame_in_var in range(frames_for_this_var):
                frame_data = self.capture_frame(frame_count)
                self.save_frame_data(frame_data, frame_count)

                frame_count += 1

                if frame_count % 100 == 0:
                    print(f"Captured {frame_count}/{total_frames} frames...")

        print(f"Variational dataset generation complete: {total_frames} frames")

    def _apply_scene_variation(self, var_idx):
        """Apply variation to the scene based on variation index"""
        import random

        # Vary lighting
        dome_light = get_prim_at_path("/World/DomeLight")
        new_intensity = random.uniform(*self.variation_params["lighting"]["intensity_range"])
        dome_light.GetAttribute("inputs:intensity").Set(new_intensity)

        # Vary object positions
        for obj in self.scene_objects:
            if obj.get("movable", True):
                base_pos = obj.get("base_position", obj["position"])
                jitter = self.variation_params["object_poses"]["position_jitter"]

                new_pos = [
                    base_pos[0] + random.uniform(-jitter, jitter),
                    base_pos[1] + random.uniform(-jitter, jitter),
                    base_pos[2] + random.uniform(-jitter, jitter)
                ]

                obj_prim = get_prim_at_path(f"/World/{obj['name']}")
                obj_prim.GetAttribute("xformOp:translate").Set(new_pos)
                obj["position"] = new_pos

        # Vary camera position
        self._vary_camera_position(var_idx)

    def _vary_camera_position(self, var_idx):
        """Vary camera position for different viewpoints"""
        import math
        import random

        # Calculate orbit position around center
        radius = random.uniform(*self.variation_params["camera_poses"]["orbit_radius_range"])
        height = random.uniform(*self.variation_params["camera_poses"]["height_range"])
        angle = (var_idx / self.variations) * 2 * math.pi  # Evenly distribute around circle

        x = radius * math.cos(angle)
        y = height
        z = radius * math.sin(angle)

        # Update all camera positions
        for cam_path in [self.rgb_camera, self.depth_camera, self.seg_camera]:
            cam_prim = get_prim_at_path(cam_path)
            cam_prim.GetAttribute("xformOp:translate").Set([x, y, z])
```

## Data Quality and Validation

### Dataset Validation Pipeline

```python
def validate_synthetic_dataset(dataset_path, validation_config=None):
    """
    Validate the quality and completeness of a synthetic dataset

    Args:
        dataset_path: Path to the dataset directory
        validation_config: Configuration for validation parameters

    Returns:
        dict: Validation results
    """
    import os
    import json
    import numpy as np
    from PIL import Image

    if validation_config is None:
        validation_config = {
            "expected_modalities": ["rgb", "depth", "segmentation"],
            "min_depth_range": 0.1,
            "max_depth_range": 10.0,
            "expected_objects": [],
            "min_resolution": (640, 480)
        }

    # Check directory structure
    required_dirs = ["images", "depth", "segmentation", "metadata"]
    missing_dirs = []
    for dir_name in required_dirs:
        dir_path = os.path.join(dataset_path, dir_name)
        if not os.path.exists(dir_path):
            missing_dirs.append(dir_name)

    if missing_dirs:
        print(f"ERROR: Missing directories: {missing_dirs}")
        return {"valid": False, "errors": [f"Missing directories: {missing_dirs}"]}

    # Count files
    image_files = [f for f in os.listdir(os.path.join(dataset_path, "images")) if f.endswith(('.png', '.jpg', '.jpeg'))]
    depth_files = [f for f in os.listdir(os.path.join(dataset_path, "depth")) if f.endswith(('.png', '.tiff', '.tif'))]
    seg_files = [f for f in os.listdir(os.path.join(dataset_path, "segmentation")) if f.endswith(('.png', '.tiff', '.tif'))]
    meta_files = [f for f in os.listdir(os.path.join(dataset_path, "metadata")) if f.endswith('.json')]

    # Check that all modalities have the same number of files
    file_counts = [len(image_files), len(depth_files), len(seg_files), len(meta_files)]
    if len(set(file_counts)) > 1:
        print(f"ERROR: Inconsistent file counts: images={len(image_files)}, depth={len(depth_files)}, seg={len(seg_files)}, meta={len(meta_files)}")
        return {"valid": False, "errors": [f"Inconsistent file counts: {file_counts}"]}

    total_frames = len(image_files)
    print(f"Dataset validation: {total_frames} frames found")

    # Validate individual files
    validation_results = {
        "valid": True,
        "total_frames": total_frames,
        "errors": [],
        "warnings": [],
        "statistics": {}
    }

    # Sample validation on first few files
    sample_size = min(10, total_frames)
    for i in range(sample_size):
        frame_id = i
        try:
            # Validate RGB image
            rgb_path = os.path.join(dataset_path, "images", f"frame_{frame_id:06d}.png")
            if os.path.exists(rgb_path):
                rgb_img = Image.open(rgb_path)
                if rgb_img.mode != 'RGB':
                    validation_results["warnings"].append(f"Frame {frame_id}: RGB image not in RGB mode")
            else:
                validation_results["errors"].append(f"Frame {frame_id}: RGB image missing")

            # Validate depth image
            depth_path = os.path.join(dataset_path, "depth", f"frame_{frame_id:06d}.tiff")
            if os.path.exists(depth_path):
                depth_img = Image.open(depth_path)
                depth_array = np.array(depth_img)
                valid_depths = depth_array[depth_array > 0]
                if len(valid_depths) == 0:
                    validation_results["warnings"].append(f"Frame {frame_id}: No valid depth values")
                else:
                    depth_min, depth_max = valid_depths.min(), valid_depths.max()
                    if depth_min < validation_config["min_depth_range"]:
                        validation_results["warnings"].append(f"Frame {frame_id}: Depth below minimum ({depth_min} < {validation_config['min_depth_range']})")
                    if depth_max > validation_config["max_depth_range"]:
                        validation_results["warnings"].append(f"Frame {frame_id}: Depth above maximum ({depth_max} > {validation_config['max_depth_range']})")
            else:
                validation_results["errors"].append(f"Frame {frame_id}: Depth image missing")

            # Validate segmentation image
            seg_path = os.path.join(dataset_path, "segmentation", f"frame_{frame_id:06d}.png")
            if os.path.exists(seg_path):
                seg_img = Image.open(seg_path)
                seg_array = np.array(seg_img)
                unique_labels = np.unique(seg_array)
                if len(unique_labels) < 2:  # Should have background + objects
                    validation_results["warnings"].append(f"Frame {frame_id}: Segmentation has too few unique labels ({len(unique_labels)})")
            else:
                validation_results["errors"].append(f"Frame {frame_id}: Segmentation image missing")

            # Validate metadata
            meta_path = os.path.join(dataset_path, "metadata", f"frame_{frame_id:06d}.json")
            if os.path.exists(meta_path):
                with open(meta_path, 'r') as f:
                    meta_data = json.load(f)
                    required_keys = ["frame_id", "timestamp", "camera_intrinsics", "capture_settings"]
                    for key in required_keys:
                        if key not in meta_data:
                            validation_results["warnings"].append(f"Frame {frame_id}: Missing metadata key '{key}'")
            else:
                validation_results["errors"].append(f"Frame {frame_id}: Metadata file missing")

        except Exception as e:
            validation_results["errors"].append(f"Frame {frame_id}: Error validating - {str(e)}")

    # Generate statistics
    validation_results["statistics"] = {
        "total_frames": total_frames,
        "sample_validated": sample_size,
        "errors_count": len(validation_results["errors"]),
        "warnings_count": len(validation_results["warnings"])
    }

    if validation_results["errors"]:
        validation_results["valid"] = False
        print(f"Dataset validation FAILED with {len(validation_results['errors'])} errors")
    else:
        print(f"Dataset validation PASSED - {total_frames} frames validated successfully")

    return validation_results
```

## Performance Optimization

### Optimizing Data Generation Performance

```python
def optimize_data_generation_performance(generator, performance_config=None):
    """
    Optimize synthetic data generation performance

    Args:
        generator: SyntheticDatasetGenerator instance
        performance_config: Configuration for performance optimization
    """
    if performance_config is None:
        performance_config = {
            "resolution_scale": 0.5,  # Scale down resolution for speed
            "enable_compression": True,
            "max_fps": 30,
            "batch_size": 10,  # Process in batches
            "use_gpu_processing": True
        }

    # Optimize sensor settings
    for sensor_path in [generator.rgb_camera, generator.depth_camera, generator.seg_camera]:
        sensor_prim = get_prim_at_path(sensor_path)

        # Scale resolution
        current_width = sensor_prim.GetAttribute("customData:resolution:width").Get()
        current_height = sensor_prim.GetAttribute("customData:resolution:height").Get()

        if current_width and current_height:
            new_width = int(current_width * performance_config["resolution_scale"])
            new_height = int(current_height * performance_config["resolution_scale"])

            sensor_prim.GetAttribute("customData:resolution:width").Set(new_width)
            sensor_prim.GetAttribute("customData:resolution:height").Set(new_height)

        # Enable compression
        sensor_prim.GetAttribute("performance:enable_compression").Set(performance_config["enable_compression"])

        # Set max FPS
        sensor_prim.GetAttribute("performance:max_fps").Set(performance_config["max_fps"])

    print(f"Performance optimization applied:")
    print(f"  - Resolution scaled to {performance_config['resolution_scale'] * 100}%")
    print(f"  - Compression enabled: {performance_config['enable_compression']}")
    print(f"  - Max FPS: {performance_config['max_fps']}")

def batch_generate_dataset(generator, batch_size=50, num_batches=20):
    """
    Generate dataset in batches for better memory management

    Args:
        generator: SyntheticDatasetGenerator instance
        batch_size: Number of frames per batch
        num_batches: Number of batches to generate
    """
    import gc

    total_frames = batch_size * num_batches
    print(f"Starting batch dataset generation: {num_batches} batches of {batch_size} frames each")

    for batch_idx in range(num_batches):
        print(f"Processing batch {batch_idx + 1}/{num_batches}")

        # Generate frames for this batch
        for frame_in_batch in range(batch_size):
            frame_id = batch_idx * batch_size + frame_in_batch
            frame_data = generator.capture_frame(frame_id)
            generator.save_frame_data(frame_data, frame_id)

        # Clean up memory
        gc.collect()

        print(f"Completed batch {batch_idx + 1}/{num_batches}")

    print(f"Batch dataset generation complete: {total_frames} frames")
```

## Real-World Application Examples

### Object Detection Dataset Example

```python
def create_object_detection_dataset():
    """
    Example of creating a dataset for object detection training
    """
    # Configuration for object detection dataset
    detection_config = {
        "objects": [
            {"name": "box1", "type": "cube", "position": [1, 0.5, 1], "scale": [0.3, 0.3, 0.3],
             "semantic_label": "box", "semantic_id": 1, "movable": True},
            {"name": "cylinder1", "type": "cylinder", "position": [-1, 0.3, 0.5], "scale": [0.2, 0.6, 0.2],
             "semantic_label": "cylinder", "semantic_id": 2, "movable": True},
            {"name": "sphere1", "type": "sphere", "position": [0, 0.4, -1], "scale": [0.25, 0.25, 0.25],
             "semantic_label": "sphere", "semantic_id": 3, "movable": True}
        ],
        "lighting": {
            "color": (0.9, 0.9, 0.9),
            "intensity": 3000,
            "directional_light": True
        }
    }

    # Sensor configuration
    sensor_config = {
        "rgb": {"position": [2, 2, 2], "resolution": {"width": 640, "height": 480}},
        "depth": {"position": [2, 2, 2], "resolution": {"width": 640, "height": 480}, "range": {"min": 0.1, "max": 5.0}},
        "segmentation": {"position": [2, 2, 2], "resolution": {"width": 640, "height": 480}},
        "enable_ros_bridge": False
    }

    # Create generator and setup
    generator = AdvancedDatasetGenerator("./datasets", "object_detection_dataset")
    generator.setup_variational_scene(detection_config, variations=50)
    generator.setup_sensors(sensor_config)

    # Generate dataset
    generator.generate_variational_dataset(num_frames_per_variation=20, total_frames=1000)

    print("Object detection dataset generation complete")

# Example usage
# create_object_detection_dataset()
```

### Navigation Dataset Example

```python
def create_navigation_dataset():
    """
    Example of creating a dataset for navigation training
    """
    # Configuration for navigation dataset
    navigation_config = {
        "objects": [
            {"name": "obstacle1", "type": "cube", "position": [2, 0.5, 0], "scale": [0.5, 1.0, 0.5],
             "semantic_label": "obstacle", "semantic_id": 1, "movable": False},
            {"name": "obstacle2", "type": "cube", "position": [-1, 0.5, 2], "scale": [0.3, 1.0, 0.3],
             "semantic_label": "obstacle", "semantic_id": 1, "movable": False},
            {"name": "goal", "type": "sphere", "position": [3, 0.5, 3], "scale": [0.3, 0.3, 0.3],
             "semantic_label": "goal", "semantic_id": 2, "movable": False}
        ],
        "lighting": {
            "color": (0.8, 0.8, 0.9),
            "intensity": 2000,
            "directional_light": True
        }
    }

    # Sensor configuration for navigation (robot-centric)
    sensor_config = {
        "rgb": {"position": [0, 1, 0], "orientation": [0.707, 0, 0.707, 0], "resolution": {"width": 640, "height": 480}},
        "depth": {"position": [0, 1, 0], "orientation": [0.707, 0, 0.707, 0], "resolution": {"width": 640, "height": 480}, "range": {"min": 0.1, "max": 10.0}},
        "segmentation": {"position": [0, 1, 0], "orientation": [0.707, 0, 0.707, 0], "resolution": {"width": 640, "height": 480}},
        "enable_ros_bridge": True
    }

    # Create generator and setup
    generator = AdvancedDatasetGenerator("./datasets", "navigation_dataset")
    generator.setup_variational_scene(navigation_config, variations=30)
    generator.setup_sensors(sensor_config)

    # Generate dataset
    generator.generate_variational_dataset(num_frames_per_variation=30, total_frames=900)

    print("Navigation dataset generation complete")

# Example usage
# create_navigation_dataset()
```

## Troubleshooting Common Issues

### Data Generation Issues and Solutions

#### 1. Empty or Invalid Data
**Symptoms**: Generated images are empty, black, or contain invalid values
**Solutions**:
- Check that objects are within camera view and clipping range
- Verify that semantic labels are properly assigned
- Ensure sufficient lighting in the scene
- Validate camera parameters and positioning

#### 2. Performance Issues
**Symptoms**: Slow frame rates or high memory usage
**Solutions**:
- Reduce image resolution
- Enable compression for sensor data
- Use batch processing for large datasets
- Optimize scene complexity

#### 3. Inconsistent Annotations
**Symptoms**: Mismatched RGB, depth, and segmentation data
**Solutions**:
- Ensure all sensors are properly synchronized
- Verify camera calibration parameters
- Check that all sensors have the same pose and timing

#### 4. File Format Issues
**Symptoms**: Problems saving or loading generated data
**Solutions**:
- Use appropriate file formats for data types (TIFF for depth, PNG for segmentation)
- Verify file permissions and paths
- Check that data types are correctly converted

## Best Practices

### Synthetic Dataset Generation Best Practices
1. **Scene Diversity**: Include varied lighting, objects, and scenarios
2. **Realistic Physics**: Use physically accurate simulations
3. **Ground Truth Quality**: Ensure perfect annotations and calibration
4. **Validation**: Regularly validate dataset quality and completeness
5. **Metadata**: Include comprehensive metadata for each frame
6. **Performance**: Balance quality with generation speed

### Data Pipeline Management
1. **Modular Design**: Separate scene setup, data capture, and storage
2. **Configuration**: Use configuration files for reproducible results
3. **Monitoring**: Track generation progress and quality metrics
4. **Error Handling**: Implement robust error handling and recovery
5. **Scalability**: Design for large-scale dataset generation

## Next Steps

After setting up synthetic data generation:
1. Configure [Performance Optimization](./performance-optimization.md)
2. Learn about [Isaac ROS Integration](./isaac-ros-perception.md)
3. Test [Humanoid Model Loading](./humanoid-model-loading.md) with generated data
4. Validate the complete [Synthetic Data Pipeline](./synthetic-data-validation.md)

## Additional Resources

- [Isaac Sim Synthetic Data Generation Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_synthetic_data.html)
- [Synthetic Datasets for Computer Vision](https://research.nvidia.com/publication/2020-06_Synthetic-Datasets-Training)
- [ROS 2 Image Pipeline](https://wiki.ros.org/image_pipeline)
- [Data Generation Best Practices](https://arxiv.org/abs/1908.06405)