# Semantic Segmentation Sensor Setup in Isaac Sim

## Overview

Semantic segmentation sensors in Isaac Sim provide pixel-level classification of objects in the scene, which is crucial for robotics perception tasks such as object detection, scene understanding, and training data generation for deep learning models. These sensors generate per-pixel labels that identify different object classes in the scene.

## Segmentation Sensor Fundamentals

### Segmentation Types
Isaac Sim supports several types of segmentation:
1. **Semantic Segmentation**: Each pixel is labeled with its object class
2. **Instance Segmentation**: Each pixel is labeled with the specific object instance
3. **Panoptic Segmentation**: Combination of semantic and instance segmentation

### Segmentation Data Formats
- **Color-coded**: Each class assigned a unique RGB color
- **ID-based**: Each class assigned a unique integer ID
- **One-hot encoded**: Binary masks for each class

## Setting Up Semantic Segmentation Sensors

### Method 1: Adding Segmentation via Python API

```python
import omni
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import get_current_stage
from pxr import Gf, Sdf

def add_segmentation_sensor(prim_path, position, orientation, config=None):
    """
    Add a semantic segmentation sensor to the scene

    Args:
        prim_path: Path for the segmentation sensor prim
        position: [x, y, z] position in world coordinates
        orientation: [x, y, z, w] quaternion orientation
        config: Dictionary with segmentation configuration
    """
    if config is None:
        config = {
            "resolution": {"width": 640, "height": 480},
            "focal_length": 24.0,
            "modality": "segmentation",  # segmentation, instance, panoptic
            "format": "rgb"  # rgb (color-coded) or id (integer IDs)
        }

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
    seg_prim = get_prim_at_path(prim_path)
    seg_prim.GetAttribute("segmentation:enabled").Set(True)
    seg_prim.GetAttribute("segmentation:modality").Set(config["modality"])
    seg_prim.GetAttribute("segmentation:format").Set(config["format"])
    seg_prim.GetAttribute("sensor:modality").Set("segmentation")

    return seg_sensor

# Example: Add segmentation sensor to humanoid head
seg_camera = add_segmentation_sensor(
    prim_path="/World/HumanoidRobot/head/seg_camera",
    position=[0, 0.1, 0],  # Slightly above the head
    orientation=[0.707, 0, 0.707, 0],  # Looking forward
    config={
        "resolution": {"width": 640, "height": 480},
        "focal_length": 24.0,
        "modality": "semantic",
        "format": "rgb"
    }
)
```

### Method 2: Adding Segmentation via USD Schema

```usda
#usda 1.0

def Camera "seg_camera"
{
    float3 xformOp:translate = (0.0, 0.1, 0.0)
    uniform token[] xformOpOrder = ["xformOp:translate"]

    # Camera properties for segmentation
    float focalLength = 24.0
    float horizontalAperture = 36.0
    float verticalAperture = 20.25
    float2 clippingRange = (0.01, 1000000.0)

    # Segmentation sensor properties
    bool segmentation:enabled = true
    string segmentation:modality = "semantic"
    string segmentation:format = "rgb"
    string sensor:modality = "segmentation"
    bool sensor:is_primary = true
}
```

## Assigning Semantic Labels to Objects

### Method 1: Using USD Schema for Semantic Labels

```usda
# Example of adding semantic labels to objects
def Xform "HumanoidRobot"
{
    # Robot parts with semantic labels
    def Xform "torso"
    {
        # Semantic label assignment
        string primvars:semantic:role = "robot"
        string primvars:semantic:label = "torso"
        int primvars:semantic:id = 1
    }

    def Xform "left_arm"
    {
        string primvars:semantic:role = "robot"
        string primvars:semantic:label = "arm"
        int primvars:semantic:id = 2
    }

    def Xform "right_arm"
    {
        string primvars:semantic:role = "robot"
        string primvars:semantic:label = "arm"
        int primvars:semantic:id = 2
    }
}

# Environment objects with semantic labels
def Xform "Environment"
{
    def Cube "floor"
    {
        string primvars:semantic:role = "environment"
        string primvars:semantic:label = "floor"
        int primvars:semantic:id = 10
    }

    def Sphere "ball"
    {
        string primvars:semantic:role = "object"
        string primvars:semantic:label = "ball"
        int primvars:semantic:id = 20
    }
}
```

### Method 2: Assigning Semantic Labels via Python API

```python
def assign_semantic_label(prim_path, label, semantic_id, role="object"):
    """
    Assign semantic label to a prim

    Args:
        prim_path: Path to the prim to label
        label: Semantic label name (e.g., "robot", "floor", "obstacle")
        semantic_id: Unique integer ID for this class
        role: Semantic role (e.g., "robot", "environment", "object")
    """
    prim = get_prim_at_path(prim_path)

    # Set semantic attributes
    prim.GetAttribute("primvars:semantic:label").Set(label)
    prim.GetAttribute("primvars:semantic:id").Set(semantic_id)
    prim.GetAttribute("primvars:semantic:role").Set(role)

    # Alternative: Set as custom data
    prim.GetCustomDataByKey("semantic_label").Set(label)
    prim.GetCustomDataByKey("semantic_id").Set(semantic_id)

    print(f"Assigned semantic label '{label}' (ID: {semantic_id}, Role: {role}) to {prim_path}")

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

## Semantic Label Configuration and Color Mapping

### Creating Semantic Label Definitions

```python
def create_semantic_label_definitions():
    """
    Create semantic label definitions for the scene
    """
    # Define semantic classes and their color mappings
    semantic_classes = {
        0: {"name": "background", "color": [0, 0, 0], "id": 0},
        1: {"name": "robot_torso", "color": [255, 0, 0], "id": 1},      # Red
        2: {"name": "robot_head", "color": [0, 255, 0], "id": 2},       # Green
        3: {"name": "robot_arm", "color": [0, 0, 255], "id": 3},        # Blue
        4: {"name": "robot_leg", "color": [255, 255, 0], "id": 4},      # Yellow
        10: {"name": "floor", "color": [100, 100, 100], "id": 10},      # Gray
        20: {"name": "wall", "color": [150, 75, 0], "id": 20},          # Brown
        30: {"name": "obstacle", "color": [255, 0, 255], "id": 30},     # Magenta
        40: {"name": "furniture", "color": [0, 255, 255], "id": 40},    # Cyan
    }

    return semantic_classes

def generate_color_map(semantic_classes):
    """
    Generate color map for semantic segmentation visualization
    """
    import numpy as np

    # Create a color map array
    max_id = max(semantic_classes.keys()) + 1
    color_map = np.zeros((max_id, 3), dtype=np.uint8)

    for sem_id, class_info in semantic_classes.items():
        color_map[sem_id] = class_info["color"]

    return color_map

# Create and use semantic label definitions
semantic_definitions = create_semantic_label_definitions()
color_map = generate_color_map(semantic_definitions)

print("Semantic label definitions created:")
for sem_id, class_info in semantic_definitions.items():
    print(f"  ID {sem_id}: {class_info['name']} -> {class_info['color']}")
```

### Configuring Segmentation Output Format

```python
def configure_segmentation_format(seg_sensor_path, output_format="color", encoding="rgb"):
    """
    Configure segmentation output format

    Args:
        seg_sensor_path: Path to segmentation sensor
        output_format: "color" for color-coded, "id" for integer IDs
        encoding: "rgb", "grayscale", or "binary_mask"
    """
    seg_prim = get_prim_at_path(seg_sensor_path)

    # Set output format
    seg_prim.GetAttribute("segmentation:format").Set(output_format)
    seg_prim.GetAttribute("segmentation:encoding").Set(encoding)

    # Configure additional format-specific attributes
    if output_format == "color":
        seg_prim.GetAttribute("segmentation:use_colormap").Set(True)
    elif output_format == "id":
        seg_prim.GetAttribute("segmentation:use_colormap").Set(False)
        seg_prim.GetAttribute("segmentation:data_type").Set("int32")

    print(f"Segmentation sensor configured: format={output_format}, encoding={encoding}")

# Configure segmentation formats
configure_segmentation_format("/World/HumanoidRobot/head/seg_camera", "color", "rgb")
```

## Instance Segmentation Setup

### Setting Up Instance Segmentation

```python
def add_instance_segmentation_sensor(prim_path, position, orientation, config=None):
    """
    Add an instance segmentation sensor to the scene

    Args:
        prim_path: Path for the instance segmentation sensor
        position: [x, y, z] position in world coordinates
        orientation: [x, y, z, w] quaternion orientation
        config: Dictionary with instance segmentation configuration
    """
    if config is None:
        config = {
            "resolution": {"width": 640, "height": 480},
            "focal_length": 24.0,
            "format": "id"  # Instance segmentation typically uses ID format
        }

    # Create the instance segmentation sensor
    instance_sensor = create_prim(
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

    # Add instance segmentation-specific attributes
    inst_prim = get_prim_at_path(prim_path)
    inst_prim.GetAttribute("segmentation:enabled").Set(True)
    inst_prim.GetAttribute("segmentation:modality").Set("instance")
    inst_prim.GetAttribute("segmentation:format").Set(config["format"])
    inst_prim.GetAttribute("sensor:modality").Set("instance_segmentation")

    return instance_sensor

def assign_instance_ids():
    """
    Assign unique instance IDs to objects for instance segmentation
    """
    # For instance segmentation, each individual object needs a unique ID
    # even if they're the same semantic class

    # Robot instances
    assign_semantic_label("/World/HumanoidRobot1", "robot", 1001, "robot_instance")
    assign_semantic_label("/World/HumanoidRobot2", "robot", 1002, "robot_instance")

    # Individual objects
    assign_semantic_label("/World/obstacle_box_1", "obstacle", 2001, "obstacle_instance")
    assign_semantic_label("/World/obstacle_box_2", "obstacle", 2002, "obstacle_instance")
    assign_semantic_label("/World/furniture_chair_1", "furniture", 3001, "furniture_instance")
    assign_semantic_label("/World/furniture_table_1", "furniture", 3002, "furniture_instance")

# Add instance segmentation sensor
inst_seg_sensor = add_instance_segmentation_sensor(
    prim_path="/World/HumanoidRobot/head/instance_seg",
    position=[0, 0.1, 0],
    orientation=[0.707, 0, 0.707, 0]
)

# Assign instance IDs
assign_instance_ids()
```

## Isaac Sim ROS 2 Bridge Integration

### Publishing Segmentation Data to ROS 2

```python
def setup_segmentation_ros_bridge(seg_sensor_path, topic_name, modality="semantic"):
    """
    Set up ROS 2 bridge for segmentation sensor data

    Args:
        seg_sensor_path: Path to the segmentation sensor in USD stage
        topic_name: ROS 2 topic name (e.g., "/camera/segmentation/image_raw")
        modality: "semantic" or "instance" segmentation
    """
    # Enable ROS bridge extension
    enable_extension("omni.isaac.ros2_bridge")

    # Configure segmentation sensor for ROS publishing
    seg_prim = get_prim_at_path(seg_sensor_path)

    # Set ROS-specific attributes for segmentation
    seg_prim.GetAttribute("ros2:topicName").Set(topic_name)
    seg_prim.GetAttribute("ros2:modality").Set(f"segmentation_{modality}")
    seg_prim.GetAttribute("ros2:enabled").Set(True)

    # Configure segmentation-specific ROS attributes
    seg_prim.GetAttribute("ros2:segmentation:encoding").Set("rgb8")  # or "32SC1" for ID format
    seg_prim.GetAttribute("ros2:segmentation:format").Set("color" if modality == "semantic" else "id")

    # Configure QoS settings appropriate for segmentation data
    qos_settings = {
        "reliability": "best_effort",  # Segmentation can tolerate some loss
        "durability": "volatile",
        "history": "keep_last",
        "depth": 1  # Only keep latest segmentation image
    }

    for attr, value in qos_settings.items():
        seg_prim.GetAttribute(f"ros2:qos:{attr}").Set(value)

    print(f"Segmentation sensor {seg_sensor_path} configured for ROS 2 publishing on {topic_name}")

# Configure segmentation sensors for ROS 2
setup_segmentation_ros_bridge("/World/HumanoidRobot/head/seg_camera", "/camera/semantic_segmentation", "semantic")
setup_segmentation_ros_bridge("/World/HumanoidRobot/head/instance_seg", "/camera/instance_segmentation", "instance")
```

## Data Capture and Processing

### Capturing Segmentation Data

```python
import cv2
import numpy as np
from omni.synthetic_utils.sensors import CameraSensor
from PIL import Image

def capture_segmentation_image(seg_sensor_path, output_path, format="png", color_map=None):
    """
    Capture and save segmentation image from Isaac Sim segmentation sensor

    Args:
        seg_sensor_path: Path to segmentation sensor prim
        output_path: Path to save the segmentation image
        format: Image format (png, tiff, npy)
        color_map: Optional color map for visualization
    """
    # Get segmentation sensor data
    seg_sensor = CameraSensor(seg_sensor_path)
    seg_data = seg_sensor.get_segmentation_data()

    if format.lower() == 'npy':
        # Save as numpy array for processing
        np.save(output_path, seg_data)
    elif format.lower() == 'png':
        if color_map is not None:
            # Apply color map for visualization
            h, w = seg_data.shape
            color_seg = np.zeros((h, w, 3), dtype=np.uint8)

            for sem_id in np.unique(seg_data):
                mask = seg_data == sem_id
                if sem_id < len(color_map):
                    color_seg[mask] = color_map[sem_id]

            # Save colorized segmentation
            cv2.imwrite(output_path, cv2.cvtColor(color_seg, cv2.COLOR_RGB2BGR))
        else:
            # Save as grayscale ID image
            seg_16bit = seg_data.astype(np.uint16)
            cv2.imwrite(output_path, seg_16bit, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    elif format.lower() == 'tiff':
        # Save as TIFF for higher precision
        seg_16bit = seg_data.astype(np.uint16)
        img = Image.fromarray(seg_16bit)
        img.save(output_path, format='TIFF')

    print(f"Segmentation image saved to {output_path}")

def capture_continuous_segmentation(seg_sensor_path, output_dir, num_frames=100, interval=0.1, color_map=None):
    """
    Capture continuous segmentation data at specified interval

    Args:
        seg_sensor_path: Path to segmentation sensor prim
        output_dir: Directory to save segmentation data
        num_frames: Number of frames to capture
        interval: Time interval between captures in seconds
        color_map: Optional color map for visualization
    """
    import time
    import os

    os.makedirs(output_dir, exist_ok=True)

    for i in range(num_frames):
        timestamp = int(time.time() * 1000)  # Millisecond timestamp
        output_path = f"{output_dir}/seg_{timestamp:0>13}.png"
        capture_segmentation_image(seg_sensor_path, output_path, format='png', color_map=color_map)

        # Wait for specified interval
        time.sleep(interval)

        if i % 10 == 0:
            print(f"Captured {i+1}/{num_frames} segmentation frames...")

    print(f"Continuous segmentation capture complete: {num_frames} frames saved to {output_dir}")

# Example usage
semantic_classes = create_semantic_label_definitions()
color_map = generate_color_map(semantic_classes)

# capture_continuous_segmentation("/World/HumanoidRobot/head/seg_camera", "./seg_dataset", num_frames=1000, interval=0.1, color_map=color_map)
```

## Multi-Camera Segmentation Setup

### Synchronizing Multiple Segmentation Sensors

```python
def setup_multi_view_segmentation(robot_path="/World/HumanoidRobot"):
    """
    Set up multiple segmentation sensors for comprehensive scene understanding

    Args:
        robot_path: Base path for the humanoid robot
    """
    # Front view segmentation
    front_seg = add_segmentation_sensor(
        prim_path=f"{robot_path}/head/front_seg",
        position=[0, 0.1, 0.1],
        orientation=[0.707, 0, 0.707, 0],
        config={"focal_length": 24.0, "modality": "semantic", "format": "rgb"}
    )

    # Side view segmentation
    side_seg = add_segmentation_sensor(
        prim_path=f"{robot_path}/shoulder/side_seg",
        position=[0.2, 0, 0],
        orientation=[0.5, 0.5, 0.5, 0.5],  # 90 degree rotation around Y
        config={"focal_length": 24.0, "modality": "semantic", "format": "rgb"}
    )

    # Top-down segmentation (if needed)
    top_seg = add_segmentation_sensor(
        prim_path=f"{robot_path}/head/top_seg",
        position=[0, 0.3, 0],
        orientation=[1, 0, 0, 0],  # Looking down
        config={"focal_length": 18.0, "modality": "semantic", "format": "rgb"}
    )

    print("Multi-view segmentation setup complete")
    return [front_seg, side_seg, top_seg]

def synchronize_segmentation_cameras(seg_cameras, sync_interval=1.0/30.0):
    """
    Synchronize multiple segmentation cameras for consistent timing

    Args:
        seg_cameras: List of segmentation camera paths
        sync_interval: Time interval between captures in seconds
    """
    for i, cam_path in enumerate(seg_cameras):
        cam_prim = get_prim_at_path(cam_path)

        # Set synchronization attributes
        cam_prim.GetAttribute("sync:enabled").Set(True)
        cam_prim.GetAttribute("sync:interval").Set(sync_interval)
        cam_prim.GetAttribute("sync:offset").Set(i * 0.001)  # Small offset to avoid conflicts

    print(f"Synchronized {len(seg_cameras)} segmentation cameras with {sync_interval}s interval")

# Set up multi-view segmentation
seg_cameras = setup_multi_view_segmentation()
synchronize_segmentation_cameras(seg_cameras, sync_interval=1.0/30.0)
```

## Performance Optimization for Segmentation

### Optimizing Segmentation Performance

```python
def optimize_segmentation_performance(seg_sensor_path, resolution_factor=1.0, enable_compression=True):
    """
    Optimize segmentation sensor settings for performance

    Args:
        seg_sensor_path: Path to segmentation sensor prim
        resolution_factor: Factor to reduce resolution (0.5 = half resolution)
        enable_compression: Whether to enable data compression
    """
    seg_prim = get_prim_at_path(seg_sensor_path)

    # Get current resolution
    current_width = seg_prim.GetAttribute("customData:resolution:width").Get()
    current_height = seg_prim.GetAttribute("customData:resolution:height").Get()

    if current_width and current_height:
        # Adjust resolution based on factor
        new_width = int(current_width * resolution_factor)
        new_height = int(current_height * resolution_factor)

        seg_prim.GetAttribute("customData:resolution:width").Set(new_width)
        seg_prim.GetAttribute("customData:resolution:height").Set(new_height)

        # Enable compression
        seg_prim.GetAttribute("performance:enable_compression").Set(enable_compression)

        print(f"Segmentation sensor optimized: {new_width}x{new_height}, compression={enable_compression}")
    else:
        print("Could not find resolution attributes to optimize")

# Optimize segmentation sensors based on requirements
optimize_segmentation_performance("/World/HumanoidRobot/head/seg_camera", resolution_factor=0.5, enable_compression=True)
```

## Data Quality and Validation

### Validating Segmentation Quality

```python
def validate_segmentation_quality(seg_sensor_path, expected_labels=None, min_coverage=0.1):
    """
    Validate segmentation quality by checking label distribution and coverage

    Args:
        seg_sensor_path: Path to segmentation sensor prim
        expected_labels: List of expected semantic labels in the scene
        min_coverage: Minimum coverage threshold for each expected label

    Returns:
        validation_results: Dictionary with validation statistics
    """
    print(f"Validating segmentation quality for {seg_sensor_path}")

    # Get segmentation data
    seg_sensor = CameraSensor(seg_sensor_path)
    seg_data = seg_sensor.get_segmentation_data()

    # Calculate label statistics
    unique_labels, counts = np.unique(seg_data, return_counts=True)
    total_pixels = seg_data.size
    label_stats = {}

    for label, count in zip(unique_labels, counts):
        coverage = count / total_pixels
        label_stats[int(label)] = {
            "count": int(count),
            "coverage": float(coverage),
            "percentage": float(coverage * 100)
        }

    # Check for expected labels
    if expected_labels:
        missing_labels = []
        low_coverage_labels = []

        for exp_label in expected_labels:
            if exp_label not in label_stats:
                missing_labels.append(exp_label)
            elif label_stats[exp_label]["coverage"] < min_coverage:
                low_coverage_labels.append((exp_label, label_stats[exp_label]["coverage"]))

        validation_results = {
            "label_stats": label_stats,
            "expected_labels_found": [label for label in expected_labels if label in label_stats],
            "missing_labels": missing_labels,
            "low_coverage_labels": low_coverage_labels,
            "total_pixels": total_pixels,
            "unique_labels_count": len(unique_labels)
        }

        print(f"Validation results:")
        print(f"  Expected labels found: {validation_results['expected_labels_found']}")
        print(f"  Missing labels: {missing_labels}")
        print(f"  Low coverage labels: {low_coverage_labels}")
        print(f"  Total unique labels: {len(unique_labels)}")
    else:
        validation_results = {
            "label_stats": label_stats,
            "total_pixels": total_pixels,
            "unique_labels_count": len(unique_labels)
        }

    return validation_results

# Validate segmentation quality
expected_semantic_ids = [0, 1, 2, 3, 4, 10]  # background, robot parts, floor
validation_results = validate_segmentation_quality(
    "/World/HumanoidRobot/head/seg_camera",
    expected_labels=expected_semantic_ids,
    min_coverage=0.01  # At least 1% coverage
)
```

## Troubleshooting Common Issues

### Segmentation Sensor Issues and Solutions

#### 1. All Background/Zero Labels
**Symptoms**: Segmentation image contains only background labels
**Solutions**:
- Check that objects have semantic attributes assigned
- Verify that objects are within camera view and range
- Ensure segmentation sensor is properly enabled

#### 2. Incorrect Labels
**Symptoms**: Objects have wrong semantic labels
**Solutions**:
- Verify semantic attributes are correctly assigned to objects
- Check that label IDs don't conflict
- Ensure proper USD prim hierarchy for labeling

#### 3. Performance Issues
**Symptoms**: Low frame rate or high GPU usage
**Solutions**:
- Reduce segmentation image resolution
- Use ID format instead of color format
- Simplify scene geometry if possible

#### 4. Missing Objects
**Symptoms**: Some objects don't appear in segmentation
**Solutions**:
- Check that objects have proper material assignments
- Verify objects are within clipping range
- Ensure objects are not marked as invisible

## Best Practices

### Segmentation Best Practices
1. **Label Consistency**: Use consistent labeling across all objects
2. **Hierarchy**: Organize labels in a meaningful hierarchy (e.g., "robot_arm_left", "robot_arm_right" both as "arm")
3. **Color Mapping**: Maintain consistent color mappings for visualization
4. **Validation**: Regularly validate segmentation quality and coverage
5. **Resolution**: Balance segmentation resolution with performance requirements

### Multi-Modal Integration
1. **Synchronization**: Ensure segmentation sensors are synchronized with RGB and depth sensors
2. **Calibration**: Maintain accurate calibration between different sensor modalities
3. **Data Association**: Properly associate segmentation with other sensor data
4. **Format Consistency**: Use consistent formats across all sensors

## Next Steps

After setting up segmentation sensors:
1. Learn about [Synthetic Dataset Generation](./synthetic-data-generation.md)
2. Explore [Isaac ROS Integration](../isaac-ros-perception.md)
3. Configure [Performance Optimization](./performance-optimization.md)
4. Test [Humanoid Model Loading](./humanoid-model-loading.md) with segmentation

## Additional Resources

- [Isaac Sim Segmentation Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_sensors.html)
- [Semantic Segmentation in Robotics](https://arxiv.org/abs/1908.06405)
- [ROS 2 Image Segmentation Conventions](https://wiki.ros.org/image_pipeline)
- [Synthetic Data for Deep Learning](https://research.nvidia.com/publication/2020-06_Synthetic-Datasets-Training)