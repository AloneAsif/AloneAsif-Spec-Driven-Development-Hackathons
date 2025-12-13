# Depth Sensor Configuration in Isaac Sim

## Overview

Depth sensors in Isaac Sim provide crucial 3D spatial information for robotics applications. Unlike traditional depth cameras that provide single-channel depth maps, Isaac Sim's depth sensors offer physically accurate depth measurements that can be used for 3D reconstruction, obstacle detection, and scene understanding.

## Depth Sensor Types

Isaac Sim provides several types of depth measurements:

- **Linear Depth**: Direct distance from camera to object
- **LIDAR Depth**: Simulated LIDAR point cloud data
- **Disparity Depth**: Depth information from stereo cameras
- **Point Cloud**: 3D point cloud data from depth information

## Adding Depth Sensors to Humanoid Models

### Method 1: Creating Depth Sensors via Python API

```python
import omni
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import get_current_stage
from pxr import Gf, Sdf

def add_depth_sensor(prim_path, position, orientation, config=None):
    """
    Add a depth sensor to the scene

    Args:
        prim_path: Path for the depth sensor prim
        position: [x, y, z] position in world coordinates
        orientation: [x, y, z, w] quaternion orientation
        config: Dictionary with depth sensor configuration
    """
    if config is None:
        config = {
            "resolution": {"width": 640, "height": 480},
            "focal_length": 24.0,
            "min_depth": 0.1,    # Minimum measurable distance in meters
            "max_depth": 10.0,   # Maximum measurable distance in meters
            "format": "R16f"     # 16-bit float format
        }

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
    depth_prim = get_prim_at_path(prim_path)
    depth_prim.GetAttribute("depth:enabled").Set(True)
    depth_prim.GetAttribute("depth:min_range").Set(config["min_depth"])
    depth_prim.GetAttribute("depth:max_range").Set(config["max_depth"])
    depth_prim.GetAttribute("depth:format").Set(config["format"])

    return depth_sensor

# Example: Add depth sensor to humanoid head
depth_camera = add_depth_sensor(
    prim_path="/World/HumanoidRobot/head/depth_sensor",
    position=[0, 0.1, 0],  # Slightly above the head
    orientation=[0.707, 0, 0.707, 0],  # Looking forward
    config={
        "resolution": {"width": 640, "height": 480},
        "focal_length": 24.0,
        "min_depth": 0.1,
        "max_depth": 10.0
    }
)
```

### Method 2: Creating Depth Sensors via USD Schema

```usda
#usda 1.0

def Camera "depth_sensor"
{
    float3 xformOp:translate = (0.0, 0.1, 0.0)
    uniform token[] xformOpOrder = ["xformOp:translate"]

    # Camera properties for depth sensing
    float focalLength = 24.0
    float horizontalAperture = 36.0
    float verticalAperture = 20.25
    float2 clippingRange = (0.1, 10.0)

    # Depth sensor properties
    bool depth:enabled = true
    float depth:min_range = 0.1
    float depth:max_range = 10.0
    string depth:format = "R16f"
    string sensor:modality = "depth"
}
```

## Configuring Depth Sensor Parameters

### Depth Range Configuration
```python
def configure_depth_range(prim_path, min_range=0.1, max_range=10.0):
    """
    Configure the depth range of a depth sensor

    Args:
        prim_path: Path to depth sensor prim
        min_range: Minimum measurable distance in meters
        max_range: Maximum measurable distance in meters
    """
    depth_prim = get_prim_at_path(prim_path)

    # Set depth range attributes
    depth_prim.GetAttribute("depth:min_range").Set(min_range)
    depth_prim.GetAttribute("depth:max_range").Set(max_range)
    depth_prim.GetAttribute("clippingRange").Set((min_range, max_range))

    print(f"Depth sensor range set to {min_range}m - {max_range}m")

# Configure different ranges for different applications
configure_depth_range("/World/HumanoidRobot/head/depth_sensor", min_range=0.1, max_range=10.0)  # Indoor
configure_depth_range("/World/HumanoidRobot/outdoor_depth", min_range=0.5, max_range=50.0)     # Outdoor
```

### Depth Resolution and Accuracy
```python
def configure_depth_resolution(prim_path, resolution_width=640, resolution_height=480, precision="R16f"):
    """
    Configure depth sensor resolution and precision

    Args:
        prim_path: Path to depth sensor prim
        resolution_width: Width of depth image in pixels
        resolution_height: Height of depth image in pixels
        precision: Depth data precision (R8, R16, R16f, R32f)
    """
    depth_prim = get_prim_at_path(prim_path)

    # Set resolution
    depth_prim.GetAttribute("customData:resolution:width").Set(resolution_width)
    depth_prim.GetAttribute("customData:resolution:height").Set(resolution_height)

    # Set precision
    depth_prim.GetAttribute("depth:format").Set(precision)

    # Calculate expected file size for data management
    if precision == "R16f":
        bytes_per_pixel = 2  # 16-bit float
    elif precision == "R32f":
        bytes_per_pixel = 4  # 32-bit float
    else:
        bytes_per_pixel = 1  # 8-bit

    total_size_mb = (resolution_width * resolution_height * bytes_per_pixel) / (1024 * 1024)
    print(f"Depth sensor configured: {resolution_width}x{resolution_height}, {precision}, ~{total_size_mb:.2f}MB per frame")

# Configure different resolutions for different needs
configure_depth_resolution("/World/HumanoidRobot/head/depth_sensor", 640, 480, "R16f")   # Standard
configure_depth_resolution("/World/HumanoidRobot/high_accuracy_depth", 1280, 720, "R32f") # High precision
```

## LIDAR Depth Simulation

### Creating LIDAR Sensors
```python
def add_lidar_sensor(prim_path, position, orientation, config=None):
    """
    Add a LIDAR depth sensor (simulated)

    Args:
        prim_path: Path for the LIDAR sensor prim
        position: [x, y, z] position in world coordinates
        orientation: [x, y, z, w] quaternion orientation
        config: Dictionary with LIDAR configuration
    """
    if config is None:
        config = {
            "rotation_frequency": 10,      # Hz
            "samples_per_rotation": 1440,  # Horizontal resolution
            "vertical_samples": 64,        # Vertical resolution (for 3D LIDAR)
            "min_range": 0.1,
            "max_range": 100.0,
            "angular_resolution": 0.25    # Degrees
        }

    # Create LIDAR sensor as a special type of camera
    lidar_sensor = create_prim(
        prim_path=prim_path,
        prim_type="RotatingLidarSensor",  # Isaac Sim specific type
        position=position,
        orientation=orientation,
        attributes={
            "rotationFrequency": config["rotation_frequency"],
            "samplesPerRotation": config["samples_per_rotation"],
            "verticalSamples": config["vertical_samples"],
            "minRange": config["min_range"],
            "maxRange": config["max_range"],
            "angularResolution": config["angular_resolution"]
        }
    )

    return lidar_sensor

# Example: Add LIDAR to humanoid torso
lidar_sensor = add_lidar_sensor(
    prim_path="/World/HumanoidRobot/torso/lidar",
    position=[0, 0.5, 0],  # On the chest area
    orientation=[0, 0, 0, 1],  # Looking forward
    config={
        "rotation_frequency": 10,
        "samples_per_rotation": 1440,
        "vertical_samples": 64,
        "min_range": 0.1,
        "max_range": 50.0
    }
)
```

## Stereo Depth Configuration

### Setting Up Stereo Depth from RGB Cameras
```python
def setup_stereo_depth(left_camera_path, right_camera_path, baseline=0.2):
    """
    Configure stereo depth calculation from stereo RGB cameras

    Args:
        left_camera_path: Path to left RGB camera
        right_camera_path: Path to right RGB camera
        baseline: Distance between cameras in meters
    """
    # Configure both cameras for stereo operation
    left_cam = get_prim_at_path(left_camera_path)
    right_cam = get_prim_at_path(right_camera_path)

    # Set stereo-specific attributes
    left_cam.GetAttribute("stereo:is_left").Set(True)
    left_cam.GetAttribute("stereo:baseline").Set(baseline)
    left_cam.GetAttribute("stereo:enabled").Set(True)

    right_cam.GetAttribute("stereo:is_left").Set(False)
    right_cam.GetAttribute("stereo:baseline").Set(baseline)
    right_cam.GetAttribute("stereo:enabled").Set(True)

    # Enable disparity map generation
    left_cam.GetAttribute("disparity:enabled").Set(True)
    right_cam.GetAttribute("disparity:enabled").Set(True)

    print(f"Stereo depth configured with baseline: {baseline}m")

# Set up stereo depth from previously created stereo cameras
setup_stereo_depth(
    "/World/HumanoidRobot/stereo/left_camera",
    "/World/HumanoidRobot/stereo/right_camera",
    baseline=0.12  # Human eye baseline
)
```

## Isaac Sim ROS 2 Bridge Integration for Depth

### Publishing Depth Data to ROS 2
```python
def setup_depth_ros_bridge(depth_sensor_path, topic_name, qos_profile="default"):
    """
    Set up ROS 2 bridge for depth sensor data

    Args:
        depth_sensor_path: Path to the depth sensor in USD stage
        topic_name: ROS 2 topic name (e.g., "/camera/depth/image_raw")
        qos_profile: Quality of service profile
    """
    # Enable ROS bridge extension
    enable_extension("omni.isaac.ros2_bridge")

    # Configure depth sensor for ROS publishing
    depth_prim = get_prim_at_path(depth_sensor_path)

    # Set ROS-specific attributes for depth
    depth_prim.GetAttribute("ros2:topicName").Set(topic_name)
    depth_prim.GetAttribute("ros2:modality").Set("depth")
    depth_prim.GetAttribute("ros2:enabled").Set(True)

    # Configure depth-specific ROS attributes
    depth_prim.GetAttribute("ros2:depth:unit").Set("meters")  # or "millimeters"
    depth_prim.GetAttribute("ros2:depth:encoding").Set("32FC1")  # 32-bit float

    # Configure QoS settings appropriate for depth data
    qos_settings = {
        "reliability": "best_effort",  # Depth data can tolerate some loss
        "durability": "volatile",
        "history": "keep_last",
        "depth": 1  # Only keep latest depth image
    }

    for attr, value in qos_settings.items():
        depth_prim.GetAttribute(f"ros2:qos:{attr}").Set(value)

    print(f"Depth sensor {depth_sensor_path} configured for ROS 2 publishing on {topic_name}")

# Configure depth sensors for ROS 2
setup_depth_ros_bridge("/World/HumanoidRobot/head/depth_sensor", "/camera/depth/image_raw")
setup_depth_ros_bridge("/World/HumanoidRobot/stereo/left_camera", "/camera/depth/disparity")  # For stereo depth
```

## Point Cloud Generation from Depth

### Converting Depth to Point Cloud
```python
import numpy as np
from scipy.spatial.transform import Rotation as R

def depth_to_pointcloud(depth_image, camera_intrinsics, position=[0,0,0], orientation=[0,0,0,1]):
    """
    Convert depth image to 3D point cloud

    Args:
        depth_image: 2D depth image array
        camera_intrinsics: Camera intrinsic matrix
        position: Camera position in world coordinates
        orientation: Camera orientation as quaternion [x,y,z,w]

    Returns:
        pointcloud: Nx3 array of 3D points
    """
    height, width = depth_image.shape
    fx, fy = camera_intrinsics[0, 0], camera_intrinsics[1, 1]
    cx, cy = camera_intrinsics[0, 2], camera_intrinsics[1, 2]

    # Generate pixel coordinates
    x_coords, y_coords = np.meshgrid(np.arange(width), np.arange(height))

    # Convert to camera coordinates
    x_cam = (x_coords - cx) * depth_image / fx
    y_cam = (y_coords - cy) * depth_image / fy
    z_cam = depth_image

    # Stack to form point cloud in camera frame
    points_cam = np.stack([x_cam, y_cam, z_cam], axis=-1).reshape(-1, 3)

    # Remove invalid points (where depth is 0 or invalid)
    valid_mask = (z_cam.flatten() > 0) & (np.isfinite(points_cam).all(axis=1))
    points_cam = points_cam[valid_mask]

    # Transform to world coordinates if needed
    if not (position == [0,0,0] and orientation == [0,0,0,1]):
        # Convert quaternion to rotation matrix
        rot = R.from_quat(orientation).as_matrix()

        # Transform points
        points_world = (rot @ points_cam.T).T + np.array(position)
        return points_world
    else:
        return points_cam

def setup_pointcloud_pipeline(depth_sensor_path):
    """
    Set up pipeline to generate point clouds from depth data
    """
    # This would typically be implemented as a ROS 2 node
    # that subscribes to depth topic and camera info topic
    # then processes depth to point cloud
    print(f"Point cloud pipeline configured for {depth_sensor_path}")
    print("This would typically run as a ROS 2 node that:")
    print("1. Subscribes to depth image topic")
    print("2. Subscribes to camera info topic")
    print("3. Processes depth to point cloud")
    print("4. Publishes point cloud to /camera/depth/points topic")

# Example pipeline setup
setup_pointcloud_pipeline("/World/HumanoidRobot/head/depth_sensor")
```

## Performance Optimization for Depth Sensors

### Optimizing Depth Sensor Performance
```python
def optimize_depth_performance(depth_sensor_path, enable_compression=True, max_fps=30):
    """
    Optimize depth sensor settings for performance

    Args:
        depth_sensor_path: Path to depth sensor prim
        enable_compression: Whether to enable data compression
        max_fps: Maximum frames per second
    """
    depth_prim = get_prim_at_path(depth_sensor_path)

    # Set performance-related attributes
    depth_prim.GetAttribute("performance:enable_compression").Set(enable_compression)
    depth_prim.GetAttribute("performance:max_fps").Set(max_fps)

    # Adjust quality based on performance needs
    if max_fps > 30:
        # For high-speed applications, reduce quality
        depth_prim.GetAttribute("customData:resolution:width").Set(320)
        depth_prim.GetAttribute("customData:resolution:height").Set(240)
        depth_prim.GetAttribute("depth:format").Set("R16")  # Lower precision
    else:
        # For accuracy-focused applications
        depth_prim.GetAttribute("customData:resolution:width").Set(640)
        depth_prim.GetAttribute("customData:resolution:height").Set(480)
        depth_prim.GetAttribute("depth:format").Set("R16f")  # Higher precision

    # Enable temporal subsampling if needed
    if max_fps < 10:
        depth_prim.GetAttribute("performance:temporal_subsampling").Set(3)  # Process every 3rd frame
    else:
        depth_prim.GetAttribute("performance:temporal_subsampling").Set(1)

    print(f"Depth sensor {depth_sensor_path} optimized for {max_fps} FPS")

# Optimize depth sensors based on use case
optimize_depth_performance("/World/HumanoidRobot/head/depth_sensor", max_fps=30)   # Standard
optimize_depth_performance("/World/HumanoidRobot/fast_depth", max_fps=60)         # High-speed
```

## Data Capture and Storage

### Capturing Depth Data
```python
import cv2
import numpy as np
from omni.synthetic_utils.sensors import CameraSensor
from PIL import Image

def capture_depth_image(depth_sensor_path, output_path, format="png"):
    """
    Capture and save depth image from Isaac Sim depth sensor

    Args:
        depth_sensor_path: Path to depth sensor prim
        output_path: Path to save the depth image
        format: Image format (png, tiff, etc.)
    """
    # Get depth sensor data
    depth_sensor = CameraSensor(depth_sensor_path)
    depth_data = depth_sensor.get_depth_data()

    # Save in appropriate format
    if format.lower() == 'tiff':
        # Save as TIFF for higher precision
        from PIL import Image
        # Convert to 16-bit or 32-bit format depending on precision
        depth_16bit = (depth_data * 1000).astype(np.uint16)  # Convert to mm, scale by 1000
        depth_image = Image.fromarray(depth_16bit)
        depth_image.save(output_path, format='TIFF')
    elif format.lower() == 'png':
        # Save as PNG with 16-bit depth
        depth_16bit = (depth_data * 1000).astype(np.uint16)  # Scale to avoid precision loss
        cv2.imwrite(output_path, depth_16bit, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    else:
        # Save as raw binary for maximum precision
        np.save(output_path.replace('.raw', '.npy'), depth_data)

    print(f"Depth image saved to {output_path}")

def capture_continuous_depth(depth_sensor_path, output_dir, num_frames=100, interval=0.1):
    """
    Capture continuous depth data at specified interval

    Args:
        depth_sensor_path: Path to depth sensor prim
        output_dir: Directory to save depth data
        num_frames: Number of frames to capture
        interval: Time interval between captures in seconds
    """
    import time
    import os

    os.makedirs(output_dir, exist_ok=True)

    for i in range(num_frames):
        timestamp = int(time.time() * 1000)  # Millisecond timestamp
        output_path = f"{output_dir}/depth_{timestamp:0>13}.tiff"
        capture_depth_image(depth_sensor_path, output_path, format='tiff')

        # Wait for specified interval
        time.sleep(interval)

        if i % 10 == 0:
            print(f"Captured {i+1}/{num_frames} depth frames...")

    print(f"Continuous depth capture complete: {num_frames} frames saved to {output_dir}")

# Example usage
# capture_continuous_depth("/World/HumanoidRobot/head/depth_sensor", "./depth_dataset", num_frames=1000, interval=0.1)
```

## Calibration and Validation

### Depth Sensor Calibration
```python
def validate_depth_sensor(depth_sensor_path, test_distance=1.0, tolerance=0.05):
    """
    Validate depth sensor accuracy using known distances

    Args:
        depth_sensor_path: Path to depth sensor prim
        test_distance: Known distance to validate against (meters)
        tolerance: Acceptable error tolerance (meters)

    Returns:
        accuracy_stats: Dictionary with accuracy statistics
    """
    print(f"Validating depth sensor at known distance: {test_distance}m")

    # Get depth sensor data
    depth_sensor = CameraSensor(depth_sensor_path)
    depth_data = depth_sensor.get_depth_data()

    # Calculate statistics for the center region (assuming target is centered)
    h, w = depth_data.shape
    center_h, center_w = h // 2, w // 2
    center_region = depth_data[center_h-20:center_h+20, center_w-20:center_w+20]

    # Remove invalid values
    valid_depths = center_region[np.isfinite(center_region) & (center_region > 0)]

    if len(valid_depths) == 0:
        print("ERROR: No valid depth values found")
        return None

    mean_depth = np.mean(valid_depths)
    std_depth = np.std(valid_depths)
    min_depth = np.min(valid_depths)
    max_depth = np.max(valid_depths)

    error = abs(mean_depth - test_distance)
    accuracy_stats = {
        "mean_depth": mean_depth,
        "std_depth": std_depth,
        "min_depth": min_depth,
        "max_depth": max_depth,
        "error": error,
        "within_tolerance": error <= tolerance,
        "valid_pixels": len(valid_depths)
    }

    print(f"Depth validation results:")
    print(f"  Expected: {test_distance}m")
    print(f"  Measured: {mean_depth:.3f}m ± {std_depth:.3f}m")
    print(f"  Error: {error:.3f}m (tolerance: ±{tolerance}m)")
    print(f"  Accuracy: {'PASS' if error <= tolerance else 'FAIL'}")

    return accuracy_stats

# Validate depth sensor
validation_results = validate_depth_sensor("/World/HumanoidRobot/head/depth_sensor", test_distance=2.0, tolerance=0.05)
```

## Troubleshooting Common Issues

### Depth Sensor Issues and Solutions

#### 1. All Zeros or Invalid Values
**Symptoms**: Depth image contains only zeros or invalid values
**Solutions**:
- Check that objects are within the configured depth range
- Verify camera clipping planes are set correctly
- Ensure proper lighting for the scene

#### 2. Low Accuracy
**Symptoms**: Depth measurements are inaccurate
**Solutions**:
- Increase depth sensor resolution
- Check that depth format is appropriate (R16f vs R16)
- Verify proper camera calibration parameters

#### 3. Performance Issues
**Symptoms**: Low frame rate or high CPU/GPU usage
**Solutions**:
- Reduce resolution or frame rate
- Use lower precision depth format
- Simplify scene geometry

#### 4. Noise in Depth Data
**Symptoms**: Depth image contains noise or artifacts
**Solutions**:
- Apply temporal filtering
- Increase lighting in the scene
- Verify camera parameters are physically accurate

## Best Practices

### Depth Sensor Best Practices
- **Range Selection**: Choose depth range appropriate for your application
- **Resolution Balance**: Balance resolution with performance requirements
- **Validation**: Regularly validate depth accuracy with known objects
- **Compression**: Use appropriate compression for storage and transmission
- **Synchronization**: Ensure depth sensors are synchronized with RGB cameras for RGB-D applications

### Multi-Sensor Integration
- **RGB-D Alignment**: Ensure depth and RGB sensors are properly aligned
- **Temporal Sync**: Synchronize capture times for RGB and depth
- **Calibration**: Maintain updated calibration parameters for sensor pairs
- **Data Association**: Properly associate depth data with RGB pixels

## Next Steps

After configuring depth sensors:
- Set up [Semantic Segmentation Sensors](./segmentation-sensor-setup.md)
- Learn about [Synthetic Dataset Generation](./synthetic-data-generation.md)
- Explore [Isaac ROS Integration](../isaac-ros-perception.md)
- Configure [Performance Optimization](./performance-optimization.md)

## Additional Resources

- [Isaac Sim Depth Sensor Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_sensors.html)
- [ROS 2 Depth Image Conventions](https://wiki.ros.org/image_pipeline)
- [Depth Camera Calibration](https://github.com/IntelRealSense/librealsense/blob/master/doc/rs_camera_calibration.pdf)
- [Point Cloud Library (PCL) Integration](https://pointclouds.org/)