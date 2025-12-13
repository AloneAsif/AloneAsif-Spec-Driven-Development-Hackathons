# RGB Camera Sensor Setup in Isaac Sim

## Overview

RGB cameras are fundamental sensors for robotics perception, providing color imagery that can be used for object detection, recognition, and scene understanding. In Isaac Sim, RGB cameras are implemented as physically accurate sensors that generate photorealistic images for synthetic data generation.

## Camera Sensor Fundamentals

### Camera Properties
- **Resolution**: Image dimensions (width × height in pixels)
- **Focal Length**: Determines field of view (typically 24mm equivalent)
- **Sensor Size**: Physical dimensions of the image sensor
- **Frame Rate**: Number of images captured per second (typically 30 FPS)
- **Format**: Color format (typically RGB8 or RGBA8)

### Camera Coordinate System
Isaac Sim uses a right-handed coordinate system:
- X: Right
- Y: Up
- Z: Forward (looking direction)

## Adding RGB Cameras to Humanoid Models

### Method 1: Adding Camera via Python API

```python
import omni
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import get_current_stage
from pxr import Gf, Sdf

def add_rgb_camera(prim_path, position, orientation, config=None):
    """
    Add an RGB camera to the scene

    Args:
        prim_path: Path for the camera prim (e.g., "/World/Camera")
        position: [x, y, z] position in world coordinates
        orientation: [x, y, z, w] quaternion orientation
        config: Dictionary with camera configuration
    """
    if config is None:
        config = {
            "resolution": {"width": 1920, "height": 1080},
            "focal_length": 24.0,
            "focus_distance": 10.0,
            "f_stop": 0.0,  # 0.0 = disabled, smaller numbers = more depth of field
            "horizontal_aperture": 36.0,
            "vertical_aperture": 20.25
        }

    # Create the camera prim
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

### Method 2: Adding Camera via USD Schema

```usda
#usda 1.0

def Camera "rgb_camera"
{
    float3 xformOp:translate = (0.0, 0.1, 0.0)
    uniform token[] xformOpOrder = ["xformOp:translate"]

    # Camera properties
    float focalLength = 24.0
    float focusDistance = 10.0
    float fStop = 0.0
    float horizontalAperture = 36.0
    float verticalAperture = 20.25
    float2 clippingRange = (0.1, 1000000.0)

    # Isaac Sim sensor properties
    bool sensor:is_primary = true
    string sensor:modality = "rgb"
}
```

## Configuring Camera Parameters

### Resolution Settings
```python
def set_camera_resolution(prim_path, width, height):
    """Set camera resolution"""
    camera_prim = get_prim_at_path(prim_path)

    # Set resolution via attributes
    camera_prim.GetAttribute("customData:resolution:width").Set(width)
    camera_prim.GetAttribute("customData:resolution:height").Set(height)

    print(f"Camera resolution set to {width}x{height}")

# Set different resolutions for different purposes
set_camera_resolution("/World/HumanoidRobot/head/rgb_camera", 1920, 1080)  # Full HD for main camera
set_camera_resolution("/World/HumanoidRobot/chest/rgb_camera", 640, 480)   # Lower res for secondary camera
```

### Field of View Configuration
```python
def configure_camera_fov(prim_path, fov_horizontal=60.0):
    """
    Configure camera field of view

    Args:
        prim_path: Path to camera prim
        fov_horizontal: Horizontal field of view in degrees
    """
    import math

    # Calculate focal length from field of view
    # Formula: focal_length = (sensor_width / 2) / tan(fov_horizontal / 2)
    sensor_width = 36.0  # Full frame sensor width in mm
    fov_rad = math.radians(fov_horizontal)
    focal_length = (sensor_width / 2) / math.tan(fov_rad / 2)

    camera_prim = get_prim_at_path(prim_path)
    camera_prim.GetAttribute("inputs:focalLength").Set(focal_length)

    print(f"Camera FOV set to {fov_horizontal}°, focal length: {focal_length}mm")

# Configure different FOVs for different applications
configure_camera_fov("/World/HumanoidRobot/head/rgb_camera", 60.0)   # Standard view
configure_camera_fov("/World/HumanoidRobot/shoulder/rgb_camera", 90.0)  # Wide angle
```

## Multiple Camera Configurations

### Stereo Camera Setup
```python
def setup_stereo_camera(base_path, baseline=0.2, fov=60.0):
    """
    Set up stereo RGB cameras for depth estimation

    Args:
        base_path: Base path for cameras (e.g., "/World/HumanoidRobot/stereo")
        baseline: Distance between left and right cameras in meters
        fov: Field of view for both cameras
    """
    # Calculate focal length from FOV
    import math
    sensor_width = 36.0
    fov_rad = math.radians(fov)
    focal_length = (sensor_width / 2) / math.tan(fov_rad / 2)

    # Left camera (offset to the left)
    create_prim(
        prim_path=f"{base_path}/left_camera",
        prim_type="Camera",
        position=[-baseline/2, 0, 0],
        attributes={
            "focalLength": focal_length,
            "horizontalAperture": sensor_width,
            "verticalAperture": sensor_width * 9/16,  # Assuming 16:9 aspect ratio
            "clippingRange": (0.1, 1000.0)
        }
    )

    # Right camera (offset to the right)
    create_prim(
        prim_path=f"{base_path}/right_camera",
        prim_type="Camera",
        position=[baseline/2, 0, 0],
        attributes={
            "focalLength": focal_length,
            "horizontalAperture": sensor_width,
            "verticalAperture": sensor_width * 9/16,
            "clippingRange": (0.1, 1000.0)
        }
    )

    print(f"Stereo camera setup complete with baseline: {baseline}m")

# Set up stereo cameras on humanoid
setup_stereo_camera("/World/HumanoidRobot/stereo", baseline=0.12)  # Human eye baseline
```

### 360-Degree Camera Setup
```python
def setup_360_camera(prim_path, position):
    """
    Set up multiple cameras to achieve 360-degree coverage
    """
    # Create 6 cameras facing different directions (cube map style)
    directions = [
        ("forward", [0, 0, 0, 1]),    # Looking +Z
        ("backward", [0, 1, 0, 0]),   # Looking -Z
        ("left", [0, 0.707, 0, 0.707]),  # Looking -X
        ("right", [0, 0.707, 0, -0.707]), # Looking +X
        ("up", [0.5, 0.5, 0.5, 0.5]),     # Looking +Y
        ("down", [0.5, -0.5, -0.5, 0.5])  # Looking -Y
    ]

    for direction_name, orientation in directions:
        create_prim(
            prim_path=f"{prim_path}/{direction_name}_cam",
            prim_type="Camera",
            position=position,
            orientation=orientation,
            attributes={
                "focalLength": 12.0,  # Wide angle for 360 coverage
                "horizontalAperture": 36.0,
                "verticalAperture": 36.0,
                "clippingRange": (0.1, 1000.0)
            }
        )

    print("360-degree camera setup complete")

# Set up 360 cameras on humanoid
setup_360_camera("/World/HumanoidRobot/surround_cam", [0, 0.5, 0])
```

## Isaac Sim ROS 2 Bridge Integration

### Publishing RGB Data to ROS 2
```python
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.ros_bridge.scripts import spawn_and_publish

def setup_rgb_ros_bridge(camera_path, topic_name, qos_profile="default"):
    """
    Set up ROS 2 bridge for RGB camera data

    Args:
        camera_path: Path to the RGB camera in USD stage
        topic_name: ROS 2 topic name (e.g., "/camera/rgb/image_raw")
        qos_profile: Quality of service profile
    """
    # Enable ROS bridge extension
    enable_extension("omni.isaac.ros2_bridge")

    # Configure camera for ROS publishing
    camera_prim = get_prim_at_path(camera_path)

    # Set ROS-specific attributes
    camera_prim.GetAttribute("ros2:topicName").Set(topic_name)
    camera_prim.GetAttribute("ros2:modality").Set("rgb")
    camera_prim.GetAttribute("ros2:enabled").Set(True)

    # Configure QoS settings
    qos_settings = {
        "reliability": "best_effort" if "depth" in topic_name else "reliable",
        "durability": "volatile",
        "history": "keep_last",
        "depth": 1 if "sensor" in topic_name else 10
    }

    for attr, value in qos_settings.items():
        camera_prim.GetAttribute(f"ros2:qos:{attr}").Set(value)

    print(f"RGB camera {camera_path} configured for ROS 2 publishing on {topic_name}")

# Configure RGB cameras for ROS 2
setup_rgb_ros_bridge("/World/HumanoidRobot/head/rgb_camera", "/camera/rgb/image_raw")
setup_rgb_ros_bridge("/World/HumanoidRobot/stereo/left_camera", "/camera/left/image_raw")
setup_rgb_ros_bridge("/World/HumanoidRobot/stereo/right_camera", "/camera/right/image_raw")
```

## Performance Optimization

### Camera Performance Settings
```python
def optimize_camera_performance(camera_path, enable_compression=True, max_fps=30):
    """
    Optimize camera settings for performance

    Args:
        camera_path: Path to camera prim
        enable_compression: Whether to enable image compression
        max_fps: Maximum frames per second
    """
    camera_prim = get_prim_at_path(camera_path)

    # Set performance-related attributes
    camera_prim.GetAttribute("performance:enable_compression").Set(enable_compression)
    camera_prim.GetAttribute("performance:max_fps").Set(max_fps)

    # Reduce resolution if performance is critical
    if max_fps > 60:
        # Use lower resolution for higher frame rates
        camera_prim.GetAttribute("customData:resolution:width").Set(1280)
        camera_prim.GetAttribute("customData:resolution:height").Set(720)
    else:
        # Use higher resolution for better quality
        camera_prim.GetAttribute("customData:resolution:width").Set(1920)
        camera_prim.GetAttribute("customData:resolution:height").Set(1080)

    print(f"Camera {camera_path} optimized for {max_fps} FPS")

# Optimize cameras based on use case
optimize_camera_performance("/World/HumanoidRobot/head/rgb_camera", max_fps=30)  # Standard
optimize_camera_performance("/World/HumanoidRobot/auxiliary_cam", max_fps=60)   # High-speed
```

### Multi-Camera Synchronization
```python
def synchronize_cameras(camera_paths, sync_interval=1.0/30.0):
    """
    Synchronize multiple cameras for consistent timing

    Args:
        camera_paths: List of camera prim paths
        sync_interval: Time interval between captures in seconds
    """
    for i, camera_path in enumerate(camera_paths):
        camera_prim = get_prim_at_path(camera_path)

        # Set synchronization attributes
        camera_prim.GetAttribute("sync:enabled").Set(True)
        camera_prim.GetAttribute("sync:interval").Set(sync_interval)
        camera_prim.GetAttribute("sync:offset").Set(i * 0.001)  # Small offset to avoid conflicts

    print(f"Synchronized {len(camera_paths)} cameras with {sync_interval}s interval")

# Synchronize stereo cameras
stereo_cameras = [
    "/World/HumanoidRobot/stereo/left_camera",
    "/World/HumanoidRobot/stereo/right_camera"
]
synchronize_cameras(stereo_cameras, sync_interval=1.0/30.0)
```

## Data Capture and Storage

### Capturing RGB Images
```python
import cv2
import numpy as np
from omni.synthetic_utils.sensors import CameraSensor
from PIL import Image

def capture_rgb_image(camera_path, output_path, format="png"):
    """
    Capture and save RGB image from Isaac Sim camera

    Args:
        camera_path: Path to camera prim
        output_path: Path to save the image
        format: Image format (png, jpg, etc.)
    """
    # Get camera sensor data
    camera_sensor = CameraSensor(camera_path)
    rgb_data = camera_sensor.get_rgb_data()

    # Convert to image format
    if format.lower() in ['jpg', 'jpeg']:
        # Convert RGB to BGR for OpenCV
        image_bgr = cv2.cvtColor(rgb_data, cv2.COLOR_RGB2BGR)
        cv2.imwrite(output_path, image_bgr)
    else:
        # Use PIL for other formats
        image = Image.fromarray(rgb_data)
        image.save(output_path, format=format.upper())

    print(f"RGB image saved to {output_path}")

def capture_continuous_rgb(camera_path, output_dir, num_images=100, interval=0.1):
    """
    Capture continuous RGB images at specified interval

    Args:
        camera_path: Path to camera prim
        output_dir: Directory to save images
        num_images: Number of images to capture
        interval: Time interval between captures in seconds
    """
    import time
    import os

    os.makedirs(output_dir, exist_ok=True)

    for i in range(num_images):
        timestamp = int(time.time() * 1000)  # Millisecond timestamp
        output_path = f"{output_dir}/rgb_{timestamp:0>13}.png"
        capture_rgb_image(camera_path, output_path)

        # Wait for specified interval
        time.sleep(interval)

        if i % 10 == 0:
            print(f"Captured {i+1}/{num_images} images...")

    print(f"Continuous RGB capture complete: {num_images} images saved to {output_dir}")

# Example usage
# capture_continuous_rgb("/World/HumanoidRobot/head/rgb_camera", "./rgb_dataset", num_images=1000, interval=0.1)
```

## Troubleshooting Common Issues

### Camera Not Publishing
**Symptoms**: Camera exists but no images on ROS topic
**Solutions**:
1. Verify ROS bridge extension is enabled
2. Check that camera has proper ROS attributes set
3. Confirm Isaac Sim is connected to ROS network
4. Validate topic names and permissions

### Low Frame Rate
**Symptoms**: Camera running below expected FPS
**Solutions**:
1. Reduce resolution
2. Disable compression if it's causing overhead
3. Check GPU performance and memory usage
4. Reduce scene complexity

### Distorted Images
**Symptoms**: Images appear stretched or have fisheye effect
**Solutions**:
1. Verify focal length and aperture settings
2. Check camera mounting position and orientation
3. Validate sensor size parameters

## Best Practices

### Camera Placement
1. **Head-mounted**: For egocentric vision tasks
2. **Stereo setup**: For depth estimation and 3D reconstruction
3. **Multiple viewpoints**: For comprehensive scene coverage
4. **Proper mounting**: Ensure cameras are securely attached to avoid vibration

### Resolution Guidelines
- **Object detection**: 640x480 minimum, 1280x720 recommended
- **Fine detail recognition**: 1920x1080 or higher
- **Real-time applications**: Balance quality with performance requirements

### Data Quality
1. **Consistent lighting**: Ensure adequate lighting for all images
2. **Proper exposure**: Avoid over/under-exposed images
3. **Stable mounting**: Minimize camera shake and vibration
4. **Calibration targets**: Include calibration patterns when needed

## Next Steps

After setting up RGB cameras:
1. Configure [Depth Sensors](./depth-sensor-config.md)
2. Set up [Semantic Segmentation Sensors](./segmentation-sensor-setup.md)
3. Learn about [Synthetic Dataset Generation](./synthetic-data-generation.md)
4. Explore [Isaac ROS Integration](../isaac-ros-perception.md)

## Additional Resources

- [Isaac Sim Camera Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_sensors.html)
- [ROS 2 Bridge Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_ros.html)
- [Synthetic Data Generation Best Practices](https://nvidia-isaac-ros.github.io/concepts/camera_models/index.html)