using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class ROSConnectionManager : MonoBehaviour
{
    ROSConnection ros;
    public string rosIP = "127.0.0.1";
    public int rosPort = 10000;

    // Robot joint transforms
    public Transform leftArmJoint;
    public Transform rightArmJoint;
    public Transform leftLegJoint;
    public Transform rightLegJoint;

    // Sensor data publishers
    public Camera unityCamera;
    public string cameraTopic = "/unity/camera/image_raw";
    private float cameraPublishRate = 30.0f; // Hz
    private float lastCameraPublishTime = 0.0f;

    // Command subscribers
    string cmdVelTopic = "/unity/cmd_vel";

    // Robot state
    Vector3 robotPosition = Vector3.zero;
    Quaternion robotRotation = Quaternion.identity;

    // Start is called before the first frame update
    void Start()
    {
        // Get the ROSConnection static instance, which will remain active between scenes
        ros = ROSConnection.GetOrCreateInstance();
        ros.Initialize(rosIP, rosPort);

        // Subscribe to command topics
        ros.Subscribe<TwistMsg>(cmdVelTopic, CmdVelCallback);

        Debug.Log("ROS Connection Manager initialized");
    }

    // Update is called once per frame
    void Update()
    {
        // Publish sensor data at appropriate rates
        PublishJointStates();
        PublishCameraData();
    }

    void CmdVelCallback(TwistMsg cmd)
    {
        // Process velocity commands from ROS
        float linearX = (float)cmd.linear.x;
        float angularZ = (float)cmd.angular.z;

        // Apply movement to robot (simplified differential drive)
        robotPosition += transform.forward * linearX * Time.deltaTime;
        transform.Rotate(Vector3.up, angularZ * Time.deltaTime);

        // Update position
        transform.position = robotPosition;
    }

    void PublishJointStates()
    {
        // Create joint state message
        var jointState = new JointStateMsg();
        jointState.header = new HeaderMsg();
        jointState.header.stamp = new TimeStamp(0, (uint)(Time.time * 1e9));
        jointState.header.frame_id = "base_link";

        // Joint names
        jointState.name = new string[] {
            "left_arm_joint",
            "right_arm_joint",
            "left_leg_joint",
            "right_leg_joint"
        };

        // Joint positions (get from transforms)
        jointState.position = new double[] {
            leftArmJoint.localEulerAngles.y,
            rightArmJoint.localEulerAngles.y,
            leftLegJoint.localEulerAngles.y,
            rightLegJoint.localEulerAngles.y
        };

        // Publish joint states
        ros.Publish("/unity/joint_states", jointState);
    }

    void PublishCameraData()
    {
        // Publish camera data at specified rate
        if (Time.time - lastCameraPublishTime >= 1.0f / cameraPublishRate)
        {
            // In a real implementation, capture and publish camera image data
            // For now, we'll just send a timestamp
            var imageMsg = new ImageMsg();
            imageMsg.header = new HeaderMsg();
            imageMsg.header.stamp = new TimeStamp(0, (uint)(Time.time * 1e9));
            imageMsg.header.frame_id = "camera_link";
            imageMsg.width = (uint)unityCamera.pixelWidth;
            imageMsg.height = (uint)unityCamera.pixelHeight;
            imageMsg.encoding = "rgb8";
            imageMsg.is_bigendian = 0;
            imageMsg.step = (uint)(3 * imageMsg.width); // 3 bytes per pixel
            imageMsg.data = new byte[imageMsg.step * imageMsg.height]; // Placeholder

            ros.Publish(cameraTopic, imageMsg);
            lastCameraPublishTime = Time.time;
        }
    }

    // Method to update joint positions from ROS data
    public void UpdateJointPositions(float leftArmPos, float rightArmPos, float leftLegPos, float rightLegPos)
    {
        if (leftArmJoint != null) leftArmJoint.localRotation = Quaternion.Euler(0, leftArmPos, 0);
        if (rightArmJoint != null) rightArmJoint.localRotation = Quaternion.Euler(0, rightArmPos, 0);
        if (leftLegJoint != null) leftLegJoint.localRotation = Quaternion.Euler(0, leftLegPos, 0);
        if (rightLegJoint != null) rightLegJoint.localRotation = Quaternion.Euler(0, rightLegPos, 0);
    }
}