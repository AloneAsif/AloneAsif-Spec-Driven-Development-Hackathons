using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class GazeboSync : MonoBehaviour
{
    ROSConnection ros;

    // Robot parts to synchronize
    public Transform baseLink;
    public Transform head;
    public Transform leftArm;
    public Transform rightArm;
    public Transform leftLeg;
    public Transform rightLeg;
    public Transform lidarLink;
    public Transform cameraLink;

    // Joint state subscription
    private string jointStatesTopic = "/joint_states";

    // Robot state variables
    private float leftArmJointPos = 0f;
    private float rightArmJointPos = 0f;
    private float leftLegJointPos = 0f;
    private float rightLegJointPos = 0f;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to joint states from Gazebo
        ros.Subscribe<JointStateMsg>(jointStatesTopic, JointStateCallback);

        Debug.Log("Gazebo Sync initialized");
    }

    // Update is called once per frame
    void Update()
    {
        // Update robot part positions based on joint states
        UpdateRobotPose();
    }

    void JointStateCallback(JointStateMsg jointState)
    {
        // Update joint positions based on received joint states
        for (int i = 0; i < jointState.name.Length; i++)
        {
            string jointName = jointState.name[i];
            float jointPos = (float)jointState.position[i];

            switch (jointName)
            {
                case "left_arm_joint":
                    leftArmJointPos = jointPos;
                    break;
                case "right_arm_joint":
                    rightArmJointPos = jointPos;
                    break;
                case "left_leg_joint":
                    leftLegJointPos = jointPos;
                    break;
                case "right_leg_joint":
                    rightLegJointPos = jointPos;
                    break;
            }
        }
    }

    void UpdateRobotPose()
    {
        // Update joint positions in Unity
        if (leftArm != null)
            leftArm.localRotation = Quaternion.Euler(0, leftArmJointPos * Mathf.Rad2Deg, 0);

        if (rightArm != null)
            rightArm.localRotation = Quaternion.Euler(0, rightArmJointPos * Mathf.Rad2Deg, 0);

        if (leftLeg != null)
            leftLeg.localRotation = Quaternion.Euler(0, 0, leftLegJointPos * Mathf.Rad2Deg);

        if (rightLeg != null)
            rightLeg.localRotation = Quaternion.Euler(0, 0, rightLegJointPos * Mathf.Rad2Deg);
    }
}