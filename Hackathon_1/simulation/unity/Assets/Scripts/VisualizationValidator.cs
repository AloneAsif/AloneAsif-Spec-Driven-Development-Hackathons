using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class VisualizationValidator : MonoBehaviour
{
    ROSConnection ros;

    // Robot parts to validate
    public Transform[] robotParts;
    public Transform[] jointTransforms;

    // Validation parameters
    private float lastJointStateTime = 0f;
    private float jointStateTimeout = 2.0f; // seconds
    private bool jointStatesReceived = false;

    // Performance tracking
    private int frameCount = 0;
    private float lastFpsUpdate = 0f;
    private float fpsUpdateInterval = 1.0f;
    private float currentFPS = 0f;

    // Validation results
    private bool isSynchronized = false;
    private bool performanceOK = true;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to joint states to validate synchronization
        ros.Subscribe<JointStateMsg>("/joint_states", ValidateJointStates);

        Debug.Log("Visualization Validator initialized");
    }

    // Update is called once per frame
    void Update()
    {
        // Update FPS
        UpdateFPS();

        // Validate robot parts are present
        ValidateRobotParts();

        // Check synchronization status
        CheckSynchronization();

        // Log validation status periodically
        if (Time.time % 5.0f < Time.deltaTime) // Log every 5 seconds
        {
            LogValidationStatus();
        }
    }

    void ValidateJointStates(JointStateMsg jointState)
    {
        lastJointStateTime = Time.time;
        jointStatesReceived = true;

        // Validate joint state message
        if (jointState.name.Length > 0 && jointState.position.Length > 0)
        {
            // Check if we have expected joints
            List<string> expectedJoints = new List<string> {
                "left_arm_joint", "right_arm_joint", "left_leg_joint", "right_leg_joint"
            };

            bool allJointsFound = true;
            foreach (string expectedJoint in expectedJoints)
            {
                if (System.Array.IndexOf(jointState.name, expectedJoint) == -1)
                {
                    allJointsFound = false;
                    Debug.LogWarning($"Missing expected joint: {expectedJoint}");
                }
            }

            if (allJointsFound)
            {
                Debug.Log("✓ All expected joints found in joint state message");
            }
        }
    }

    void ValidateRobotParts()
    {
        // Check that all robot parts are present
        if (robotParts.Length == 0)
        {
            Debug.LogWarning("No robot parts defined for validation");
            return;
        }

        foreach (Transform part in robotParts)
        {
            if (part == null)
            {
                Debug.LogError("Missing robot part in visualization");
                return;
            }
        }

        Debug.Log("✓ All robot parts present in visualization");
    }

    void UpdateFPS()
    {
        frameCount++;
        if (Time.time - lastFpsUpdate >= fpsUpdateInterval)
        {
            currentFPS = frameCount / (Time.time - lastFpsUpdate);
            frameCount = 0;
            lastFpsUpdate = Time.time;

            // Check if performance is adequate (>30 FPS for real-time visualization)
            performanceOK = currentFPS >= 30.0f;
            if (!performanceOK)
            {
                Debug.LogWarning($"Performance warning: {currentFPS:F1} FPS (target: 30+ FPS)");
            }
            else
            {
                Debug.Log($"Performance: {currentFPS:F1} FPS");
            }
        }
    }

    void CheckSynchronization()
    {
        // Check if joint states are being received in a timely manner
        float timeSinceLastJointState = Time.time - lastJointStateTime;
        isSynchronized = jointStatesReceived && timeSinceLastJointState < jointStateTimeout;

        if (!isSynchronized && jointStatesReceived)
        {
            Debug.LogError($"Joint state timeout: last message was {timeSinceLastJointState:F1}s ago");
        }
    }

    void LogValidationStatus()
    {
        Debug.Log($"=== Visualization Validation Status ===");
        Debug.Log($"Synchronization: {(isSynchronized ? "OK" : "ISSUE")}");
        Debug.Log($"Performance: {(performanceOK ? "OK" : "ISSUE")} ({currentFPS:F1} FPS)");
        Debug.Log($"Joint States: {(jointStatesReceived ? "RECEIVING" : "NOT RECEIVING")}");
        Debug.Log($"=====================================");
    }

    // Public method to check overall validation status
    public bool IsVisualizationValid()
    {
        return isSynchronized && performanceOK && jointStatesReceived;
    }
}