using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class CommandProcessor : MonoBehaviour
{
    ROSConnection ros;

    // Robot movement parameters
    public float linearSpeed = 1.0f;
    public float angularSpeed = 1.0f;

    // Robot components
    public Transform robotBase;
    private Vector3 robotPosition;
    private Quaternion robotRotation;

    // Command topic
    private string cmdVelTopic = "/cmd_vel";

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Initialize robot state
        robotPosition = robotBase.position;
        robotRotation = robotBase.rotation;

        Debug.Log("Command Processor initialized");
    }

    // Update is called once per frame
    void Update()
    {
        // Process user input for manual control (for testing)
        ProcessUserInput();
    }

    void ProcessUserInput()
    {
        // For testing, allow keyboard control
        float linear = 0f;
        float angular = 0f;

        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
            linear = linearSpeed;
        else if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow))
            linear = -linearSpeed;

        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
            angular = angularSpeed;
        else if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
            angular = -angularSpeed;

        // Publish command if there's input
        if (linear != 0 || angular != 0)
        {
            PublishCommand(linear, 0, 0, 0, 0, angular);
        }
    }

    public void PublishCommand(float linearX, float linearY, float linearZ,
                              float angularX, float angularY, float angularZ)
    {
        var cmd = new TwistMsg();
        cmd.linear = new Vector3Msg(linearX, linearY, linearZ);
        cmd.angular = new Vector3Msg(angularX, angularY, angularZ);

        ros.Publish(cmdVelTopic, cmd);

        // Update robot position locally for immediate feedback
        robotPosition += robotBase.forward * linearX * Time.deltaTime;
        robotRotation *= Quaternion.Euler(0, angularZ * Mathf.Rad2Deg * Time.deltaTime, 0);

        robotBase.position = robotPosition;
        robotBase.rotation = robotRotation;
    }

    // Method to receive commands from ROS (would be called by ROS callback in real implementation)
    public void ProcessROSCommand(TwistMsg cmd)
    {
        float linearX = (float)cmd.linear.x;
        float angularZ = (float)cmd.angular.z;

        // Apply the command to the robot
        robotPosition += robotBase.forward * linearX * Time.deltaTime;
        robotRotation *= Quaternion.Euler(0, angularZ * Mathf.Rad2Deg * Time.deltaTime, 0);

        robotBase.position = robotPosition;
        robotBase.rotation = robotRotation;
    }
}