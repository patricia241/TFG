using UnityEngine;
using ROS2;
using Unity.Robotics.UrdfImporter.Control;
using static UnityEditor.PlayerSettings;
using System;
using Assimp.Configs;

namespace RosSharp.Control
{
    public enum ControlMode { Keyboard, ROS };

    public class AGVController : MonoBehaviour
    {
        public GameObject wheel1;
        public GameObject wheel2;
        public ControlMode mode = ControlMode.ROS;
        long lastCmdReceived = 0;
        float ROSTimeout = 0.5f;

        private ArticulationBody wA1;
        private ArticulationBody wA2;

        public float maxLinearSpeed = 2; //  m/s
        public float maxRotationalSpeed = 1;//
        public float wheelRadius = 0.033f; //meters
        public float trackWidth = 0.288f; // meters Distance between tyres
        public float forceLimit = 10;
        public float damping = 10;

        ROS2UnityComponent ros;
        private ROS2Node ros_node;
        private ISubscription<geometry_msgs.msg.Twist> vel_sub;
        private RotationDirection direction;
        private float rosLinear = 0f;
        private float rosAngular = 0f;

        void Start()
        {
            wA1 = wheel1.GetComponent<ArticulationBody>();
            wA2 = wheel2.GetComponent<ArticulationBody>();
            SetParameters(wA1);
            SetParameters(wA2);
            ros = GetComponent<ROS2UnityComponent>();
            ros_node = ros.CreateNode("AGVController");
            vel_sub = ros_node.CreateSubscription<geometry_msgs.msg.Twist>(
              "cmd_vel", ReceiveROSCmd);
        }

        void ReceiveROSCmd(geometry_msgs.msg.Twist cmdVel)
        {
            rosLinear = (float)cmdVel.Linear.X;
            rosAngular = (float)cmdVel.Angular.Z;
            lastCmdReceived = DateTime.Now.Ticks;
        }

        void FixedUpdate()
        {
            if (mode == ControlMode.Keyboard)
            {
                KeyBoardUpdate();
            }
            else if (mode == ControlMode.ROS)
            {
                ROSUpdate();
            }
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            // ArticulationDrive aplies forces and torques to the connected bodies
            ArticulationDrive drive = joint.xDrive;
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;
                Debug.Log("WheelSpeed Nan "+ drive.targetVelocity);
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
                Debug.Log("WheelSpeed " + drive.targetVelocity);
            }
            joint.xDrive = drive;
        }

        private void KeyBoardUpdate()
        {
            float moveDirection = Input.GetAxis("Vertical");
            float inputSpeed;
            float inputRotationSpeed;
            if (moveDirection > 0)
            {
                inputSpeed = maxLinearSpeed;
            }
            else if (moveDirection < 0)
            {
                inputSpeed = maxLinearSpeed * -1;
            }
            else
            {
                inputSpeed = 0;
            }

            float turnDirction = Input.GetAxis("Horizontal");
            if (turnDirction > 0)
            {
                inputRotationSpeed = maxRotationalSpeed;
            }
            else if (turnDirction < 0)
            {
                inputRotationSpeed = maxRotationalSpeed * -1;
            }
            else
            {
                inputRotationSpeed = 0;
            }
            RobotInput(inputSpeed, inputRotationSpeed);
        }


        private void ROSUpdate()
        {
            // Elapsed ticks since last ROS msg received
            long elapsedTicks = DateTime.Now.Ticks - lastCmdReceived;
            TimeSpan elapsedSpan = new TimeSpan(elapsedTicks);

            if (elapsedSpan.TotalSeconds > ROSTimeout)
            {
                rosLinear = 0f;
                rosAngular = 0f;
            }

            RobotInput(rosLinear, -rosAngular);
        }

        private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
        {
            if (speed > maxLinearSpeed)
            {
                speed = maxLinearSpeed;
            }
            if (rotSpeed > maxRotationalSpeed)
            {
                rotSpeed = maxRotationalSpeed;
            }
            float wheel1Rotation = (speed / wheelRadius);
            float wheel2Rotation = wheel1Rotation;
            float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);
            if (rotSpeed != 0)
            {
                wheel1Rotation = (wheel1Rotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                wheel2Rotation = (wheel2Rotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
            }
            else
            {
                wheel1Rotation *= Mathf.Rad2Deg;
                wheel2Rotation *= Mathf.Rad2Deg;
            }
            SetSpeed(wA1, wheel1Rotation);
            SetSpeed(wA2, wheel2Rotation);
        }
    }
}