using RobotPlugins;
using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics;
using UnityEngine;

namespace RobotPlugins
{
    public class DiffDrive : MonoBehaviour, IController
    {
        ArticulationBody[] wheel_joints;

        public IController.ControlType control = IController.ControlType.PositionControl;
        public float stiffness { get; set; }
        public float damping { get; set; }
        public float forceLimit { get; set; }
        public float speed { get; set; }  = 5f; // Units: degree/s
        public float torque { get; set; }  = 100f; // Units: Nm or N
        public float acceleration { get; set; } = 5f;// Units: m/s^2 / degree/s^2

        public List<string> wheel_names;
        void Start()
        {
            if (wheel_names.Count < 2)
            {
                Debug.LogError("Debes introducir los nombres de las ruedas del robot");
            }
            this.gameObject.AddComponent<Unity.Robotics.UrdfImporter.Control.FKRobot>();
            ArticulationBody[] articulationChain = this.GetComponentsInChildren<ArticulationBody>();
            wheel_joints = articulationChain.Where(p => wheel_names.Contains(p.name)).ToArray();

            int defDyanmicVal = 10;
            foreach (ArticulationBody joint in wheel_joints)
            {
                joint.gameObject.AddComponent<JointControlPlugin>();
                joint.jointFriction = defDyanmicVal;
                joint.angularDamping = defDyanmicVal;
                ArticulationDrive currentDrive = joint.xDrive;
                currentDrive.forceLimit = forceLimit;
                joint.xDrive = currentDrive;
            }
        }
 
        void Update()
        {
            UpdateDirection();
        }

        /// <summary>
        /// Sets the direction of movement of the joint on every update
        /// </summary>
        private void UpdateDirection()
        {
            float moveDirection = Input.GetAxis("Vertical");
            JointControlPlugin current = wheel_joints[0].GetComponent<JointControlPlugin>();

            if (current.controltype != control)
            {
                UpdateControlType(current);
            }

            if (moveDirection > 0)
            {
                current.direction = IController.RotationDirection.Positive;
            }
            else if (moveDirection < 0)
            {
                current.direction = IController.RotationDirection.Negative;
            }
            else
            {
                current.direction = IController.RotationDirection.None;
            }
        }
       
        public void UpdateControlType(JointControlPlugin joint)
        {
            joint.controltype = control;
            if (control == IController.ControlType.PositionControl)
            {
                ArticulationDrive drive = joint.joint.xDrive;
                drive.stiffness = stiffness;
                drive.damping = damping;
                joint.joint.xDrive = drive;
            }
        } 
    }

}