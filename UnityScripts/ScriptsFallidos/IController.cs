using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RobotPlugins
{
    public interface IController
    {
        public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };
        public enum ControlType { PositionControl };
        float stiffness { get; set; }
        float damping { get; set; }
        float forceLimit { get; set; }
        float speed { get; set; } // Units: degree/s
        float torque { get; set; } // Units: Nm or N
        float acceleration { get; set; }// Units: m/s^2 / degree/s^2
        void UpdateControlType(JointControlPlugin joint);
    }
}