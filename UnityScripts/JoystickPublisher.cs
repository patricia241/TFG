using ROS2;
using UnityEngine;
using UnityEngine.XR;


[RequireComponent(typeof(InputOculusDevices))]
public class JoystickPublisher : MonoBehaviour
{
    private InputOculusDevices right_controller_;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<geometry_msgs.msg.Pose> cam_pose_pub;

    private void Start()
    {
        right_controller_ = GetComponent<InputOculusDevices>();
        ros2Unity = GetComponent<ROS2UnityComponent>();

        ros2Node = ros2Unity.CreateNode("ROS2UnityJoystickNode");
        cam_pose_pub = ros2Node.CreatePublisher<geometry_msgs.msg.Pose>("camera_position");
    }

    private void SendState()
    {
        geometry_msgs.msg.Pose msg = new geometry_msgs.msg.Pose();

        if (right_controller_.right_controller_.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 joystick_pose))
        {
            msg.Position.X = joystick_pose.x;
            msg.Position.Y = joystick_pose.y;
            msg.Position.Z = 0;

            cam_pose_pub.Publish(msg);
            Debug.Log("Joystick Value: " + joystick_pose);
        }
    }

    void Update()
    {
        if (ros2Unity.Ok())
        {
            SendState();
        }
    }
}
