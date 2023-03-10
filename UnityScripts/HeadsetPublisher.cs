using ROS2;
using UnityEngine;
using UnityEngine.XR;

[RequireComponent(typeof(InputOculusDevices))]
public class HeadsetPublisher : MonoBehaviour
{
    public string topic = "camera_position";
    private InputOculusDevices headset_;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<geometry_msgs.msg.Pose> cam_pose_pub;

    private void Awake()
    {
        headset_ = GetComponent<InputOculusDevices>();
        ros2Unity = GetComponent<ROS2UnityComponent>();

        ros2Node = ros2Unity.CreateNode("ROS2UnityHeadsetNode");
        cam_pose_pub = ros2Node.CreatePublisher<geometry_msgs.msg.Pose>(topic);
    }

    private void SendState()
    {
        geometry_msgs.msg.Pose msg = new geometry_msgs.msg.Pose();

        if (headset_.headset_.TryGetFeatureValue(CommonUsages.centerEyeRotation, out Quaternion headset_rot))
        {
            msg.Position.X = -transform.transform.position.z;
            msg.Position.Y = transform.transform.position.x;
            msg.Position.Z = transform.transform.position.y; 
            msg.Orientation.X = headset_rot.y;
            msg.Orientation.Y = -headset_rot.x;
            msg.Orientation.Z = -headset_rot.z;
            msg.Orientation.W = headset_rot.w;
            
            cam_pose_pub.Publish(msg);
            Debug.Log("Headset pose: (" + transform.transform.position.x + ", " + transform.transform.position.y + ", " + transform.transform.position.z);
            Debug.Log("Headset orientation: (" + headset_rot.x + ", " + headset_rot.y + ", " + headset_rot.z);
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
