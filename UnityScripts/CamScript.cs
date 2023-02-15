using ROS2;
using UnityEngine;
using UnityEngine.UI;


public class CamScript : MonoBehaviour
{
    Texture2D texRos;
    public RawImage display;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<sensor_msgs.msg.Image> image_sub;

    private byte[] image_data;
    private bool new_img = false;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        texRos = new Texture2D(1920, 1080, TextureFormat.RGB24, false);

        ros2Node = ros2Unity.CreateNode("ROS2UnityCameraNode");
        QualityOfServiceProfile qos = new QualityOfServiceProfile(QosPresetProfile.SENSOR_DATA);

        image_sub = ros2Node.CreateSubscription<sensor_msgs.msg.Image>(
          "/global_camera/image_raw", ImageCallback, qos);
    }

    void ImageCallback(sensor_msgs.msg.Image img)
    {
        image_data = img.Data;
        new_img = true;
    }
    void Update()
    {
        if (new_img)
        {
            texRos.LoadRawTextureData(image_data);

            texRos.Apply();
            display.texture = texRos;

            new_img = false;
        }
    }
   
}
