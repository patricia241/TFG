using ROS2;
using UnityEngine;
using UnityEngine.UI;

public class CamScript : MonoBehaviour
{
    Texture2D texRos;
    public RawImage display_lft;
    public RawImage display_rght;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<sensor_msgs.msg.Image> image_sub;

    private byte[] image_data;
    private bool new_img = false;

    void Awake()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        texRos = new Texture2D(320, 240, TextureFormat.RGB24, false);
    }

    void Update()
    {
        if (ros2Unity.Ok() && ros2Node == null)
        {
            ros2Node = ros2Unity.CreateNode("ROS2UnityImg");
            QualityOfServiceProfile qos = new QualityOfServiceProfile(QosPresetProfile.SENSOR_DATA);

            image_sub = ros2Node.CreateSubscription<sensor_msgs.msg.Image>(
              "/intel_realsense_r200_depth/image_raw", ImageCallback, qos);
        }

        if (new_img)
        {
            texRos.LoadRawTextureData(image_data);

            texRos.Apply();
            display_lft.texture = texRos;
            display_rght.texture = texRos;
            // Graphics.CopyTexture(texRos, layer);

            new_img = false;
        }
    }
    void ImageCallback(sensor_msgs.msg.Image img)
    {
        image_data = img.Data;
        new_img = true;
    }

}
