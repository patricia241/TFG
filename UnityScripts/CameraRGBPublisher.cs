using UnityEngine;
using ROS2;

public class CameraRGBPublisher : Sensor<sensor_msgs.msg.Image>
{
    public Camera cam_;
    public int image_width = 320;
    public int image_height = 240;

    public string frameId = "base_link";
    sensor_msgs.msg.Image msg;
    RenderTexture render_texture_;
    byte[] data;
    Texture2D texture2d;
    bool updated_data = false;

    public void Awake()
    {
        desiredFrameTime = 25;
        render_texture_ = new RenderTexture(image_width, image_height, 24, RenderTextureFormat.ARGB32);
        cam_.targetTexture = render_texture_;
        texture2d = new Texture2D(image_width, image_height, TextureFormat.RGB24, false);

        msg = new sensor_msgs.msg.Image();
        ROS2UnityComponent ros2Component = gameObject.GetComponent<ROS2UnityComponent>();
        if (ros2Component == null)
        {
            ros2Component = gameObject.AddComponent<ROS2UnityComponent>();
        }
        ROS2Node ros2Node = ros2Component.CreateNode("UnityRGBImagePub");
        CreateROSParticipants(ros2Component, ros2Node, "");
    }

    protected override void OnUpdate()
    {
        texture2d = GetCameraTexture();
        data = texture2d.GetRawTextureData();
        data = RotateImage180(data, image_width, image_height);
        updated_data = true;
    }
    public override string frameName()
    {
        return frameId;
    }
    protected override sensor_msgs.msg.Image AcquireValue()
    {
        msg.Height = (uint)image_height;
        msg.Width = (uint)image_width;
        msg.Encoding = "bgr8";
        msg.Is_bigendian = 0;
        msg.Step = 3 * msg.Width;
        msg.Data = data;

        return msg;
    }

    protected override bool HasNewData()
    {
        if (data != null && updated_data)
        {
            updated_data = false;
            return true;
        }
        return false;
    }
    private Texture2D GetCameraTexture()
    {
        // Guardar la textura activa actual
        RenderTexture activa = RenderTexture.active;
        // Asignar la textura de la cámara como textura activa
        RenderTexture.active = cam_.targetTexture;
        // Leer los datos de la textura activa y asignarlos a la nueva textura 2D
        texture2d.ReadPixels(new Rect(0, 0, cam_.targetTexture.width, cam_.targetTexture.height), 0, 0);
        texture2d.Apply();
        // Restaurar la textura activa original
        RenderTexture.active = activa;
        return texture2d;
    }

    private byte[] RotateImage180(byte[] bytes, int width, int height)
    {
        byte[] rotatedBytes = new byte[bytes.Length];

        int i = 0;
        for (int y = height - 1; y >= 0; y--)
        {
            for (int x = width - 1; x >= 0; x--)
            {
                int index = (y * width + x) * 3;
                rotatedBytes[i++] = bytes[index + 2];
                rotatedBytes[i++] = bytes[index + 1];
                rotatedBytes[i++] = bytes[index];
            }
        }

        return rotatedBytes;
    }
}