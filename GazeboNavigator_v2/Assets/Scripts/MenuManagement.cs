using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class MenuManagement : MonoBehaviour
{
    public Canvas canvas; // Canvas to enable/disable
    private bool isCanvasActive = false;

    void Update()
    {
        if (OVRInput.GetDown(OVRInput.Button.Start, OVRInput.Controller.LTouch))
        {
            Debug.Log("Boton pulsado");
            isCanvasActive = !isCanvasActive;
            canvas.gameObject.SetActive(isCanvasActive);
        }
    }
}
