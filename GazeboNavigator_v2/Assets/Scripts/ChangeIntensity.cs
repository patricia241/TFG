using UnityEngine;
using UnityEngine.EventSystems;

public class ChangeIntensity : MonoBehaviour
{
    public float newEmissionIntensity;
    private Material myMaterial;
    private Color initColor;

    void Start()
    {
        myMaterial = GetComponent<Renderer>().material;
        initColor = myMaterial.GetColor("_EmissionColor");
    }

    public void ChangeEmissionIntensity()
    {
        Color finalColor = initColor * Mathf.LinearToGammaSpace(newEmissionIntensity);
        myMaterial.SetColor("_EmissionColor", finalColor);

        Debug.Log("Cambiando Intensidad a " + finalColor);
    }

    public void ResetEmissionIntensity()
    {
        myMaterial.SetColor("_EmissionColor", initColor);
        Debug.Log("Reseteando Intensidad a " + initColor);
    }
}
