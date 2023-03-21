using UnityEngine;

public class TriangleMesh : MonoBehaviour
{
    void Start()
    {
        Vector3[] vertices = new Vector3[4];
        Color[] colors = new Color[4];
        int[] triangles = new int[6];

        vertices[0] = new Vector3(0, 1, 0);
        vertices[1] = new Vector3(1, 1, 0);
        vertices[2] = new Vector3(0, 0, 0);
        vertices[3] = new Vector3(1, 0, 0);

        colors[0] = Color.red;
        colors[1] = Color.red;
        colors[2] = Color.red;
        colors[3] = Color.blue;

        triangles[0] = 0;
        triangles[1] = 1;
        triangles[2] = 2;
        triangles[3] = 2;
        triangles[4] = 1;
        triangles[5] = 3;

        Mesh  mesh = new Mesh();

        mesh.vertices = vertices;
        mesh.colors = colors;
        mesh.triangles = triangles; 

        GameObject gameObject = new GameObject("Mesh", typeof(MeshFilter), typeof(MeshRenderer));
        gameObject.transform.localScale = new Vector3(30, 30, 1);

        gameObject.GetComponent<MeshFilter>().mesh = mesh;
        gameObject.GetComponent<MeshRenderer>().material = new Material(Shader.Find("Custom/VertexColorBlend"));
    }
}
