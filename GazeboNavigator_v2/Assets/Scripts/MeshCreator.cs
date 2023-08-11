using UnityEngine;
using static UnityEngine.Rendering.DebugUI.Table;

public class MeshCreator : MonoBehaviour
{
    public int rows, cols;
    public Vector3[,] points;

    void Start()
    {
        Mesh mesh = new Mesh();

        points = new Vector3[rows, cols];

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                points[i, j] = new Vector3(i, j, (float)(i * j * 10) / (float)(rows * cols));
            }
        }


        // Creamos una lista de vértices a partir de los puntos en la matriz
        Vector3[] vertices = new Vector3[points.GetLength(0) * points.GetLength(1)];
        for (int i = 0; i < points.GetLength(0); i++)
        {
            for (int j = 0; j < points.GetLength(1); j++)
            {
                vertices[i * points.GetLength(1) + j] = points[i, j];
            }
        }

        // Creamos la lista de triángulos de la malla
        int[] triangles = new int[(points.GetLength(0) - 1) * (points.GetLength(1) - 1) * 6];
        int index = 0;
        for (int i = 0; i < points.GetLength(0) - 1; i++)
        {
            for (int j = 0; j < points.GetLength(1) - 1; j++)
            {
                int a = i * points.GetLength(1) + j;
                int b = (i + 1) * points.GetLength(1) + j;
                int c = i * points.GetLength(1) + j + 1;
                int d = (i + 1) * points.GetLength(1) + j + 1;
                triangles[index++] = a;
                triangles[index++] = d;
                triangles[index++] = b;
                triangles[index++] = a;
                triangles[index++] = c;
                triangles[index++] = d;
            }
        }

        // Asignamos la lista de vértices y triángulos a la malla
        mesh.SetVertices(vertices);
        mesh.SetTriangles(triangles, 0);

        GetComponent<MeshFilter>().mesh = mesh;
    }
}