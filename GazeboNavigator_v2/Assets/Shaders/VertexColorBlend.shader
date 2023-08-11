Shader "Custom/VertexColorBlend" {
    SubShader{
        Tags { "Queue" = "Transparent" "RenderType" = "Transparent" }
        Pass {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata {
                float4 vertex : POSITION;
                float4 color : COLOR;
            };

            struct v2f {
                float4 pos : SV_POSITION;
                float4 color : COLOR;
            };

            v2f vert(appdata v) {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.color = v.color;
                return o;
            }

            fixed4 frag(v2f i) : SV_Target {
                return i.color * 0.5; // multiplicar el color del v�rtice por 0.5 para la mezcla
            }
            ENDCG
        }
    }
}
