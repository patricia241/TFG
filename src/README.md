Clonar este repositorio en el /src de tu workspace, luego estando en el raíz del workspace para compilar ejecutar:
~~~
colcon build --symlink-install
~~~
Se te deben haber generado 3 carpetas: /log /build /install. Comprobar que en /install/oculus_gz_navigator/lib existe **libCameraPlugin.so**.  
Para lanzar el mundo de Gazebo:  
~~~
ros2 launch oculus_gz_navigator view_cafe_wrld.launch.py 
~~~

Chequear que en la terminal donde has ejecutado este comando sale la traza **CAMERA POSITION PLUGIN**  

![](https://github.com/patricia241/TFG/blob/main/wiki_images/launch_example.png?raw=true)
