# SISTEMA DE REALIDAD VIRTUAL EN ENTORNOS DE SIMULACIÓN PARA ROBOTS [DEVELOPING]

[![License](https://img.shields.io/badge/license-Apache--2.0-green.svg)](LICENSE.md)
![ROS](https://img.shields.io/badge/ros2-humble-brightgreen)
![Unity](https://img.shields.io/badge/unity-2021.3.16f1-brightgreen)

Este proyecto te permitirá introducirte en un mundo en Gazebo con las Oculus Quest 2. Para ello necesitarás:  
- Unity en Windows 10
- App Oculus
- VM con Ubuntu 20.04 o otro PC con un Ubuntu 20.04. Dentro debe tener ROS2 Foxy y Gazebo.  

## Bitácora del proyecto  

Para ver la evolución y toda la investigación detrás de esto puedes consultar la wiki, la cual comenzó con una entrada de instalación de ROS2 Humble y Gazebo en Windows por uno de tantos intentos, esta documentación la dejo por si es de interés para alguna otra cosa. Las demás entradas de la wiki están organizadas por meses y el desarrollo que hice en cada periodo.

## Instrucciones de uso

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
