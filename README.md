# SIMULADOR ROBÓTICO INMERSIVO CON ROS 2 Y UNITY

[![License](https://img.shields.io/badge/license-Apache--2.0-green.svg)](LICENSE.md)
![ROS](https://img.shields.io/badge/ros2-humble-brightgreen)
![Unity](https://img.shields.io/badge/unity-2021.3.16f1-brightgreen)

Este proyecto te permitirá introducirte en el mundo Granny Annie que podemos encontrar en el simulador Gazebo con las Oculus Quest 2, además
se ha integrado el robot TiaGo de PAL Robotics, el cual podemos teleoperar con los controladores Oculus (se publican mensajes ROS 2 de tipo Twist para este cometido)
y se ha implementado un sensor RGB-D del cuál se publican la imagen RGB y la de profundidad en diferentes topics.  
Para hacer uso de este proyecto Unity se deben cumplir los siguientes requisitos:
- Windows 10 con ROS 2 Humble
- Unity editor 2021.3.16f1
- App Oculus  

## Bitácora del proyecto  

Para ver la evolución y toda la investigación detrás de esto puedes consultar la wiki, la cual comenzó con una entrada de instalación de ROS2 Humble y Gazebo en Windows, se intentó conectar las Oculus directamente a Gazebo en Linux (opción descartada por incompatibilidad con los drivers, solo para Windows), posteriormente se quiso tener una cámara en gazebo con su propio plugin para que se moviera igual que las gafas VR en Unity con Windows y controlar y enviar dichas imagenes a las gafas a través de topics ROS 2 (opción descartada por gran saturación de la red y falta de inmersividad al no poder visualizar las imagenes tal cual en las gafas), por último se optó por realizar un simulador robótico en Unity donde ROS 2 esté integrado.  

Las entradas de la wiki se encuentran organizadas por meses y cada uno de ellos se divide en una serie de retos que se han ido realizando hasta alcanzar el resultado final.

## Instrucciones de uso

Clonar este repositorio y abrir la carpeta GazeboNavigator_v2 (proyecto Unity) en Unity. Asegurarse de tener instalados los paquetes OculusIntegration, URDF-Importer, XR Interaction Toolkit, Oculus XR Plugin, Post Processing, TextMeshPro y el asset Ros2 for Unity.  

Una vez cumplidos todos los requisitos se debe abrir la app Oculus, asegurarse de que está configurado para usar las Oculus con cable y conectar las Oculus Quest 2 con el cable USB-C. Luego dentro de las Oculus se debe visualizar el escritorio Windows, para ello se debe acceder a Oculus Link y una vez dentro seleccionar el escritorio. Por último en Unity seleccionar la escena Menú y darle al play.  
