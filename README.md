# RI

![download](https://github.com/CarloHSUA/RI/blob/main/assets/RI.gif)


# Resumen

En este estudio, desarrollamos un sistema de teleoperación para un robot móvil basado en gestos, implementado en el entorno ROS (Robot Operating System).
Utilizando técnicas de aprendizaje profundo, el sistema reconoce los gestos de la mano a partir de imágenes capturadas por la cámara del operador y los traduce en comandos de control para el robot.
Para garantizar la seguridad, una capa de seguridad procesa estos comandos para evitar colisiones.
Los resultados experimentales en un entorno simulado demuestran una alta precisión en el reconocimiento de gestos y una efectiva evitación de obstáculos, mostrando el potencial del sistema para un control del robot natural e intuitivo.
El trabajo futuro se centrará en mejorar el reconocimiento de gestos complejos y la respuesta a obstáculos repentinos.

# Requisitos
Es necesario tener un equipo con un sistema operativo de Linux. También se puede realizar en la WSL desde windows.
1. Instalar Docker [aquí](https://docs.docker.com/engine/install/ubuntu/).
2. Tener una Webcam conectada.

# Ejecución
Una vez instaladas los requisitos necesarios, podemos ejecutar el siguiente programa.
  1. Se le da permisos de ejecución al script en bash para poder ejecutar el programa y sus funcionalidades.
```
sudo chmod +x init_docker.sh
``` 
  
  2. En una terminal separada, se ejecuta el siguiente comando, que inicializa el servicio de Gazebo. Esto abre una nueva terminal temporal.
```
./init_docker.sh container
``` 
  3. En la misma terminal principal, se puede ejecutar el siguiente comando encargado de la navegación mediante teleoperaciones con gestos de las manos.
```
./init_docker.sh teleop
```
