#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ackermann_msgs.msg import AckermannDrive
import numpy as np

class ObstacleAvoidance:
    def __init__(self):
        # Inicializa el nodo de ROS
        rospy.init_node('obstacle_avoidance')

        # Suscriptor para la nube de puntos de los obstáculos captados por el lidar
        rospy.Subscriber("/obstacles", PointCloud2, self.obstacle_callback)

        # Suscriptor para comandos de control Ackermann
        rospy.Subscriber("/blue/preorder_ackermann_cmd", AckermannDrive, self.ackermann_callback)

        # TODO Publicador para comandos Ackermann modificados
        self.cmd_pub = rospy.Publisher("/blue/ackermann_cmd", AckermannDrive, queue_size=10)

        # Almacenar el último mensaje Ackermann recibido
        self.last_ackermann_cmd = AckermannDrive()

    def obstacle_callback(self, msg):
        # Procesar la nube de puntos con los obstáculos teniendo en cuenta el último mensaje de movimiento recibido para evitar colisiones
        cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        if len(cloud_points) == 0:
            rospy.loginfo("No obstacles detected.")
            cmd = self.last_ackermann_cmd
        else:
            cloud_array = np.array(cloud_points)

            # Definir un umbral de distancia para evitar colisiones
            threshold_distance = 1.0  # metros

            # Obtener las distancias y posiciones de los puntos detectados
            distances = np.sqrt(cloud_array[:, 0]**2 + cloud_array[:, 1]**2)
            
            # Verificar si hay algún obstáculo dentro del umbral de distancia
            if np.any(distances < threshold_distance):
                rospy.loginfo("Obstacle detected, modifying Ackermann command")

                # Verificar la posición del obstáculo
                closest_point = cloud_array[np.argmin(distances)]
                if closest_point[0] >= 0:
                    # El obstáculo está delante del robot
                    cmd = self.modify_ackermann_command(speed=-1.0)
                else:
                    # El obstáculo está detrás del robot
                    cmd = self.modify_ackermann_command(speed=1.0)
            else:
                cmd = self.last_ackermann_cmd

        # Publicar el comando Ackermann modificado
        self.cmd_pub.publish(cmd)
        
        return

    def ackermann_callback(self, msg):
        # Almacenar el último comando recibido
        self.last_ackermann_cmd = msg

    def modify_ackermann_command(self, speed):
        # Modificar el comando Ackermann para evitar obstáculos
        cmd = self.last_ackermann_cmd
        cmd.steering_angle = 0.0
        cmd.speed = speed
        return cmd

        

if __name__ == '__main__':
    oa = ObstacleAvoidance()
    rospy.spin()
