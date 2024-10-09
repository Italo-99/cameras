#!/usr/bin/env python3

# This file publishes USO (Unknown Standing Objects) coords on RVIZ

import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import csv


def publish_coordinate():
    rospy.init_node('uso_onRViz')
    pub = rospy.Publisher('coordinate', Marker, queue_size=10)

    # Lista coordinate oggetti
    infoObj = []

    # Nome del file CSV dove salvare coordinate
    filename = 'coordinate.csv'

    while not rospy.is_shutdown():
        try:
            name = input('\n\nNome oggetto: ')
            x = float(input('Coordinata x: '))
            y = float(input('Coordinata y: '))
            z = float(input('Coordinata z: '))

            # Creo messaggio di tipo PointStamped
            point = PointStamped()
            point.header.stamp = rospy.Time.now()
            point.point.x = x
            point.point.y = y
            point.point.z = z
            point.header.frame_id = 'map'

            # Aggiungi punti alla lista
            data = {'name': name, 'point': point}
            infoObj.append(data)

            # Pubblica su RViz
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.points = [p['point'].point for p in infoObj]
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

            # Pubblica il messaggio
            pub.publish(marker)

            # Salva le coordinate dei punti su un file CSV
            with open(filename, 'a') as file:

                writer = csv.writer(file)
                writer.writerow(['Nome', 'x', 'y', 'z'])
                for p in infoObj:
                    writer.writerow([p['name'], p['point'].point.x, p['point'].point.y, p['point'].point.z])             

        except ValueError:
            rospy.logwarn('Inserisci coordinate valide')


if __name__ == '__main__':

    try:
        publish_coordinate()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass