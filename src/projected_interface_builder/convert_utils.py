#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, PolygonStamped

def QtPolyToROS(poly, name, x, y, z, res, frame_id):
    header = rospy.Header()
    header.frame_id = frame_id
    ps = PolygonStamped(header=header)
    for pt in poly:
        ps.polygon.points.append(Point(pt.x()*res+x, pt.y()*res+y, z))
    return ps