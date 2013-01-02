#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, PolygonStamped
from PySide.QtGui import QPolygon
from PySide.QtCore import QPoint

def QtPolyToROS(poly, name, x, y, z, res, frame_id):
    header = rospy.Header()
    header.frame_id = frame_id
    ps = PolygonStamped(header=header)
    for pt in poly:
        ps.polygon.points.append(Point(pt.x()*res+x, pt.y()*res+y, z))
    return ps

def QtRectToPoly(rect):
	x1,y1,x2,y2 = rect.getCoords()
	poly = QPolygon()
	poly.push_back(QPoint(x1,y1))
	poly.push_back(QPoint(x2,y1))
	poly.push_back(QPoint(x2,y2))
	poly.push_back(QPoint(x1,y2))
	return poly