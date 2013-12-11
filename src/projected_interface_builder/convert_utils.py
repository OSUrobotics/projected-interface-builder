#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, PolygonStamped
from PySide.QtGui import QPolygon
from PySide.QtCore import QPoint
from visualization_msgs.msg import Marker

def QtPolyToROS(poly, name, x, y, z, res, frame_id):
    header = rospy.Header()
    header.frame_id = frame_id
    ps = PolygonStamped(header=header)
    for pt in poly:
        # Qt defines 0,0 in the upper left, so, and down as +y, so flip the y
        ps.polygon.points.append(Point(pt.x()*res+x, -pt.y()*res+y, z))
    return ps

def QtRectToPoly(rect):
	x1,y1,x2,y2 = rect.getCoords()
	poly = QPolygon()
	poly.push_back(QPoint(x1,y1))
	poly.push_back(QPoint(x2,y1))
	poly.push_back(QPoint(x2,y2))
	poly.push_back(QPoint(x1,y2))
	return poly

def toMarker(poly, uid):
	marker = Marker()
	marker.header.stamp = rospy.Time.now()
	marker.header.frame_id = str(poly.header.frame_id)
	marker.type = Marker.LINE_STRIP
	marker.action = Marker.ADD
	marker.scale.x = 0.01
	marker.pose.orientation.w = 1.0
	marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
	marker.id = uid

	for pt in poly.polygon.points:
		marker.points.append(pt)
	marker.points.append(poly.polygon.points[0])

	return marker