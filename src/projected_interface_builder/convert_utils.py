#!/usr/bin/env python
# Copyright (c) 2013, Oregon State University
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Oregon State University nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL OREGON STATE UNIVERSITY BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author Dan Lazewatsky/lazewatd@engr.orst.edu

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