#!/usr/bin/env python
import pickle
import sys
import hashlib
import pystache
import roslib; roslib.load_manifest('projected_interface_builder')
from projected_interface_builder.convert_utils import QtPolyToROS
from PySide.QtCore import QPoint

header = '''#!/usr/bin/env python
# FIXME this package name is just for testing
import roslib; roslib.load_manifest('projected_interface_builder')
import rospy
from projector_interface.srv import GetCursorStats, DrawPolygon, DrawPolygonRequest
from geometry_msgs.msg import Point, PolygonStamped
from projected_interface_builder.colors import *

class ProjectedInterface(object):
    def __init__(self):
        self.initPolygons()
        self.start()
        
    def start(self):
    	self.polygon_proxy = rospy.ServiceProxy('/draw_polygon', DrawPolygon)
    	rospy.loginfo("Waiting for polygon service")
    	self.polygon_proxy.wait_for_service()
    	rospy.loginfo("polygon service ready")
        rospy.loginfo("Waiting for polygon clear service")
        self.polygon_clear_proxy = rospy.ServiceProxy('/clear_polygons', ClearPolygons)
        rospy.loginfo("polygon clear service ready")
    	self.polygon_viz = rospy.Publisher('/polygon_viz', PolygonStamped)
        rospy.Subscriber('/clicked_object', String, self.dispatch)
        
        self.publish_polygons()
    
    def publish_polygons(self):
        for polygon in self.polygons.itervalues():
            self.polygon_proxy(polygon)
         
    
    def dispatch(self, msg):
        poly_name = msg.data
        poly_hash = hashlib.md5(poly_name).hexdigest()
        self.__getattribute__('_%s' % poly_hash).__call__(self.polygons['poly_name'])
    
{{functions}}


if __name__ == '__main__':
    rospy.init_node('projected_interface')
    interface = ProjectedInterface()
'''

func_template = '''
    # callback for {{funcname}}
    def _{{funchash}}(self, polygon):
        pass
'''

polygon_template = '''
        self.polygons['{{name}}'] = DrawPolygonRequest(
            '{{name}}',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='{{frame_id}}'),
            ),
            WHITE
        )
        self.polygons['{{name}}'].polygon.polygon.points = [{{points}}]
'''

def load(fname):
    with open(fname, 'r') as f:
        return pickle.load(f)

def name_dict(name):
    return dict(
        funcname =  "'%s'" % name.replace('\n', '\\n'),
        funchash = hashlib.md5(name).hexdigest()
    )

if __name__ == '__main__':
    # _, infile, outfile = sys.argv
    # _, infile, outfile = sys.argv
    data = load('physical_copy.pkl')
    res = float(data['resolution'])
    functions = ''
    polygons = '''
    def initPolygons(self):
        self.polygons=dict()'''
    for name, poly in data['polygons'].iteritems():
        namedict = name_dict(name)
        functions += pystache.render(func_template, namedict)
        points = ','.join(['Point(%s,%s,%s)' % (p.x()*res+float(data['offset_x']),p.y()*res+float(data['offset_y']),data['offset_z']) for p in poly])
        polygons  += pystache.render(
            polygon_template,
            dict(
                name=name.replace('\n', '\\n'),
                frame_id=data['frame_id'],
                points=points
            )
        )

    print pystache.render(header, dict(
        functions = polygons + functions
    ))