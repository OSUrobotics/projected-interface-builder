#!/usr/bin/env python
import pickle
import sys
import hashlib
import pystache

header = '''
#!/usr/bin/env python
import roslib; roslib.load_manifest(PKG_NAME_HERE)
import rospy
from projector_interface.srv import GetCursorStats, DrawPolygon, CircleInhibit

class ProjectedInterface(object):
    
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
        
    
    def dispatch(self, msg):
        poly_name = msg.data
        poly_hash = hashlib.md5(poly_name).hexdigest()
        self.__getattribute__('_%s' % poly_hash).__call__()
    
{{functions}}


if __name__ == '__main__':
    rospy.init_node('projected_interface')
    interface = ProjectedInterface()
    interface.start()
'''

func_template = '''
    # callback for {{funcname}}
    def _{{funchash}}(self):
        pass
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
    functions = ''
    for name, poly in data['polygons'].iteritems():
        functions += pystache.render(func_template, name_dict(name))

    print pystache.render(header, dict(
        functions = functions
    ))