import pickle
import roslib; roslib.load_manifest('projected_interface_builder')
import rospy
import std_msgs.msg
from projector_interface.srv import DrawPolygon, DrawPolygonRequest, ClearPolygons
from geometry_msgs.msg import Point, PolygonStamped
from projected_interface_builder.colors import *
from projected_interface_builder.convert_utils import QtPolyToROS, QtRectToPoly

class ProjectedInterface(object):
    def __init__(self, polygon_file):
        with open(polygon_file, 'r') as f:
            data = pickle.load(f)
        self.res = float(data['resolution'])
        self.x = float(data['offset_x']
        self.y = float(data['offset_y'])
        self.z = float(data['offset_z'])
        self.frame_id = float(data['frame_id'])
        self.polygons = data['polygons']
        
    def start(self):
    	self.polygon_proxy = rospy.ServiceProxy('/draw_polygon', DrawPolygon)
    	rospy.loginfo("Waiting for polygon service")
    	self.polygon_proxy.wait_for_service()
    	rospy.loginfo("polygon service ready")
        rospy.loginfo("Waiting for polygon clear service")
        self.polygon_clear_proxy = rospy.ServiceProxy('/clear_polygons', ClearPolygons)
        rospy.loginfo("polygon clear service ready")
    	self.polygon_viz = rospy.Publisher('/polygon_viz', PolygonStamped)
        rospy.Subscriber('/clicked_object', std_msgs.msg.String, self.dispatch)
        self.publish_polygons()
        
    def dispatch(self, msg):
        uid = msg.data
        if hasattr(self, uid):
            getattr(self, uid).__call__(self.polygons[uid])
        
    def publish_polygon(self, polygon):
        ps = QtPolyToROS(poly_info.polygon, polygon.uid, self.x, self.y, self.z, self.res, self.frame_id)
        ps.header.stamp = rospy.Time.now()
        text_rect = QtPolyToROS(QtRectToPoly(polygon.text_rect), '', self.x, self.y, self.z, self.res, self.frame_id)
        self.polygon_proxy(uid, poly_info.name, ps, text_rect.polygon, Colors.WHITE)
            
    def publish_polygons(self):
        self.polygon_clear_proxy()
        for uid, polygon in self.polygons.iteritems():
            self.publish_polygon(polygon)