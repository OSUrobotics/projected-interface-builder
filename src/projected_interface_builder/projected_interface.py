import pickle
import roslib; roslib.load_manifest('projected_interface_builder')
import rospy
import std_msgs.msg
from projector_interface.srv import DrawPolygon, DrawPolygonRequest, ClearPolygons
from geometry_msgs.msg import Point, PolygonStamped
from projected_interface_builder.colors import *
from projected_interface_builder.convert_utils import QtPolyToROS, QtRectToPoly
from projected_interface_builder.data_structures import PolygonInfo
from projected_interface_builder import colors

from threading import RLock

from dynamic_reconfigure.server import Server
from projected_interface_builder.cfg import InterfaceConfig

class ProjectedInterface(object):
    callbacks = dict()
    dispatch_lock = RLock()
    config_inited = False
    save_changes = False
    def __init__(self, polygon_file, dispatch_rate=0.1):
        with open(polygon_file, 'r') as f:
            data = pickle.load(f)
        self.polygon_file = polygon_file
        self.res = float(data['resolution'])
        self.x = float(data['offset_x'])
        self.y = float(data['offset_y'])
        self.z = float(data['offset_z'])
        self.frame_id = data['frame_id']
        self.polygons = data['polygons']
        self.dispatch_rate = dispatch_rate
        
    def start(self):
    	self.polygon_proxy = rospy.ServiceProxy('/draw_polygon', DrawPolygon)
    	rospy.loginfo("Waiting for polygon service")
    	self.polygon_proxy.wait_for_service()
    	rospy.loginfo("polygon service ready")
        rospy.loginfo("Waiting for polygon clear service")
        self.polygon_clear_proxy = rospy.ServiceProxy('/clear_polygons', ClearPolygons)
        rospy.loginfo("polygon clear service ready")
    	self.polygon_viz = rospy.Publisher('/polygon_viz', PolygonStamped)
        self.subscribe_to_clicks()
        self.dispatch_rate = rospy.Rate(self.dispatch_rate)
        reconfig_srv = Server(InterfaceConfig, self.reconfig_cb)

        self.publish_polygons()

    def reconfig_cb(self, config, level):
        # ignore the first config we get
        # import pdb; pdb.set_trace()
        if self.config_inited:
            self.frame_id = config['frame_id']
            self.res = config['res']
            self.x = config['offset_x']
            self.y = config['offset_y']
            self.z = config['offset_z']
            self.save_changes = config['commit']
        else:
            config['groups']['parameters']['frame_id'] = self.frame_id
            config['res'] = self.res
            config['offset_x'] = self.x
            config['offset_y'] = self.y
            config['offset_z'] = self.z
            self.config_inited = True
        self.publish_polygons()

        return config

    def subscribe_to_clicks(self):
        self.click_sub = rospy.Subscriber('/clicked_object', std_msgs.msg.String, self.dispatch, queue_size=1)
        
    def dispatch(self, msg):
        if self.dispatch_lock.acquire(blocking=False) is True:
            uid = msg.data
            if uid in self.callbacks:
                self.callbacks[uid].__call__(self.polygons[uid])
            self.dispatch_lock.release()
        
    def register_callback(self, uid, func):
        self.callbacks[uid] = func

    def publish_polygon(self, polygon):
        ps = QtPolyToROS(polygon.polygon, polygon.id, self.x, self.y, self.z, self.res, self.frame_id)
        ps.header.stamp = rospy.Time.now()
        text_rect = QtPolyToROS(QtRectToPoly(polygon.text_rect), '', self.x, self.y, self.z, self.res, self.frame_id)
        self.polygon_proxy(polygon.id, polygon.name, ps, text_rect.polygon, colors.WHITE)
            
    def publish_polygons(self):
        self.polygon_clear_proxy()
        for uid, polygon in self.polygons.iteritems():
            self.publish_polygon(polygon)

    def maybe_write_changes(self):
        if self.save_changes:
            with open(self.polygon_file, 'w') as f:
                data = dict()
                data['resolution'] = self.res
                data['offset_x'] = self.x
                data['offset_y'] = self.y
                data['offset_z'] = self.z
                data['frame_id'] = self.frame_id
                data['polygons'] = self.polygons
                pickle.dump(data, f)
                rospy.loginfo('Wrote changes to %s' % self.polygon_file)