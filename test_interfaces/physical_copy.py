#!/usr/bin/env python
# FIXME this package name is just for testing
import roslib; roslib.load_manifest('projected_interface_builder')
import rospy
import std_msgs.msg
from projector_interface.srv import DrawPolygon, DrawPolygonRequest, ClearPolygons
from geometry_msgs.msg import Point, PolygonStamped
from projected_interface_builder.colors import *

from Py9 import Predictor

class ProjectedInterface(object):
    def __init__(self):
        self.initPolygons()
        self.predictor = Predictor(self.letter_map)
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
        self.subscribe_to_clicks()
        
        self.publish_polygons()
    
    def subscribe_to_clicks(self):
        self.click_sub = rospy.Subscriber('/clicked_object', std_msgs.msg.String, self.dispatch, queue_size=1)

    def publish_polygons(self):
        for polygon in self.polygons.itervalues():
            self.polygon_proxy(polygon)
         
    
    def dispatch(self, msg):
        self.click_sub.unregister()
        poly_name = msg.data
        self.output_words(self.predictor.predict_incremental(msg.data))
        rospy.sleep(4)
        self.subscribe_to_clicks()
        # poly_hash = hashlib.md5(poly_name).hexdigest()
        # self.__getattribute__('_%s' % poly_hash).__call__(self.polygons['poly_name'])

    def output_words(self, words):
        print words

    def initPolygons(self):
        self.polygons=dict()
        self.polygons['D E F\n2'] = DrawPolygonRequest(
            'D E F\n2',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['D E F\n2'].polygon.polygon.points = [Point(0.1792,-0.3037,0.0),Point(0.1792,-0.45,0.0),Point(-0.0507,-0.45,0.0),Point(-0.0507,-0.3037,0.0)]

        self.polygons['P Q R\n6'] = DrawPolygonRequest(
            'P Q R\n6',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['P Q R\n6'].polygon.polygon.points = [Point(-0.0507,-0.1574,0.0),Point(-0.0507,-0.01,0.0),Point(-0.285,-0.01,0.0),Point(-0.285,-0.1574,0.0)]

        self.polygons['A B C\n1'] = DrawPolygonRequest(
            'A B C\n1',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['A B C\n1'].polygon.polygon.points = [Point(-0.285,-0.45,0.0),Point(-0.285,-0.3037,0.0),Point(-0.0507,-0.3037,0.0),Point(-0.0507,-0.45,0.0)]

        self.polygons['W X Y Z\n9'] = DrawPolygonRequest(
            'W X Y Z\n9',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['W X Y Z\n9'].polygon.polygon.points = [Point(0.1792,-0.01,0.0),Point(0.419,-0.01,0.0),Point(0.419,-0.1574,0.0),Point(0.1792,-0.1574,0.0)]

        self.polygons['J K L\n4'] = DrawPolygonRequest(
            'J K L\n4',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['J K L\n4'].polygon.polygon.points = [Point(-0.0507,-0.1574,0.0),Point(-0.285,-0.1574,0.0),Point(-0.285,-0.3037,0.0),Point(-0.0507,-0.3037,0.0)]

        self.polygons['0'] = DrawPolygonRequest(
            '0',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['0'].polygon.polygon.points = [Point(-0.0507,-0.1574,0.0),Point(0.1792,-0.1574,0.0),Point(0.1792,-0.3037,0.0),Point(-0.0507,-0.3037,0.0)]

        self.polygons['M N O\n5'] = DrawPolygonRequest(
            'M N O\n5',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['M N O\n5'].polygon.polygon.points = [Point(0.419,-0.3037,0.0),Point(0.1792,-0.3037,0.0),Point(0.1792,-0.1574,0.0),Point(0.419,-0.1574,0.0)]

        self.polygons['G H I\n3'] = DrawPolygonRequest(
            'G H I\n3',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['G H I\n3'].polygon.polygon.points = [Point(0.419,-0.45,0.0),Point(0.1792,-0.45,0.0),Point(0.1792,-0.3037,0.0),Point(0.419,-0.3037,0.0)]

        self.polygons['S T U V\n7 8'] = DrawPolygonRequest(
            'S T U V\n7 8',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['S T U V\n7 8'].polygon.polygon.points = [Point(0.1792,-0.1574,0.0),Point(0.1792,-0.01,0.0),Point(-0.0507,-0.01,0.0),Point(-0.0507,-0.1574,0.0),Point(-0.0507,-0.1574,0.0)]

        self.letter_map = dict()
        for name in self.polygons.keys():
            self.letter_map[name] = name.split()


    # callback for 'D E F\n2'
    def _a0150e3c942cacbdc1d7001badc1b702(self, polygon):
        pass

    # callback for 'P Q R\n6'
    def _f7e46b2e69e2cb91a306664de63d10ef(self, polygon):
        pass

    # callback for 'A B C\n1'
    def _3d97057f7cf95081eb58b9508b980926(self, polygon):
        pass

    # callback for 'W X Y Z\n9'
    def _63ddf89d8784d23603223543a43a07a2(self, polygon):
        pass

    # callback for 'J K L\n4'
    def _5d19e5e637ac9e21f845e11cb16165ed(self, polygon):
        pass

    # callback for '0'
    def _cfcd208495d565ef66e7dff9f98764da(self, polygon):
        pass

    # callback for 'M N O\n5'
    def _a070703691153f8d56535bf990fee864(self, polygon):
        pass

    # callback for 'G H I\n3'
    def _fc08faa8e59d85e2d2727f43d04c0701(self, polygon):
        pass

    # callback for 'S T U V\n7 8'
    def _c27f196979a24373fce59d3ac734cffa(self, polygon):
        pass



if __name__ == '__main__':
    rospy.init_node('projected_interface')
    interface = ProjectedInterface()
    rospy.spin()
