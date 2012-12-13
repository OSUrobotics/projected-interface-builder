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
    current_words = []
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
        self.polygon_clear_proxy()
        self.publish_polygons()
    
    def subscribe_to_clicks(self):
        self.click_sub = rospy.Subscriber('/clicked_object', std_msgs.msg.String, self.dispatch, queue_size=1)

    def publish_polygons(self):
        for polygon in self.polygons.itervalues():
            self.polygon_proxy(polygon)
         
    
    def dispatch(self, msg):
        self.click_sub.unregister()
        poly_name = msg.data

        if poly_name in self.current_words:
            self.sentence_poly.name += ' %s' % poly_name
            self.current_words = []
            self.predictor.choose_word(poly_name)
            self.clear_choices()
            self.publish_polygons()
        else:
            self.output_words(self.predictor.predict_incremental(msg.data))

        rospy.sleep(4)
        self.subscribe_to_clicks()

    def clear_choices(self):
        for i, poly in enumerate(self.word_polygons):
            # del self.polygons[poly.name]
            poly.name = ' '*i
            self.polygons[poly.name] = poly
            self.polygon_clear_proxy()

    def output_words(self, words):
        self.current_words = words
        for poly in self.word_polygons:
            poly.display_name = False
            del self.polygons[poly.name]
        for word, poly in zip(words, self.word_polygons):
            poly.display_name = True
            poly.name = word
            self.polygons[word] = poly
        self.polygon_clear_proxy()
        self.publish_polygons()

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
        self.polygons['D E F\n2'].polygon.polygon.points = [Point(0.121,-0.3404,0.0),Point(0.121,-0.468,0.0),Point(-0.082,-0.468,0.0),Point(-0.082,-0.3404,0.0)]

        self.polygons['P Q R\n6'] = DrawPolygonRequest(
            'P Q R\n6',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['P Q R\n6'].polygon.polygon.points = [Point(-0.082,-0.2128,0.0),Point(-0.082,-0.0852,0.0),Point(-0.285,-0.0852,0.0),Point(-0.285,-0.2128,0.0)]

        self.polygons['Sentence'] = DrawPolygonRequest(
            'Sentence',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['Sentence'].polygon.polygon.points = [Point(0.4574,-0.0852,0.0),Point(0.4574,-0.004,0.0),Point(-0.285,-0.004,0.0),Point(-0.285,-0.0852,0.0)]

        self.polygons['A B C\n1'] = DrawPolygonRequest(
            'A B C\n1',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['A B C\n1'].polygon.polygon.points = [Point(-0.285,-0.468,0.0),Point(-0.285,-0.3404,0.0),Point(-0.082,-0.3404,0.0),Point(-0.082,-0.468,0.0)]

        self.polygons['Choice 5'] = DrawPolygonRequest(
            'Choice 5',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['Choice 5'].polygon.polygon.points = [Point(0.324,-0.2128,0.0),Point(0.324,-0.149,0.0),Point(0.4574,-0.149,0.0),Point(0.4574,-0.2128,0.0)]

        self.polygons['W X Y Z\n9'] = DrawPolygonRequest(
            'W X Y Z\n9',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['W X Y Z\n9'].polygon.polygon.points = [Point(0.121,-0.0852,0.0),Point(0.324,-0.0852,0.0),Point(0.324,-0.2128,0.0),Point(0.121,-0.2128,0.0)]

        self.polygons['Choice 2'] = DrawPolygonRequest(
            'Choice 2',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['Choice 2'].polygon.polygon.points = [Point(0.4574,-0.3404,0.0),Point(0.4574,-0.4042,0.0),Point(0.324,-0.4042,0.0),Point(0.324,-0.3404,0.0)]

        self.polygons['J K L\n4'] = DrawPolygonRequest(
            'J K L\n4',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['J K L\n4'].polygon.polygon.points = [Point(-0.082,-0.2128,0.0),Point(-0.285,-0.2128,0.0),Point(-0.285,-0.3404,0.0),Point(-0.082,-0.3404,0.0)]

        self.polygons['Choice 4'] = DrawPolygonRequest(
            'Choice 4',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['Choice 4'].polygon.polygon.points = [Point(0.324,-0.2766,0.0),Point(0.324,-0.2128,0.0),Point(0.4574,-0.2128,0.0),Point(0.4574,-0.2766,0.0)]

        self.polygons['Choice 6'] = DrawPolygonRequest(
            'Choice 6',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['Choice 6'].polygon.polygon.points = [Point(0.324,-0.149,0.0),Point(0.324,-0.0852,0.0),Point(0.4574,-0.0852,0.0),Point(0.4574,-0.149,0.0)]

        self.polygons['0'] = DrawPolygonRequest(
            '0',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['0'].polygon.polygon.points = [Point(-0.082,-0.2128,0.0),Point(0.121,-0.2128,0.0),Point(0.121,-0.3404,0.0),Point(-0.082,-0.3404,0.0)]

        self.polygons['Choice 1'] = DrawPolygonRequest(
            'Choice 1',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['Choice 1'].polygon.polygon.points = [Point(0.324,-0.468,0.0),Point(0.4574,-0.468,0.0),Point(0.4574,-0.4042,0.0),Point(0.324,-0.4042,0.0)]

        self.polygons['M N O\n5'] = DrawPolygonRequest(
            'M N O\n5',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['M N O\n5'].polygon.polygon.points = [Point(0.324,-0.3404,0.0),Point(0.121,-0.3404,0.0),Point(0.121,-0.2128,0.0),Point(0.324,-0.2128,0.0)]

        self.polygons['G H I\n3'] = DrawPolygonRequest(
            'G H I\n3',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['G H I\n3'].polygon.polygon.points = [Point(0.324,-0.468,0.0),Point(0.121,-0.468,0.0),Point(0.121,-0.3404,0.0),Point(0.324,-0.3404,0.0)]

        self.polygons['Choice 3'] = DrawPolygonRequest(
            'Choice 3',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['Choice 3'].polygon.polygon.points = [Point(0.324,-0.2766,0.0),Point(0.4574,-0.2766,0.0),Point(0.4574,-0.3404,0.0),Point(0.324,-0.3404,0.0)]

        self.polygons['S T U V\n7 8'] = DrawPolygonRequest(
            'S T U V\n7 8',
            True,
            PolygonStamped(
                header=rospy.Header(frame_id='/table'),
            ),
            WHITE
        )
        self.polygons['S T U V\n7 8'].polygon.polygon.points = [Point(0.121,-0.2128,0.0),Point(0.121,-0.0852,0.0),Point(-0.082,-0.0852,0.0),Point(-0.082,-0.2128,0.0)]

        self.letter_map = dict()
        for name in self.polygons.keys():
            if not name.startswith('Choice') and name is not 'Sentence':
                self.letter_map[name] = name.split()

        self.word_polygons = [poly for (name, poly) in self.polygons.iteritems() if name.startswith('Choice')]
        self.sentence_poly = self.polygons['Sentence']
        self.sentence_poly.display_name = False
        self.sentence_poly.name = ''

if __name__ == '__main__':
    rospy.init_node('projected_interface')
    interface = ProjectedInterface()
    rospy.spin()
