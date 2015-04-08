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

import pickle
import rospy
import std_msgs.msg
from std_srvs.srv import Empty, EmptyResponse
from projector_interface.srv import DrawPolygon, ClearPolygons
from visualization_msgs.msg import MarkerArray
from projected_interface_builder.convert_utils import QtPolyToROS, QtRectToPoly, toMarker
from projected_interface_builder import colors
from std_msgs.msg import ColorRGBA
import numpy as np

from threading import RLock

from dynamic_reconfigure.server import Server
from projected_interface_builder.cfg import InterfaceAdjustmentsConfig as InterfaceConfig
import tf

class ProjectedInterface(object):
    '''
    Base class for all projected interfaces. Automatically starts up service
    proxies for sending and clearing polygons, as well as a dynamic_reconfigure
    server for configuring the node, and a subscription for clicks. 
    '''
    def __init__(self, polygon_file, dispatch_rate=0.1):
        '''
        ProjectedInterface constructor.

        @param polygon_file: pickle-formatted file path containing interface data
        @param dispatch_rate:  not currently used
        '''
        self._callbacks = dict()
        self._hover_callbacks = dict()
        self._dispatch_lock = RLock()
        self._config_inited = False
        self._save_changes = False
        self._hidden = set()

        self._publish_viz = rospy.get_param('~publish_viz', True)

        with open(polygon_file, 'r') as f:
            data = pickle.load(f)
        self.polygon_file = polygon_file
        self.res = float(data['resolution'])
        self.x = float(data['offset_x'])
        self.y = float(data['offset_y'])
        self.z = float(data['offset_z'])
        self.frame_id = data['frame_id']
        self.polygons = data['polygons']
        self.polygon_colors = dict(zip(self.polygons.keys(), len(self.polygons.keys())*[colors.WHITE]))
        self.dispatch_rate = dispatch_rate

    def start(self):
        '''
        Initializes all of the service proxies, subscribers and publishers.
        Also publishes all of the polygons.
        '''
        self.polygon_proxy = rospy.ServiceProxy('/draw_polygon', DrawPolygon)
        rospy.loginfo("Waiting for polygon service")
        self.polygon_proxy.wait_for_service()
        rospy.loginfo("polygon service ready")
        rospy.loginfo("Waiting for polygon clear service")
        self.polygon_clear_proxy = rospy.ServiceProxy('/clear_polygons', ClearPolygons)
        rospy.loginfo("polygon clear service ready")
        if self._publish_viz:
            self.polygon_viz = rospy.Publisher('/polygon_viz', MarkerArray, latch=True)
        self._subscribe_to_clicks()
        self._subscribe_to_hover()
        self.dispatch_rate = rospy.Rate(self.dispatch_rate)
        reconfig_srv = Server(InterfaceConfig, self._reconfig_cb)

        self.display_mute_service = rospy.Service('~display_mute', Empty, self.display_mute)
        self.display_unmute_service = rospy.Service('~display_unmute', Empty, self.display_unmute)

        self._wait_for_frame()
        self.publish_polygons()

    def _wait_for_frame(self):
        r = rospy.Rate(10)
        tfl = tf.TransformListener()
        frame = self.frame_id.strip('/')
        rospy.loginfo('Waiting for frame %s to exist' % frame)
        while (not tfl.frameExists(frame)) and (not rospy.is_shutdown()):
            r.sleep()
        rospy.loginfo('Frame ready')

    def _reconfig_cb(self, config, level):
        '''Callback for dynamic reconfigure. Don't overwrite this'''
        # ignore the first config we get
        if self._config_inited:
            self.frame_id = config['frame_id']
            self.res = config['res']
            self.x = config['offset_x']
            self.y = config['offset_y']
            self.z = config['offset_z']
            self._save_changes = config['commit']
        else:
            config['groups']['parameters']['frame_id'] = self.frame_id
            config['res'] = self.res
            config['offset_x'] = self.x
            config['offset_y'] = self.y
            config['offset_z'] = self.z
            self._config_inited = True
        self.publish_polygons()

        return config

    def _subscribe_to_clicks(self):
        self.hover_sub = rospy.Subscriber('/clicked_object', std_msgs.msg.String, self._dispatch, queue_size=10)

    def _subscribe_to_hover(self):
        self.hover_sub = rospy.Subscriber('/hovered_object', std_msgs.msg.String, self._dispatch_hover, queue_size=10)

    def _dispatch(self, msg):
        '''Receives a click and calls any callbacks defined for the specified object.'''
        if self._dispatch_lock.acquire(blocking=False) is True:
            uid = msg.data
            # import pdb; pdb.set_trace()
            if uid in self._callbacks:
                self._callbacks[uid].__call__(self.polygons[uid])
            self._dispatch_lock.release()

    def _dispatch_hover(self, msg):
        '''Receives a hover and calls any callbacks defined for the specified objetc.'''
        if self._dispatch_lock.acquire(blocking=False) is True:
            uid = msg.data
            if uid in self._hover_callbacks:
                self._hover_callbacks[uid].__call__(self.polygons[uid])
            self._dispatch_lock.release()

    def register_callback(self, uid, func):
        '''
        Registers a callback for a particular polygon.

        @param uid: Unique ID of a polygon
        @param func: Callback function. Should take a polygon object as an argument
        '''
        self._callbacks[uid] = func

    def register_hover_callback(self, uid, func):
        '''
        Registers a hover callback for a particular polygon.

        @param uid: Unique ID of a polygon
        @param func: Callback function. Should take a polygon object as an argument
        '''
        self._hover_callbacks[uid] = func

    def publish_polygon(self, polygon):
        '''Publishes a single polygon.'''
        ps = QtPolyToROS(polygon['polygon'], polygon['uid'], self.x, self.y, self.z, self.res, self.frame_id)
        ps.header.stamp = rospy.Time.now()
        text_rect = QtPolyToROS(QtRectToPoly(polygon['text_rect']), '', self.x, self.y, self.z, self.res, self.frame_id)
        self.polygon_proxy(polygon['uid'], polygon['name'], ps, text_rect.polygon, self.polygon_colors[polygon['uid']])
        return ps

    def set_hidden(self, polygon):
        '''Hides a polygon. Has no effect if the polygon is already hidden.'''
        self._hidden.add(polygon)

    def set_nothidden(self, polygon):
        '''Unhides a polygon. Has no effect if the polygon is not hidden.'''
        if polygon in self._hidden:
            self._hidden.remove(polygon)

    def set_color(self, poly_id, colorRGB):
        self.polygon_colors[poly_id] = ColorRGBA(*list(colorRGB) + [0])

    def reset_color(self, poly_id):
        self.polygon_colors[poly_id] = colors.WHITE

    def clear_hidden(self):
        '''Unhides all polygons.'''
        self._hidden.clear()

    def publish_polygons(self):
        '''Publishes all polygons.'''
        self.polygon_clear_proxy()
        markers = MarkerArray()
        for uid, polygon in self.polygons.iteritems():
            if uid not in self._hidden:
                ps = self.publish_polygon(polygon)
                markers.markers.append(toMarker(ps, np.int32(hash(uid))))
        if self._publish_viz:
            self.polygon_viz.publish(markers)

    def display_mute(self, msg):
        self.polygon_clear_proxy()
        return EmptyResponse()

    def display_unmute(self, msg):
        self.publish_polygons()
        return EmptyResponse()

    def maybe_write_changes(self):
        '''Writes configuration changes to the interface file.'''
        # TODO: call this in __del__
        if self._save_changes:
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
