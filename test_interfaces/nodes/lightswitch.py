import roslib; roslib.load_manifest('projected_interface_builder')
from projected_interface_builder.projected_interface import ProjectedInterface
from projected_interface_builder.data_structures import PolygonInfo
import rospy

#########
# chdown
# chup
# l_0
# l_1
# onoff
# slash
##########
class LightswitchInterface(ProjectedInterface):
    active_switch = None
    switch_on = dict(sw_bed=False, sw_bath=False)

    def __init__(self, polygon_file):
        super(LightswitchInterface, self).__init__(polygon_file)
        self.register_callback('sw_bed'   , self.switch_cb)
        self.register_callback('sw_bath'  , self.switch_cb)
        self.register_callback('on'       , self.on_cb)
        self.register_callback('off'      , self.off_cb)

        self.hideonoff()

    def hideonoff(self):
        self.set_hidden('on')
        self.set_hidden('off')

    def switch_cb(self, poly):
        if not self.active_switch:
            self.clear_hidden()
            self.publish_polygon(self.polygons['on'])
            self.publish_polygon(self.polygons['off'])
            self.interf_active = True
        self.active_switch = poly.id

        self.reset_color('sw_bed')
        self.reset_color('sw_bath')
        self.set_color(poly.id, (255, 0, 255))
        self.publish_polygon(self.polygons['sw_bed'])
        self.publish_polygon(self.polygons['sw_bath'])

    def on_cb(self, poly):
        self.polygons[self.active_switch].name = self.polygons[self.active_switch].name.replace('off', 'on')
        self.switch_on[self.active_switch] = True
        self.publish_polygon(self.polygons[self.active_switch])

    def off_cb(self, poly):
        self.polygons[self.active_switch].name = self.polygons[self.active_switch].name.replace('on', 'off')
        self.switch_on[self.active_switch] = False
        self.publish_polygon(self.polygons[self.active_switch])

    def toggle_switch(self, poly_id):
        if self.switch_on[poly_id]:
            self.polygons[poly_id].name = self.polygons[poly_id].name.replace('on', 'off')
        else:
            self.polygons[poly_id].name = self.polygons[poly_id].name.replace('off', 'on')
        self.switch_on[poly_id] = not self.switch_on[poly_id]
        self.publish_polygon(self.polygons[poly_id])

if __name__ == '__main__':
    rospy.init_node('tv_interface')
    interf = LightswitchInterface('../interfaces/lightswitch.pkl')
    interf.start()
    rospy.spin()
    interf.maybe_write_changes()

