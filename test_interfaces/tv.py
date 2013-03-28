import roslib; roslib.load_manifest('projected_interface_builder')
roslib.load_manifest('ir_comm')
from projected_interface_builder.projected_interface import ProjectedInterface
from projected_interface_builder.data_structures import PolygonInfo
import rospy
from std_msgs.msg import String
from functools import partial

#########
# chdown
# chup
# l_0
# l_1
# onoff
# slash
##########
class TVInterface(ProjectedInterface):
    tvon = False
    codebook = {
        'chup' : 'channelup_tivo',
        'chdown' : 'channeldown_tivo',
    }
    def __init__(self, polygon_file):
        super(TVInterface, self).__init__(polygon_file)
        self.code_pub = rospy.Publisher('ir_code_name', String)
        self.register_callback('onoff'  , self.onoff)
        self.register_callback('chup'   , self.send_code)
        self.register_callback('chdown' , self.send_code)

        self.set_hidden('chup')
        self.set_hidden('chdown')

    def onoff(self, poly):
        self.code_pub.publish('onoff_samsung')
        if not self.tvon:
            self.clear_hidden()
            self.publish_polygons()
        self.tvon = True

    def send_code(self, poly):
        if poly.id in self.codebook:
            self.code_pub.publish(self.codebook[poly.id])
            rospy.loginfo('Sending %s' % self.codebook[poly.id])
        else:
            rospy.logwarn('Unknown code for %s', poly.id)

if __name__ == '__main__':
    rospy.init_node('tv_interface')
    interf = TVInterface('/home/lazewatskyd/ros-pkgs/projected_interface_builder/tv.pkl')
    interf.start()
    rospy.spin()
    interf.maybe_write_changes()

