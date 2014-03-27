#!/usr/bin/env python
import roslib; roslib.load_manifest('projected_interface_builder')
from projected_interface_builder.projected_interface import ProjectedInterface
from projected_interface_builder.data_structures import PolygonInfo
import rospy

from sound_play.libsoundplay import SoundClient

from Py9 import Predictor


class LetterboardInterface(ProjectedInterface):
    current_words = []
    def __init__(self, polygon_file):
        super(LetterboardInterface, self).__init__(polygon_file)

        self.sound_client = SoundClient()

        self.letter_boxes = [
            'abc',
            'def',
            'ghi',
            'jkl',
            'mno',
            'pqr',
            'stuv',
            'wxyz'
        ]

        self.choice_boxes = []
        for i in xrange(1,7):
            self.choice_boxes.append(self.polygons['choice%s' % i])

        self.predictor = Predictor(dict(zip(self.letter_boxes, self.letter_boxes)))
        for box in self.letter_boxes:
            self.register_callback(box, self.letter_cb)

        for box in self.choice_boxes:
            self.register_callback(box.id, self.choice_cb)
            box.name = ''

        self.register_callback('sent', self.sent_cb)

        self.register_callback('delword'  , self.delword)
        self.register_callback('delletter', self.delletter)

        self.polygons['sent'].name = ''

    def sent_cb(self, poly):
        self.sound_client.say(self.polygons['sent'].name)
        self.polygons['sent'].name = ''
        self.publish_polygon(self.polygons['sent'])

    def letter_cb(self, poly):
        self.output_words(self.predictor.predict_incremental(poly.id))

    def choice_cb(self, poly):
        self.predictor.choose_word(poly.name)
        self.polygons['sent'].name += ' %s' % poly.name
        self.publish_polygon(self.polygons['sent'])
        for box in self.choice_boxes:
            box.name = ''
            self.publish_polygon(box)

    def output_words(self, words):
        self.current_words = words
        for word, poly in zip(words, self.choice_boxes):
            poly.name = word
            self.publish_polygon(poly)

    def delword(self, poly):
        self.predictor.delete_last_word()
        self.polygons['sent'].name = ' '.join(self.predictor.sent[1:]) # get rid of the leading period
        self.publish_polygon(self.polygons['sent'])

    def delletter(self, poly):
        self.predictor.delete_last_letter()

        for box in self.choice_boxes:
            box.name = ''

        if len(self.predictor.seq) > 0:
            self.output_words(self.predictor.predict(self.predictor.seq))


    def maybe_write_changes(self):
        self.polygons['sent'].name = ''
        for box in self.choice_boxes:
            box.name = ''

        super(LetterboardInterface, self).maybe_write_changes()

if __name__ == '__main__':
    rospy.init_node('letterboard_interface')
    interf = LetterboardInterface('/home/lazewatd/ros_ws/projected_interface_builder/test_interfaces/interfaces/physical_copy_choices_new.pkl')
    interf.start()
    rospy.spin()
    interf.maybe_write_changes()