import roslib; roslib.load_manifest('projected_interface_builder')
from projected_interface_builder.projected_interface import ProjectedInterface
from projected_interface_builder.data_structures import PolygonInfo
import rospy

from Py9 import Predictor


class LetterboardInterface(ProjectedInterface):
	current_words = []
	def __init__(self, polygon_file):
		super(LetterboardInterface, self).__init__(polygon_file)
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

	def letter_cb(self, poly):
		self.output_words(self.predictor.predict_incremental(poly.id))

	def output_words(self, words):
		self.current_words = words
		for word, poly in zip(words, self.choice_boxes):
			poly.name = word
			self.publish_polygon(poly)

if __name__ == '__main__':
	rospy.init_node('test_if')
	interf = LetterboardInterface('/home/lazewatskyd/ros-pkgs/projected_interface_builder/physical_copy_choices_new.pkl')
	interf.start()
	rospy.spin()