from PySide import QtGui, QtCore
import datetime, time
import numpy as np

FONT = QtGui.QFont('Decorative', 30)

class PolygonInfo:
    gfx_item = None
    text_item = None
    def __init__(self, polygon, uid=None, name=''):
        self.polygon = polygon
        self.name = name
        self.id = uid
        self.text_rect = polygon.boundingRect()
        self.update_font_box()
        if uid is None:
            self.id = self._gen_id()

    def update_font_box(self, font_metrics=QtGui.QFontMetrics(FONT)):
        bounding_rect = font_metrics.boundingRect(self.name)
        lines = self.name.split('\n')
        nlines = len(lines)
        longest_line = lines[np.argmax([len(l) for l in lines])]
        
        bounding_rect.setWidth(font_metrics.width(longest_line)*1.25)
        bounding_rect.setHeight((font_metrics.height()+5)*nlines)
        bounding_rect.moveCenter(self.text_rect.center())
        self.text_rect = bounding_rect

    def zValue(self):
        return self.gfx_item.zValue()

    def update_item(self):
        self.gfx_item.setPolygon(self.polygon)

    def set_item(self, item):
        self.gfx_item = item

    def clear_item(self):
        self.gfx_item = None

    def set_text_item(self, item):
        self.text_item = item

    def _gen_id(self):
        return 'poly%s' % int(time.mktime(datetime.datetime.now().timetuple()))
        
    def in_poly(self, pt):
        return pnpoly(pt.x(), pt.y(), [(p.x(), p.y()) for p in self.polygon])
        
    def in_text_box(self, pt):
        return pt[0] > self.text_rect.left() \
           and pt[0] < self.text_rect.right() \
           and pt[1] < self.text_rect.bottom() \
           and pt[1] > self.text_rect.top()
