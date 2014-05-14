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

from PySide import QtGui, QtCore
import datetime, time
import numpy as np
from projected_interface_builder import modes
from functools import partial

FONT = QtGui.QFont('Decorative', 30)

class PolygonInfo(QtGui.QGraphicsPolygonItem):
    class Signaler(QtCore.QObject):
        changed = QtCore.Signal(QtGui.QPolygonF)
        focus = QtCore.Signal(bool)
        moved = QtCore.Signal(QtCore.QPoint)

    class PolygonVertex(QtGui.QGraphicsRectItem):
        def __init__(self, rect, parent=None):
            super(PolygonInfo.PolygonVertex, self).__init__(rect, parent)
            self.setAcceptHoverEvents(True)
            self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
            self.signaler = PolygonInfo.Signaler()

        def hoverEnterEvent(self, event):
            self.setBrush(QtGui.QBrush(QtCore.Qt.gray))
            self.parentItem().setFlag(QtGui.QGraphicsItem.ItemIsMovable, False)

        def hoverLeaveEvent(self, event):
            self.setBrush(QtGui.QBrush(QtCore.Qt.gray, bs=QtCore.Qt.NoBrush))
            self.parentItem().setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)

        def mouseMoveEvent(self, event):
            self.signaler.moved.emit(self.mapToScene(self.rect().center()).toPoint())
            super(PolygonInfo.PolygonVertex, self).mouseMoveEvent(event)

    text_item = None
    def __init__(self, polygon=None, text_rect=None, uid=None, name=''):
        super(PolygonInfo, self).__init__(polygon)
        if not polygon:
            raise TypeError('A polygon is required')
        self.name = name
        self.id = uid
        if text_rect:
            self.text_rect = text_rect
        else:
            self.text_rect = polygon.boundingRect()
        self.text_item = QtGui.QGraphicsTextItem(name, parent=self)
        self.text_item.setPos(self.text_rect.center())
        self.text_item.setFont(FONT)

        self.setFlags(
            QtGui.QGraphicsItem.ItemIsMovable
          | QtGui.QGraphicsItem.ItemIsSelectable
          | QtGui.QGraphicsItem.ItemIsFocusable
        )

        self.update_font_box()
        if uid is None:
            self.id = self._gen_id()

        self.signaler = PolygonInfo.Signaler()

    def update_font_box(self, font_metrics=QtGui.QFontMetrics(FONT)):
        if self.text_item:
            self.text_item.setHtml(self.name)
        bounding_rect = font_metrics.boundingRect(self.name)
        lines = self.name.split('\n')
        nlines = len(lines)
        longest_line = lines[np.argmax([len(l) for l in lines])]
        
        bounding_rect.setWidth(font_metrics.width(longest_line)*1.25)
        bounding_rect.setHeight((font_metrics.height()+5)*nlines)
        c = self.text_rect.center()
        if type(c) is QtCore.QPointF: c = c.toPoint()
        bounding_rect.moveCenter(c)
        self.text_rect = bounding_rect

    def update_item(self):
        self.setPolygon(self.polygon)

    def clear_item(self):
        raise DeprecationWarning('PolygonInfo is now a subclass of QGraphicsPolygonItem. \
            Functions that touch gfx_item are no longer supported.')

    def set_text_item(self, item):
        self.text_item = item
        self.text_item.setParentItem(self)

    def _gen_id(self):
        return 'poly%s' % int(time.mktime(datetime.datetime.now().timetuple()))
        
    def in_poly(self, pt):
        return pnpoly(pt.x(), pt.y(), [(p.x(), p.y()) for p in self.polygon])
        
    def in_text_box(self, pt):
        return pt[0] > self.text_rect.left() \
           and pt[0] < self.text_rect.right() \
           and pt[1] < self.text_rect.bottom() \
           and pt[1] > self.text_rect.top()

    def exportable(self):
        # return PolygonInfo(self.polygon(), self.id, self.name)
        return dict(
            polygon=self.mapToScene(self.polygon()).toPolygon(),
            text_rect=self.mapToScene(self.text_rect).boundingRect().toRect(),
            uid=self.id,
            name=self.name)


    def mouseMoveEvent(self, event):
        self.signaler.changed.emit(self.mapToScene(self.polygon()))
        super(PolygonInfo, self).mouseMoveEvent(event)

    def focusInEvent(self, event):
        self.signaler.focus.emit(True)
        super(PolygonInfo, self).focusInEvent(event)
        self.showControlPoints()

    def focusOutEvent(self, event):
        self.signaler.focus.emit(False)
        super(PolygonInfo, self).focusOutEvent(event)

    def showControlPoints(self):
        template = QtCore.QRectF(0,0,6,6)
        rect_pen = self.pen()
        rect_pen.setColor(QtCore.Qt.white)
        for idx, pt in enumerate(self.polygon()):
            rect = QtCore.QRectF(template)
            rect.moveCenter(pt)
            rectItem = PolygonInfo.PolygonVertex(rect, parent=self)
            rectItem.signaler.moved.connect(partial(self.vertexMoved, idx))
            rectItem.setPen(rect_pen)

    def vertexMoved(self, idx, point):
        poly = self.polygon()
        poly.replace(idx, point)
        self.setPolygon(poly)
        self.signaler.changed.emit(self.mapToScene(self.polygon()))