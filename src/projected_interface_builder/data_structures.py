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

    def exportable(self):
        return PolygonInfo(self.polygon, self.id, self.name)