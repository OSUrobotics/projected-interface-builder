#!/usr/bin/env python
import PySide
from PySide import QtGui, QtCore
from PySide.QtGui import QPalette

import sys

class Builder(QtGui.QWidget):
    objects = []
    polygon_active = False
    current_poly = []
    def __init__(self):
        super(Builder, self).__init__()
        self.setGeometry(0, 480, 640, 480)
        timer = PySide.QtCore.QTimer(self)
        timer.timeout.connect(self.update)
        timer.start()
        self.show()

    def mousePressEvent(self, event):
        if (event.button() == QtCore.Qt.MouseButton.LeftButton) and (self.polygon_active):
            if len(self.current_poly) == 1:
                self.current_poly[0][1] = (event.x(), event.y())
            else:
                self.current_poly.extend([self.current_poly[-1],event.pos()])
        elif (event.button() == QtCore.Qt.MouseButton.LeftButton) and (not self.polygon_active):
            self.current_poly.extend([event.pos(),event.pos()])
            self.polygon_active = True
               
    def mouseDoubleClickEvent(self, event):
        if len(self.current_poly) == 1:
            self.current_poly[0][1] = (event.x(), event.y())
        else:
            self.current_poly.extend([self.current_poly[-1],event.pos()])
        
        poly = PySide.QtGui.QPolygon.fromList(self.current_poly)
        self.objects.append(QtGui.QPolygon(poly))
        self.polygon_active = False
        self.current_poly = []        
        
                
    def mouseMoveEvent(self, event):
        pass

    def mouseHoverEvent(self, event):
        print event

    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)
        qp.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        if self.polygon_active:
            qp.drawLines(self.current_poly)
        for obj in self.objects:
            # import pdb; pdb.set_trace()
            qp.drawPolygon(obj)
        qp.end()

if __name__ == '__main__':
    app = PySide.QtGui.QApplication(sys.argv)
    gui = Builder()
    sys.exit(app.exec_())