#!/usr/bin/env python
import PySide
from PySide import QtGui, QtCore
from PySide.QtGui import QPalette

from matplotlib.nxutils import pnpoly
import sys

class Builder(QtGui.QWidget):
    def __init__(self):
        super(Builder, self).__init__()
        self.initUI()
        
        self.show()
        # import pdb; pdb.set_trace()
        
        
    def initUI(self):
        self.setGeometry(0, 480, 800, 400)
        # layout = QtGui.QVBoxLayout()
        # layout.setDirection(QtGui.QVBoxLayout.Direction.LeftToRight)
        layout = QtGui.QGridLayout()
        self.setLayout(layout)
        self.wid_draw = DrawWidget()
        self.wid_draw.polygonAdded.connect(self.polygonAdded)
        
        self.wid_list = QtGui.QListWidget()
        self.wid_list.itemClicked.connect(self.itemClicked)
        # self.wid_list.itemDoubleClicked.connect(self.itemDoubleClicked)
        
        self.but_delete = QtGui.QPushButton('Delete', self)
        self.but_delete.clicked.connect(self.deleteClick)
        
        self.wid_name = QtGui.QLineEdit(self)
        self.wid_name.textChanged[str].connect(self.updateName)
        
        layout.addWidget(self.wid_draw, 0, 0, 3, 1)
        layout.addWidget(self.wid_list, 0, 1)
        layout.addWidget(self.but_delete, 1, 1)
        layout.addWidget(self.wid_name, 2, 1)
        layout.setColumnMinimumWidth(0, 640)
        
    def deleteClick(self):
        self.wid_draw.removeObject(self.wid_list.currentItem().text())
        self.wid_list.takeItem(self.wid_list.currentRow())
        
    def polygonAdded(self, name):
        self.wid_list.addItem(name)
        
    def itemClicked(self, item):
        self.wid_draw.setActive(item.text())
        self.wid_name.setText(item.text())
        
    def updateName(self, text):
        old_name = self.wid_list.currentItem().text()
        self.wid_list.currentItem().setText(text)
        self.wid_draw.updateName(old_name, text)
        
    # def itemDoubleClicked(self, item):
    #     self.wid_list.editItem(item)
        
class DrawWidget(QtGui.QWidget):
    objects = dict()
    polygon_active = False
    current_poly = []
    polygonAdded = QtCore.Signal(str)
    active_poly = ''
    
    def __init__(self):
        super(DrawWidget, self).__init__()
        self.setGeometry(0, 480, 640, 400)
        timer = PySide.QtCore.QTimer(self)
        timer.setInterval(100)
        timer.timeout.connect(self.update)
        timer.start()
        # self.show()

    def updateName(self, oldName, newName):
        if oldName != newName:
            self.objects[newName] = self.objects[oldName]
            self.removeObject(oldName)
            self.setActive(newName)

    def removeObject(self, name):
        del self.objects[name]

    def setActive(self, name):
        self.active_poly = name

    def generate_name(self):
        return 'Polygon %s' % len(self.objects)

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
        poly_name = self.generate_name()
        self.objects[poly_name] = QtGui.QPolygon(poly)
        self.polygonAdded.emit(poly_name)
        self.polygon_active = False
        self.current_poly = []        
                
    def mouseMoveEvent(self, event):
        pass

    def mouseHoverEvent(self, event):
        print event

    def paintEvent(self, e):
        # cursor = (self.cursor().pos().x()-self.pos().x(), self.cursor().pos().y()-self.pos().y())
        qp = QtGui.QPainter()
        qp.begin(self)
        qp.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        if self.polygon_active:
            qp.drawLines(self.current_poly)
        for name, obj in self.objects.iteritems():
            # active = pnpoly(cursor[0], cursor[1], [(p.x(), p.y()) for p in obj])
            if self.active_poly == name:
                pen = qp.pen()
                pen.setWidth(3)
                qp.setPen(pen)
            qp.drawPolygon(obj)
            pen = qp.pen()
            pen.setWidth(1)
            qp.setPen(pen)
            
            qp.drawText(obj.boundingRect(), QtCore.Qt.AlignCenter, name)
            
        qp.end()

if __name__ == '__main__':
    app = PySide.QtGui.QApplication(sys.argv)
    gui = Builder()
    sys.exit(app.exec_())