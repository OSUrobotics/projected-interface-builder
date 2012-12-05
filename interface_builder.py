#!/usr/bin/env python
import PySide
from PySide import QtGui, QtCore
from PySide.QtGui import QPalette
from math import hypot

from std_msgs.msg import ColorRGBA

from matplotlib.nxutils import pnpoly
import sys
import numpy as np

class Colors:
    WHITE = ColorRGBA(255,255,255,0)
    RED   = ColorRGBA(255,  0,  0,0)
    GREEN = ColorRGBA(  0,255,  0,0)
    BLUE  = ColorRGBA(  0,  0,255,0)

class Builder(QtGui.QWidget):
    def __init__(self):
        super(Builder, self).__init__()
        self.initUI()
        self.show()        
        
    def initUI(self):
        self.setGeometry(0, 500, 900, 400)
        layout = QtGui.QGridLayout()
        self.setLayout(layout)
        self.wid_draw = DrawWidget()
        self.wid_draw.polygonAdded.connect(self.polygonAdded)
        
        self.but_save = QtGui.QPushButton('Save', self)
        self.but_load = QtGui.QPushButton('Load', self)
        
        self.but_save.clicked.connect(self.save_polygons)
        self.but_load.clicked.connect(self.load_polygons)
        
        # Widgets for the polygon tab
        polygon_tab_container = QtGui.QWidget()
        self.wid_list = QtGui.QListWidget()
        self.wid_list.itemClicked.connect(self.itemClicked)
        
        self.but_delete = QtGui.QPushButton('Delete', polygon_tab_container)
        self.but_delete.clicked.connect(self.deleteClick)
        
        self.wid_name = QtGui.QLineEdit(polygon_tab_container)
        self.wid_name.textChanged[str].connect(self.updateName)
        
        self.wid_tabs = QtGui.QTabWidget(polygon_tab_container)
        layout.addWidget(self.wid_tabs, 0, 1)

        polygon_tab_layout = QtGui.QGridLayout()
        polygon_tab_container.setLayout(polygon_tab_layout)
        self.wid_tabs.addTab(polygon_tab_container, 'Polygons')
        polygon_tab_layout.addWidget(self.wid_list,   0, 0)
        polygon_tab_layout.addWidget(self.but_delete, 1, 0)
        polygon_tab_layout.addWidget(self.wid_name,   2, 0)
        
        #Widgets for the ROS tab
        ros_tab_container = QtGui.QWidget()
        # ros_tab_layout = QtGui.QGridLayout()
        ros_tab_layout = QtGui.QVBoxLayout()
        ros_tab_layout.addStretch(1)
        ros_tab_container.setLayout(ros_tab_layout)
        self.wid_tabs.addTab(ros_tab_container, 'ROS')
        
        self.but_startros = QtGui.QPushButton('Start Node', ros_tab_container)
        self.but_startros.clicked.connect(self.startnode)

        self.but_send = QtGui.QPushButton('Send Polygons', ros_tab_container)
        self.but_send.clicked.connect(self.sendPolys)
        
        self.wid_frame = QtGui.QLineEdit('/base_link', ros_tab_container)
        self.wid_resolution = QtGui.QLineEdit('1', ros_tab_container)
        self.offset_x = QtGui.QLineEdit('0.0', ros_tab_container)
        self.offset_y = QtGui.QLineEdit('0.0', ros_tab_container)
        self.offset_z = QtGui.QLineEdit('0.0', ros_tab_container)
        
        # ros_tab_layout.addWidget(self.but_startros,                    0, 0)
        # ros_tab_layout.addWidget(QtGui.QLabel('Interface frame id'),   1, 0)
        # ros_tab_layout.addWidget(self.wid_frame,                       2, 0)
        # ros_tab_layout.addWidget(QtGui.QLabel('Resolution (m/pixel)'), 3, 0)
        # ros_tab_layout.addWidget(self.wid_resolution,                  4, 0)
        # ros_tab_layout.addWidget(QtGui.QLabel('Z'),                    5, 0)
        # ros_tab_layout.addWidget(self.wid_z,                           6, 0)
        # ros_tab_layout.addWidget(self.but_send,                        7, 0)
        
        ros_tab_layout.addWidget(self.but_startros                   )
        ros_tab_layout.addWidget(QtGui.QLabel('Interface frame id')  )
        ros_tab_layout.addWidget(self.wid_frame                      )
        ros_tab_layout.addWidget(QtGui.QLabel('Resolution (m/pixel)'))
        ros_tab_layout.addWidget(self.wid_resolution                 )
        ros_tab_layout.addWidget(QtGui.QLabel('x offset')            )
        ros_tab_layout.addWidget(self.offset_x                       )
        ros_tab_layout.addWidget(QtGui.QLabel('y offset')            )
        ros_tab_layout.addWidget(self.offset_y                       )
        ros_tab_layout.addWidget(QtGui.QLabel('z offset')            )
        ros_tab_layout.addWidget(self.offset_z                       )
        ros_tab_layout.addWidget(self.but_send                       )
        
        layout.addWidget(self.wid_tabs, 0, 1, 1, 2)
        layout.addWidget(self.wid_draw, 0, 0, 2, 1)
        layout.addWidget(self.but_save, 2, 1)
        layout.addWidget(self.but_load, 2, 2)
        
        
        layout.setColumnMinimumWidth(0, 640)
        
    def save_polygons(self):
        fname, _ = QtGui.QFileDialog.getSaveFileName(self, 'Save') 
        import pickle
        with open(fname, 'w') as f:
            pickle.dump(dict(
                polygons   = self.wid_draw.objects,
                frame_id   = self.wid_frame.text(),
                resolution = self.wid_resolution.text(),
                offset_x   = self.offset_x.text(),
                offset_y   = self.offset_y.text(),
                offset_z   = self.offset_z.text()
            ), f)
            
    def load_polygons(self):
        fname, _ = QtGui.QFileDialog.getOpenFileName(self, 'Load') 
        import pickle
        with open(fname, 'r') as f:
            data = pickle.load(f)
            self.wid_draw.objects = data['polygons']
            self.wid_frame.setText(data['frame_id'])
            self.wid_resolution.setText(data['resolution'])
            self.offset_x.setText(data['offset_x'])
            self.offset_y.setText(data['offset_y'])
            self.offset_z.setText(data['offset_z'])
            
            self.wid_list.clear()
            for name in data['polygons'].keys():
                self.polygonAdded(name)
                    
    def sendPolys(self):
        import rospy
        from geometry_msgs.msg import Point, PolygonStamped

        header = rospy.Header()
        header.frame_id = self.wid_frame.text()
        header.stamp = rospy.Time.now()
        
        res = float(self.wid_resolution.text())
        x = float(self.offset_x.text())
        y = float(self.offset_y.text())
        z = float(self.offset_z.text())
        
        self.polygon_clear_proxy()

        for name, qt_poly in self.wid_draw.objects.iteritems():
            ps = PolygonStamped(header=header)
            for pt in qt_poly:
                ps.polygon.points.append(Point(pt.x()*res+x, pt.y()*res+y, z))
            self.polygon_proxy(name, True, ps, Colors.WHITE)
            self.polygon_viz.publish(ps)
            
            print ps
        
    def startnode(self):
        import roslib; roslib.load_manifest('projector_interface')
        import rospy
        from projector_interface.srv import DrawPolygon, ClearPolygons
        from geometry_msgs.msg import Point, PolygonStamped

        rospy.init_node('interface_builder', anonymous=True)
        self.but_startros.setEnabled(False)
        
    	self.polygon_proxy = rospy.ServiceProxy('/draw_polygon', DrawPolygon)
    	rospy.loginfo("Waiting for polygon service")
    	self.polygon_proxy.wait_for_service()
    	rospy.loginfo("polygon service ready")
        rospy.loginfo("Waiting for polygon clear service")
        self.polygon_clear_proxy = rospy.ServiceProxy('/clear_polygons', ClearPolygons)
        rospy.loginfo("polygon clear service ready")
    	self.polygon_viz = rospy.Publisher('/polygon_viz', PolygonStamped)
        
        
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
        
    def keyPressEvent(self, event):
        if event.key() in (16777248, 16777216):
            self.wid_draw.keyPressEvent(event)
        
    def keyReleaseEvent(self, event):
        if event.key() in (16777248, 16777216):
            self.wid_draw.keyReleaseEvent(event)
        
        
class DrawWidget(QtGui.QWidget):
    objects = dict()
    polygon_active = False
    current_poly = []
    polygonAdded = QtCore.Signal(str)
    active_poly = ''
    snap = False
    axis_align = False
    
    def __init__(self):
        super(DrawWidget, self).__init__()
        self.setGeometry(0, 480, 640, 400)
        timer = PySide.QtCore.QTimer(self)
        timer.setInterval(50)
        timer.timeout.connect(self.update)
        timer.start()

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

    def remove_duplicate_points(self, points):
        newpoints = [points[0]]

        for i in range(1,len(points)):
            if points[i] != points[i-1]:
                newpoints.append(points[i]) 
        
        return newpoints

    def closeTo(self, p1, p2):
        d = hypot(p1.x()-p2.x(), p1.y()-p2.y())
        return d < 10

    def closeToAny(self, p):
        for poly in self.objects.values():
            for v in poly:
                if self.closeTo(v,p):
                    return v
        return None

    def snapToAxis(self, pt):
        xdist = abs(pt.x() - self.current_poly[-1].x())
        ydist = abs(pt.y() - self.current_poly[-1].y())
        if xdist < ydist:
            return QtCore.QPoint(self.current_poly[-1].x(), pt.y())
        else:
            return QtCore.QPoint(pt.x(), self.current_poly[-1].y())

    def mousePressEvent(self, event):
        self.setMouseTracking(True)
        if self.snap and not self.otherSnap:
            self.mouseDoubleClickEvent(QtGui.QMouseEvent(
                event.type(),
                self.snapPos,
                event.button(),
                event.buttons(),
                event.modifiers(),
            ))
            return
        if (event.button() == QtCore.Qt.MouseButton.LeftButton) and (self.polygon_active):
            if self.axis_align:
                pos = self.snapPos
            else:
                pos = event.pos()
            if len(self.current_poly) == 1:
                self.current_poly[0][1] = (pos.x(), pos.y())
            else:
                self.current_poly.extend([self.current_poly[-1],pos])
        elif (event.button() == QtCore.Qt.MouseButton.LeftButton) and (not self.polygon_active):
            self.current_poly.extend([event.pos(),event.pos()])
            self.cursorx = event.x()
            self.cursory = event.y()
            self.polygon_active = True
               
    def mouseDoubleClickEvent(self, event):
        if len(self.current_poly) == 1:
            self.current_poly[0][1] = (event.x(), event.y())
        else:
            self.current_poly.extend([self.current_poly[-1],event.pos()])        
        
        poly = PySide.QtGui.QPolygon.fromList(self.remove_duplicate_points(self.current_poly))
        poly_name = self.generate_name()
        self.objects[poly_name] = QtGui.QPolygon(poly)
        self.polygonAdded.emit(poly_name)
        self.polygon_active = False
        self.current_poly = []        
        self.snap = False
                
    def mouseMoveEvent(self, event):
        self.cursorx = event.x()
        self.cursory = event.y()

    def keyPressEvent(self, event):
        if event.key() == 16777248:
            self.axis_align = True
        elif event.key() == 16777216:
            self.polygon_active = False
            self.current_poly = []
        
    def keyReleaseEvent(self, event):
        if event.key() == 16777248:
            self.axis_align = False

    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)
        qp.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        if self.polygon_active:
            cursor = QtCore.QPoint(self.cursorx, self.cursory)
            if self.axis_align:
                cursor = self.snapToAxis(cursor)
                self.snapPos = cursor
            qp.drawLines(self.current_poly)
            self.snap = False
            for p in self.current_poly:
                if self.closeTo(cursor, p):
                    qp.drawLine(self.current_poly[-1], p)
                    self.snap = True
                    self.snapPos = p
                    break
            self.otherSnap = self.closeToAny(cursor)
            if self.otherSnap is not None:
                qp.drawLine(self.current_poly[-1], self.otherSnap)
                self.snap = True
                self.snapPos = self.otherSnap
            if not self.snap:
                qp.drawLine(self.current_poly[-1], cursor)
            
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
            
            # qp.drawText(obj.boundingRect(), QtCore.Qt.AlignCenter, name)
            qp.drawText(np.mean(obj), name)
            
        qp.end()

if __name__ == '__main__':
    app = PySide.QtGui.QApplication(sys.argv)
    gui = Builder()
    sys.exit(app.exec_())