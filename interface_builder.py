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
        ros_tab_layout = QtGui.QGridLayout()
        ros_tab_container.setLayout(ros_tab_layout)
        self.wid_tabs.addTab(ros_tab_container, 'ROS')
        
        self.but_startros = QtGui.QPushButton('Start Node', ros_tab_container)
        self.but_startros.clicked.connect(self.startnode)

        self.but_send = QtGui.QPushButton('Send Polygons', ros_tab_container)
        self.but_send.clicked.connect(self.sendPolys)
        
        self.wid_frame = QtGui.QLineEdit('/base_link', ros_tab_container)
        self.wid_resolution = QtGui.QLineEdit('1', ros_tab_container)
        
        ros_tab_layout.addWidget(self.but_startros,                    0, 0)
        ros_tab_layout.addWidget(QtGui.QLabel('Interface frame id'),   1, 0)
        ros_tab_layout.addWidget(self.wid_frame,                       2, 0)
        ros_tab_layout.addWidget(QtGui.QLabel('Resolution (m/pixel)'), 3, 0)
        ros_tab_layout.addWidget(self.wid_resolution,                  4, 0)
        ros_tab_layout.addWidget(self.but_send,                        5, 0)
        
        layout.addWidget(self.wid_draw, 0, 0, 1, 1)
        
        layout.setColumnMinimumWidth(0, 640)
        
    def sendPolys(self):
        import roslib; roslib.load_manifest('projector_interface')
        import rospy
        from projector_interface.srv import GetCursorStats, DrawPolygon, CircleInhibit
        from geometry_msgs.msg import Point, PolygonStamped
        
        header = rospy.Header()
        header.frame_id = self.wid_frame.text()
        header.stamp = rospy.Time.now()
        
        res = float(self.wid_resolution.text())
        
        for name, qt_poly in self.wid_draw.objects.iteritems():
            ps = PolygonStamped(header=header)
            for pt in qt_poly:
                ps.polygon.points.append(Point(pt.x()*res, pt.y()*res, 1))
            
            print ps
        
    def startnode(self):
        import rospy
        rospy.init_node('interface_builder', anonymous=True)
        self.but_startros.setEnabled(False)
        
    	self.polygon_proxy = rospy.ServiceProxy('/draw_polygon', DrawPolygon)
    	rospy.loginfo("Waiting for polygon service")
    	self.polygon_proxy.wait_for_service()
    	rospy.loginfo("polygon service ready")
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
        
        poly = PySide.QtGui.QPolygon.fromList(self.remove_duplicate_points(self.current_poly))
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