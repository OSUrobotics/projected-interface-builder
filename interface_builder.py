#!/usr/bin/env python
from __future__ import division

import roslib; roslib.load_manifest('projected_interface_builder')
import rospy
import PySide
from PySide import QtGui, QtCore
from PySide.QtGui import QPalette
from math import hypot

from std_msgs.msg import ColorRGBA

from matplotlib.nxutils import pnpoly
import sys
import numpy as np

from projected_interface_builder.data_structures import PolygonInfo
from projected_interface_builder import colors


FONT = QtGui.QFont('Decorative', 30)

class BuilderWindow(QtGui.QMainWindow):
    def __init__(self):
        super(BuilderWindow, self).__init__()
        self.setWindowTitle('Interface Builder')
        self.builder = Builder()
        self.setCentralWidget(self.builder)
        self.statusBar()
        self.show()        

    def load_polygons(self, path):
        self.builder.load_polygons(path)

class Builder(QtGui.QWidget):
    draw_mode = ' '
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
        self.wid_draw.mouseMoved.connect(self.mouseMoved)
        self.wid_draw.modeUpdate.connect(self.modeUpdate)
        
        self.but_save = QtGui.QPushButton('Save', self)
        self.but_load = QtGui.QPushButton('Load', self)
        
        self.but_save.clicked.connect(self.save_polygons)
        self.but_load.clicked.connect(self.load_polygons_click)
        
        # Widgets for the polygon tab
        polygon_tab_container = QtGui.QWidget()
        self.wid_list = QtGui.QListWidget()
        # self.wid_list.itemClicked.connect(self.itemClicked)
        self.wid_list.itemSelectionChanged.connect(self.listItemSelectionChanged)
        
        self.but_delete = QtGui.QPushButton('Delete', polygon_tab_container)
        self.but_delete.clicked.connect(self.deleteClick)
        
        # self.wid_name = QtGui.QLineEdit(polygon_tab_container)
        # self.wid_name.textChanged[str].connect(self.updateName)
        
        self.wid_tabs = QtGui.QTabWidget(polygon_tab_container)
        layout.addWidget(self.wid_tabs, 0, 1)

        self.wid_edit = QtGui.QTableWidget(2, 1, polygon_tab_container)
        self.wid_edit.setVerticalHeaderLabels(['Name', 'ID'])
        self.wid_edit.cellChanged.connect(self.cellChanged)
        self.wid_edit.itemSelectionChanged.connect(self.itemSelectionChanged)
        orig_press = self.wid_edit.keyPressEvent
        def new_press(event):
            if event.key() == QtCore.Qt.Key.Key_Delete:
                # remove the point from the underlying polygon, then reload
                if self.wid_edit.currentRow() > 1:
                    poly_info = self.wid_draw.objects[self.wid_list.currentItem().text()]
                    poly_info.polygon.remove(self.wid_edit.currentRow()-2)
                    poly_info.update_item()
                    # import pdb; pdb.set_trace()
                    # poly_info.gfx_item
                    self.wid_draw.clear_active_point()
                    self.listItemSelectionChanged()
            orig_press(event)
        self.wid_edit.keyPressEvent = new_press

        polygon_tab_layout = QtGui.QGridLayout()
        polygon_tab_container.setLayout(polygon_tab_layout)
        self.wid_tabs.addTab(polygon_tab_container, 'Polygons')
        polygon_tab_layout.addWidget(self.wid_list,   0, 0)
        # polygon_tab_layout.addWidget(self.wid_name,   1, 0)
        polygon_tab_layout.addWidget(self.wid_edit,   1, 0)
        polygon_tab_layout.addWidget(self.but_delete, 2, 0)
        
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
        self.but_send.setEnabled(False)
        
        self.wid_frame = QtGui.QLineEdit('/base_link', ros_tab_container)
        self.wid_resolution = QtGui.QLineEdit('1', ros_tab_container)
        self.offset_x = QtGui.QLineEdit('0.0', ros_tab_container)
        self.offset_y = QtGui.QLineEdit('0.0', ros_tab_container)
        self.offset_z = QtGui.QLineEdit('0.0', ros_tab_container)
                
        self.wid_resolution.textChanged[str].connect(self.resolutionChanged)

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
        
        layout.addWidget(self.wid_tabs, 0, 1, 2, 2)
        layout.addWidget(self.wid_draw, 0, 0, 3, 1)
        layout.addWidget(self.but_save, 2, 1)
        layout.addWidget(self.but_load, 2, 2)
        
        layout.setColumnMinimumWidth(0, 640)
        
    def save_polygons(self):
        fname, _ = QtGui.QFileDialog.getSaveFileName(self, 'Save')
        if len(fname) == 0: return
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
            
    def load_polygons_click(self):
        fname, _ = QtGui.QFileDialog.getOpenFileName(self, 'Load')
        self.load_polygons(fname)

    def load_polygons(self, fname):
        import pickle
        if len(fname) == 0: return
        with open(fname, 'r') as f:
            data = pickle.load(f)
            if type(data['polygons'].values()[0]) == PySide.QtGui.QPolygon: #enables loading old data
                seq = 0
                for name, poly in data['polygons'].iteritems():
                    poly_info = PolygonInfo(poly, name=name, uid='poly%s' % seq)
                    # self.wid_draw.objects[poly_info.id] = poly_info
                    self.wid_draw.add_polygon(poly_info)
                    seq += 1
            else:        
                self.wid_draw.objects = data['polygons']
            self.wid_frame.setText(data['frame_id'])
            self.wid_resolution.setText(str(data['resolution']))
            self.offset_x.setText(str(data['offset_x']))
            self.offset_y.setText(str(data['offset_y']))
            self.offset_z.setText(str(data['offset_z']))
            
            self.wid_list.clear()

            for name in sorted(self.wid_draw.objects.keys()):
                self.polygonAdded(name)
                    
    def sendPolys(self):
        from projected_interface_builder.convert_utils import QtPolyToROS, QtRectToPoly, toMarker
        from visualization_msgs.msg import MarkerArray
        
        res = float(self.wid_resolution.text())
        x = float(self.offset_x.text())
        y = float(self.offset_y.text())
        z = float(self.offset_z.text())
        
        self.polygon_clear_proxy()

        markers = MarkerArray()
        for uid, poly_info in self.wid_draw.objects.iteritems():
            ps = QtPolyToROS(poly_info.polygon, uid, x, y, z, res, self.wid_frame.text())
            ps.header.stamp = rospy.Time.now()
            text_rect = QtPolyToROS(QtRectToPoly(poly_info.text_rect), '', x, y, z, res, self.wid_frame.text())
            print ps
            self.polygon_proxy(uid, poly_info.name, ps, text_rect.polygon, colors.WHITE)
            markers.markers.append(toMarker(ps, np.int32(hash(uid))))
        self.polygon_viz.publish(markers)
        
    def startnode(self):
        self.but_send.setEnabled(True)
        from projector_interface.srv import DrawPolygon, ClearPolygons
        from geometry_msgs.msg import Point, PolygonStamped
        from visualization_msgs.msg import MarkerArray

        rospy.init_node('interface_builder', anonymous=True)
        self.but_startros.setEnabled(False)
        
        self.polygon_proxy = rospy.ServiceProxy('/draw_polygon', DrawPolygon)
        rospy.loginfo("Waiting for polygon service")
        self.polygon_proxy.wait_for_service()
        rospy.loginfo("polygon service ready")
        rospy.loginfo("Waiting for polygon clear service")
        self.polygon_clear_proxy = rospy.ServiceProxy('/clear_polygons', ClearPolygons)
        rospy.loginfo("polygon clear service ready")
        self.polygon_viz = rospy.Publisher('/polygon_viz', MarkerArray)

        self.but_startros.setText('Node Running...')
        
    def resolutionChanged(self, text):
        try:
            FONT.setPointSize(30-float(text)*12000)
        except Exception:
            pass
        
    def deleteClick(self):
        self.wid_draw.removeObject(self.wid_list.currentItem().text())
        self.wid_list.takeItem(self.wid_list.currentRow())
        self.wid_draw.clear_active_point()
        
    def polygonAdded(self, name):
        self.wid_list.addItem(name)
        
    def mouseMoved(self, location):
        self.window().statusBar().showMessage('%s x=%s, y=%s' % (self.draw_mode, location.x(), location.y()))

    def modeUpdate(self, mode):
        self.draw_mode = mode

    def listItemSelectionChanged(self):
        item = self.wid_list.currentItem()
        poly = self.wid_draw.objects[item.text()]
        self.wid_draw.clear_active_point()
        self.wid_draw.setActive(item.text())
        self.wid_edit.setItem(0, 0, QtGui.QTableWidgetItem(poly.name))
        self.wid_edit.setItem(1, 0, QtGui.QTableWidgetItem(poly.id))
        points = poly.polygon
        self.wid_edit.setRowCount(2+len(points))
        for px, point in enumerate(points):
            self.wid_edit.setItem(px+2, 0, QtGui.QTableWidgetItem('(%s, %s)' % (point.x(), point.y())))
        
    def updateName(self, text):
        text = text.replace('\\n', '\n')
        self.wid_draw.updateName(self.wid_list.currentItem().text(), text)

    def updateId(self, text):
        old_id = self.wid_list.currentItem().text()

        self.wid_draw.updateId(old_id, text)
        self.wid_list.currentItem().setText(text)
        
    def keyPressEvent(self, event):
        if event.key() in (16777248, 16777216):
            self.wid_draw.keyPressEvent(event)
        
    def keyReleaseEvent(self, event):
        if event.key() in (16777248, 16777216):
            self.wid_draw.keyReleaseEvent(event)
    
    def cellChanged(self, row, col):
        item = self.wid_edit.item(row, col)
        if row == 0:
            self.updateName(item.text())
        elif row == 1:
            self.updateId(item.text())
        else:
            try:
                # print self
                exec('x,y=%s' % item.text())
                self.wid_draw.updatePoint(self.wid_list.currentItem().text(), row-2, x, y)
                # self.wid_draw.update_active_point(QtCore.QPoint(x,y))
                self.wid_draw.clear_active_point()
            except Exception, e:
                print e
                
    # def cellPressed(self, row, col):
    def itemSelectionChanged(self):
        item = self.wid_edit.currentItem()
        if self.wid_edit.currentRow() > 1:
            try:
                exec('x,y=%s' % item.text())
                self.wid_draw.update_active_point(QtCore.QPoint(x,y))
            except Exception, e:
                print e
                 
class DrawWidget(QtGui.QGraphicsView):
    objects = dict()
    polygon_active = False
    current_poly = []
    polygonAdded = QtCore.Signal(str)
    mouseMoved = QtCore.Signal(QtCore.QPointF)
    modeUpdate = QtCore.Signal(str)
    active_poly = ''
    snap = False
    axis_align = False
    active_point = None
    cursorx, cursory = 0, 0
    text_move = False
    scale = 1.0
    trans = QtCore.QPoint()
    drag_start = False
    otherSnap = None
    last_click = None
    snap_to_grid = False
    POLYGON_PEN      = QtGui.QPen(QtGui.QColor(128,128,128))
    GRID_PEN         = QtGui.QPen(QtGui.QColor( 25, 25, 25))
    ACTIVE_POINT_PEN = QtGui.QPen(QtGui.QColor(  0,255,  0), 4)
    # ACTIVE_PEN  = QtGui.QPen(color=QtGui.QColor(128,128,128), width=3)

    def __init__(self):
        super(DrawWidget, self).__init__()
        self.scene = QtGui.QGraphicsScene()
        self.scene.setBackgroundBrush(QtGui.QColor(0,0,0))
        self.setScene(self.scene)
        self.draw_grid_lines()
    
    def add_polygon(self, poly_info):
        poly_info.set_item(self.scene.addPolygon(poly_info.polygon, pen=self.POLYGON_PEN))
        text_item = self.scene.addText(poly_info.name)
        text_item.setPos(poly_info.text_rect.topLeft())
        text_item.setFont(FONT)
        poly_info.set_text_item(text_item)
        self.objects[poly_info.id] = poly_info

    def update_active_point(self, point):
        self.clear_active_point()
        self.active_point = self.scene.addEllipse(QtCore.QRect(point-QtCore.QPoint(1,1), point+QtCore.QPoint(1,1)), self.ACTIVE_POINT_PEN)

    def clear_active_point(self):
        if self.active_point:
            self.scene.removeItem(self.active_point)

    def updateName(self, pid, newName):
        self.objects[pid].name = newName
        self.objects[pid].update_font_box()

    def updateId(self, oldId, newId):
        if oldId != newId:
            self.objects[newId] = self.objects[oldId]
            self.objects[newId].id = newId
            self.removeObject(oldId)
        self.setActive(newId)

    def updatePoint(self, uid, point_index, x, y):
        # TODO update the item's poly as well
        pt = self.objects[uid].polygon[point_index]
        pt.setX(x)
        pt.setY(y)
        self.objects[uid].polygon.replace(point_index, pt)

    def removeObject(self, name):
        obj = self.objects[name]
        self.scene.removeItem(obj.gfx_item)
        self.scene.removeItem(obj.text_item)
        del obj

    def setActive(self, name):
        self.active_poly = name

    def generate_name(self):
        return 'Polygon %s' % len(self.objects)

    def remove_duplicate_points(self, points):
        newpoints = []
        for i in range(len(points)):
            if points[i] != points[i-1]:
                newpoints.append(points[i]) 
        
        return newpoints

    def closeTo(self, p1, p2):
        d = hypot(p1.x()-p2.x(), p1.y()-p2.y())
        return d < 10

    def closeToAny(self, p):
        for poly_info in self.objects.values():
            for v in poly_info.polygon:
                if self.closeTo(v,p):
                    return v
        return None

    def closeToCurrent(self, p):
        for line in self.current_poly[:-1]:
            p1 = line.line().p1()
            if self.closeTo(p1, p): return p1
            p2 = line.line().p2()
            if self.closeTo(p2, p): return p2

        return None

    def snapToAxis(self, pt):
        xdist = abs(pt.x() - self.current_poly[-1].line().p1().x())
        ydist = abs(pt.y() - self.current_poly[-1].line().p1().y())
        if xdist < ydist:
            return QtCore.QPoint(self.current_poly[-1].line().p1().x(), pt.y())
        else:
            return QtCore.QPoint(pt.x(), self.current_poly[-1].line().p1().y())

    def do_polygon_click(self, event):
        cursor = self.mapToScene(event.pos())
        if self.snap and not self.otherSnap:
            self.mouseDoubleClickEvent(QtGui.QMouseEvent(
                event.type(),
                self.snapPos,
                event.button(),
                event.buttons(),
                event.modifiers(),
            ))
            return

        # Create the line connecting to the cursor
        if (event.button() == QtCore.Qt.MouseButton.LeftButton) and (self.polygon_active):
            # If this isn't the first point in the polygon
            self.reset_active_line(self.get_line_endpoint(cursor))
        elif (event.button() == QtCore.Qt.MouseButton.LeftButton) and (not self.polygon_active):
            # If this is the first point in the polygon
            pt = self.closeToAny(cursor)
            if pt is not None:
                self.last_click = pt

            else:
                self.last_click = cursor
            self.reset_active_line(self.last_click)

            self.cursorx = cursor.x()
            self.cursory = cursor.y()
            self.polygon_active = True

    def reset_active_line(self, pos):
        self.active_line = QtGui.QGraphicsLineItem(QtCore.QLineF(pos, pos))
        self.active_line.setPen(self.POLYGON_PEN)
        self.scene.addItem(self.active_line)
        self.current_poly.append(self.active_line)

    def get_line_endpoint(self, pos):
        if self.axis_align:
            return self.snapToAxis(pos)
        cta = self.closeToAny(pos)
        if cta: return cta
        cts = self.closeToCurrent(pos)
        if cts: return cts
        return pos

    def do_text_click(self, event):
        if self.text_move:
            self.text_move = False
        else:
            self.text_move = True
            self.text_rect_orig = self.objects[self.active_poly].text_rect.translated(QtCore.QPoint())

    def mousePressEvent(self, event):
        self.setMouseTracking(True)
        if event.button() == QtCore.Qt.MouseButton.MiddleButton:
            # subtracting out self.trans makes sure we take into account 
            # the previous translation rather than just using the delta
            self.drag_start = (event.pos()/self.scale - self.trans)
        elif self.active_poly and self.objects[self.active_poly].in_text_box((self.cursorx, self.cursory)):
            self.do_text_click(event)
        else:
            self.do_polygon_click(event)
        self.last_click = self.mapToScene(event.pos())

    def mouseReleaseEvent(self, event):
        self.drag_start = False
        
    def mouseDoubleClickEvent(self, event):        
        # polygon-ize the line list
        poly = PySide.QtGui.QPolygon.fromList([l.line().p1().toPoint() for l in self.current_poly])
        poly_container = PolygonInfo(QtGui.QPolygon(poly), name=self.generate_name())
        self.add_polygon(poly_container)
        self.polygonAdded.emit(poly_container.id)
        self.polygon_active = False
        self.remove_active_poly_items()
        self.snap = False
        self.last_click = None
                
    def mouseMoveEvent(self, event):
        pos = self.mapToScene(event.pos())
        self.cursorx = pos.x()
        self.cursory = pos.y()
        self.mouseMoved.emit(pos)
        if event.buttons() & QtCore.Qt.MouseButton.MiddleButton:
            if self.drag_start:
                trans = (pos/self.scale - self.drag_start)
                self.translate(trans.x(), trans.y())

        elif self.text_move:
            self.objects[self.active_poly].text_rect.moveCenter(QtCore.QPoint(self.cursorx, self.cursory))
        if self.polygon_active:
            # Note: line() returns a *copy* of the backing line, so changing it doesn't do anything
            line = self.active_line.line()
            line.setP2(self.get_line_endpoint(pos))
            self.active_line.setLine(line)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Shift:
            self.axis_align = True
            self.modeUpdate.emit('|')
        elif event.key() == QtCore.Qt.Key.Key_Escape:
            if self.text_move:
                self.text_move = False
                self.objects[self.active_poly].text_rect = self.text_rect_orig
            self.polygon_active = False
            self.remove_active_poly_items()
            
            self.snap = False
        elif event.key() ==  PySide.QtCore.Qt.Key.Key_Control:
            self.snap_to_grid = True
            self.modeUpdate.emit('#')

    def wheelEvent(self, event):
        mul = 1.0
        if event.delta() < 0:
            mul *= 0.9
        if event.delta() > 0:
            mul *= 1.1

        self.scale *= mul
        pos = self.mapToScene(event.pos())
        self.trans = -(pos*self.scale - pos)
        
        
    def keyReleaseEvent(self, event):
        if event.key() == QtCore.Qt.Key_Shift:
            self.axis_align = False
        elif event.key() ==  PySide.QtCore.Qt.Key.Key_Control:
            self.snap_to_grid = False
        self.modeUpdate.emit(' ')

    def remove_active_poly_items(self):
        for line in self.current_poly:
            line.scene().removeItem(line)
            self.current_poly = []

    def draw_grid_lines(self):
        '''
        Draw a grid inside @rect
        '''
        lines = []
        step = 20
        def round_close(n):
            return int(step*round(n/step))

        top    = -1000
        bottom =  1000
        left   = -1000
        right  =  1000
        for inc in range(left, right, step):
            inc = round_close(inc)
            self.scene.addLine(QtCore.QLine(QtCore.QPoint(inc, top), QtCore.QPoint(inc, bottom)), self.GRID_PEN)
        for inc in range(top, bottom, step):
            inc = round_close(inc)
            self.scene.addLine(QtCore.QLine(QtCore.QPoint(left, inc), QtCore.QPoint(right, inc)), self.GRID_PEN)

if __name__ == '__main__':
    app = PySide.QtGui.QApplication(sys.argv)
    gui = BuilderWindow()
    if len(rospy.myargv()) == 2:
        gui.load_polygons(rospy.myargv()[1])
    sys.exit(app.exec_())