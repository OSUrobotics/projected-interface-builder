#!/usr/bin/env python
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

from __future__ import division

PKG_NAME = 'projected_interface_builder'
import roslib; roslib.load_manifest(PKG_NAME)
import rospy, rospkg
from PySide import QtGui, QtCore
from math import hypot
from pr2_python import pointclouds

import sys, argparse
import numpy as np

from functools import partial

from projected_interface_builder.data_structures import PolygonInfo
from projected_interface_builder import colors, modes

import os

FONT = QtGui.QFont('Decorative', 30)
RULER_FONT = QtGui.QFont('Decorative', 12)

MOUSE_MODE_EDIT = 0
MOUSE_MODE_TEST = 1

MODE_AXIS_ALIGN = '|'
MODE_SNAP_TO_GRID = '#'
MODE_NONE = ' '

class BuilderWindow(QtGui.QMainWindow):
    def __init__(self, savefile=None, standalone=False):
        super(BuilderWindow, self).__init__()
        self.rp = rospkg.RosPack()
        self._setWindowTitle()
        self.builder = Builder(standalone)
        self.builder.fileLoaded.connect(self._handleFileLoaded)
        self.builder.fileSaved.connect(self._handleFileSaved)
        self.builder.interfaceChanged.connect(self._handleInterfaceChanged)
        self.builder.wid_draw.modeUpdate.connect(self._updateDrawMode)
        self.setCentralWidget(self.builder)
        self.statusBar()
        self.file_loaded = False
        self.unsaved_changes = False
        self.setupToolbar()

        if savefile:
            self.load_polygons(savefile)
            self.file_loaded = True

        self.show()

    def closeEvent(self, event):
        if self.unsaved_changes:
            save = QtGui.QMessageBox.question(
                self,
                'Unsaved Changes',
                'You have unsaved changes. Save?',
                QtGui.QMessageBox.Yes | QtGui.QMessageBox.No, QtGui.QMessageBox.Yes
            )
            if save == QtGui.QMessageBox.Yes:
                self.builder.save_polygons(self.builder.savefile)
        QtGui.QMainWindow.closeEvent(self, event)

    def _updateDrawMode(self, mode):
        self.act_sta.setChecked(False)
        self.act_stg.setChecked(False)

        if mode == MODE_AXIS_ALIGN:
            self.act_sta.setChecked(True)
        elif mode == MODE_SNAP_TO_GRID:
            self.act_stg.setChecked(True)

    def _get_icon(self, name):
        path = os.path.join(self.rp.get_path('projected_interface_builder'), 'icons', '%s.svg' % name)
        return QtGui.QIcon(path)

    def setupToolbar(self):
        self.toolbar = self.addToolBar('Toolbar')
        self.act_open = self.toolbar.addAction(QtGui.QIcon.fromTheme('document-open'), 'Open')
        self.act_save = self.toolbar.addAction(QtGui.QIcon.fromTheme('document-save'), 'Save')
        self.act_savea = self.toolbar.addAction(QtGui.QIcon.fromTheme('document-save-as'), 'Save As')
        self.toolbar.addSeparator()
        self.act_stg = self.toolbar.addAction(self._get_icon('grid'), 'Snap to Grid (Shift)')
        self.act_sta = self.toolbar.addAction(self._get_icon('axis'), 'Snap to Axis (Ctrl)')
        self.toolbar.addSeparator()
        self.act_add_rect  = self.toolbar.addAction(self._get_icon('rect'), 'Add Rectangle (F1)')
        self.act_add_poly  = self.toolbar.addAction(self._get_icon('poly'), 'Add Polygon (F2)')
        # self.act_add_line = self.toolbar.addAction(self._get_icon('point'),'Add Points (F3)')
        self.toolbar.addSeparator()
        self.act_sel_obj = self.toolbar.addAction(self._get_icon('select_obj'),'Select Object (F4)')
        self.act_sel_pt = self.toolbar.addAction(self._get_icon('select_pt'),  'Select Point (F5)')
        self.act_sel_txt = self.toolbar.addAction(QtGui.QIcon.fromTheme('format-text-italic'),  'Select Text (F6)')
        self.toolbar.addSeparator()
        self.act_help = self.toolbar.addAction(QtGui.QIcon.fromTheme('help-contents'), 'Help')

        # action groups
        group_snap = QtGui.QActionGroup(self)
        group_snap.addAction(self.act_stg)
        group_snap.addAction(self.act_sta)

        group_ins_sel = QtGui.QActionGroup(self)
        group_ins_sel.addAction(self.act_add_rect)
        group_ins_sel.addAction(self.act_add_poly)
        # group_ins_sel.addAction(self.act_add_line)
        group_ins_sel.addAction(self.act_sel_obj)
        group_ins_sel.addAction(self.act_sel_pt)
        group_ins_sel.addAction(self.act_sel_txt)

        # set action shortcuts
        self.act_add_rect.setShortcut('F1')
        self.act_add_poly.setShortcut('F2')
        # self.act_add_line.setShortcut('F3')
        self.act_sel_obj.setShortcut('F4')
        self.act_sel_pt.setShortcut('F5')
        self.act_sel_txt.setShortcut('F6')
        self.act_open.setShortcut(QtGui.QKeySequence.Open)
        self.act_save.setShortcut(QtGui.QKeySequence.Save)
        self.act_savea.setShortcut(QtGui.QKeySequence.SaveAs)


        self.act_stg.setCheckable(True)
        self.act_sta.setCheckable(True)

        self.act_add_rect.setCheckable(True)
        self.act_add_poly.setCheckable(True)
        # self.act_add_line.setCheckable(True)


        self.act_add_poly.triggered.connect(self.builder.wid_draw.setDrawModePolygon)
        self.act_add_rect.triggered.connect(self.builder.wid_draw.setDrawModeRect)


        self.act_sel_obj.setCheckable(True)
        self.act_sel_obj.triggered.connect(self.builder.wid_draw.setEditModeObject)

        self.act_sel_pt.setCheckable(True)
        self.act_sel_pt.triggered.connect(self.builder.wid_draw.setEditModePoint)

        self.act_sel_txt.setCheckable(True)
        self.act_sel_txt.triggered.connect(self.builder.wid_draw.setEditModeText)

        self.act_open.triggered.connect(self.builder.load_polygons_click)

        self.act_save.triggered.connect(self._save)

        self.act_savea.triggered.connect(self._save_as)

        self.act_savea.triggered.connect(self.builder.save_polygons)
        url = roslib.manifest.parse_file(roslib.manifest.manifest_file(PKG_NAME)).url
        self.act_help.triggered.connect(lambda : QtGui.QDesktopServices.openUrl(url))
        # import pdb; pdb.set_trace()

    def _save(self):
        self.builder.save_polygons(fname=self.builder.savefile)

    def _save_as(self):
        self.builder.save_polygons(fname=None)

    def _handleFileSaved(self, filename):
        self.file_loaded = True
        self.unsaved_changes = False
        self._setWindowTitle(filename)

    def _handleFileLoaded(self, filename):
        self.file_loaded = True
        self.unsaved_changes = False
        self._setWindowTitle(filename)

    def _handleInterfaceChanged(self):
        self.unsaved_changes = True
        if self.file_loaded:
            title = self.windowTitle()
            if not title.endswith('*'):
                self.setWindowTitle(self.windowTitle() + ' *')

    def _setWindowTitle(self, filename=''):
        title = 'Interface Builder'
        if filename:
            title += ': ' + filename
        self.setWindowTitle(title)

    def load_polygons(self, path):
        self.builder.load_polygons(path)

class Builder(QtGui.QWidget):
    fileLoaded       = QtCore.Signal(str)
    fileSaved        = QtCore.Signal(str)
    interfaceChanged = QtCore.Signal()
    interfaceSaved   = QtCore.Signal()
    draw_mode = ' '
    def __init__(self, standalone=False):
        super(Builder, self).__init__()
        self._standalone = standalone
        self.initUI()
        self.show()
        self._changePoly = False
        self.savefile = None
        
    def initUI(self):
        self.RES = 0.001
        self.setGeometry(0, 500, 900, 400)
        layout = QtGui.QGridLayout()
        self.setLayout(layout)
        self.wid_draw = DrawWidget()
        self.wid_draw.polygonAdded.connect(self.polygonAdded)
        self.wid_draw.mouseMoved.connect(self.mouseMoved)
        self.wid_draw.mouseClick.connect(self.mouseClick)
        self.wid_draw.modeUpdate.connect(self.modeUpdate)
        self.wid_draw.set_resolution(self.RES)

        # self.but_save = QtGui.QPushButton('Save', self)
        # self.but_load = QtGui.QPushButton('Load', self)
        
        # self.but_save.clicked.connect(self.save_polygons)
        # self.but_load.clicked.connect(self.load_polygons_click)
        
        # Widgets for the polygon tab
        polygon_tab_container = QtGui.QWidget()
        self.wid_list = QtGui.QListWidget()
        self.wid_list.itemSelectionChanged.connect(self.listItemSelectionChanged)
        
        self.but_delete = QtGui.QPushButton('Delete Polygon', polygon_tab_container)
        self.but_delete.clicked.connect(self.deleteClick)
        
        self.wid_tabs = QtGui.QTabWidget(polygon_tab_container)
        self.wid_tabs.setMinimumWidth(200)
        layout.addWidget(self.wid_tabs, 0, 1)

        self.wid_tabs.currentChanged.connect(self.wid_draw.setMouseMode)

        self.wid_edit = QtGui.QTableWidget(2, 1, polygon_tab_container)
        # self.wid_edit.setColumnMinimumWidth
        self.wid_edit.setVerticalHeaderLabels(['Name', 'ID'])
        self.wid_edit.setColumnWidth(0, 150)
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
                    self.wid_draw.clear_active_point()
                    self.listItemSelectionChanged()
            orig_press(event)
        self.wid_edit.keyPressEvent = new_press

        polygon_tab_layout = QtGui.QGridLayout()
        polygon_tab_container.setLayout(polygon_tab_layout)
        self.wid_tabs.addTab(polygon_tab_container, 'Polygons')
        polygon_tab_layout.addWidget(self.wid_list,   0, 0)
        polygon_tab_layout.addWidget(self.wid_edit,   1, 0)
        polygon_tab_layout.addWidget(self.but_delete, 2, 0)
        
        #Widgets for the ROS tab
        ros_tab_container = QtGui.QWidget()
        ros_tab_layout = QtGui.QVBoxLayout()
        ros_tab_layout.addStretch(1)
        ros_tab_container.setLayout(ros_tab_layout)
        self.wid_tabs.addTab(ros_tab_container, 'Test')
        
        self.but_startros = QtGui.QPushButton('Start Node', ros_tab_container)
        self.but_startros.clicked.connect(self.startnode)

        self.radio_z_dir_towards = QtGui.QRadioButton('Z Points Towards', ros_tab_container)
        self.radio_z_dir_away    = QtGui.QRadioButton('Z Points Away',    ros_tab_container)
        self.radio_z_dir_away.setChecked(True)
        self.radio_z_dir_towards.toggled.connect(self.directionChanged)

        self.but_send = QtGui.QPushButton('Send Polygons', ros_tab_container)
        self.but_send.clicked.connect(self.sendPolys)
        self.but_send.setEnabled(False)
        
        self.wid_frame = QtGui.QLineEdit('/base_link', ros_tab_container)
        self.wid_resolution = QtGui.QLineEdit(str(self.RES), ros_tab_container)
        self.offset_x = QtGui.QLineEdit('0.0', ros_tab_container)
        self.offset_y = QtGui.QLineEdit('0.0', ros_tab_container)
        self.offset_z = QtGui.QLineEdit('0.0', ros_tab_container)
                
        self.offset_x.textChanged.connect(lambda : self.interfaceChanged.emit())
        self.offset_y.textChanged.connect(lambda : self.interfaceChanged.emit())
        self.offset_z.textChanged.connect(lambda : self.interfaceChanged.emit())

        self.wid_resolution.textChanged[str].connect(self.resolutionChanged)

        ros_tab_layout.addWidget(self.but_startros                   )
        ros_tab_layout.addWidget(QtGui.QLabel('Z Direction')         )
        ros_tab_layout.addWidget(self.radio_z_dir_towards            )
        ros_tab_layout.addWidget(self.radio_z_dir_away               )
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
        
        layout.addWidget(self.wid_tabs, 0, 1, 3, 2)
        layout.addWidget(self.wid_draw, 0, 0, 3, 1)
        # layout.addWidget(self.but_save, 2, 1)
        # layout.addWidget(self.but_load, 2, 2)
        
        layout.setColumnMinimumWidth(0, 640)
        
    def directionChanged(self):
        trans = self.wid_draw.transform()
        if self.radio_z_dir_towards.isChecked():
            mat = np.eye(3).flatten().tolist()
            mat[0] = -1.0
            trans.setMatrix(*mat)
        elif self.radio_z_dir_away.isChecked():
            mat = np.eye(3).flatten().tolist()
            mat[0] = 1.0
            trans.setMatrix(*mat)
        self.wid_draw.setTransform(trans, False)
        self.interfaceChanged.emit()

    def save_polygons(self, fname=None):
        if not fname:
            fname, _ = QtGui.QFileDialog.getSaveFileName(self, 'Save')
        if len(fname) == 0: return
        import pickle
        with open(fname, 'w') as f:
            pickle.dump(dict(
                polygons   = dict([(s, p.exportable()) for s, p in self.wid_draw.objects.iteritems()]),
                frame_id   = self.wid_frame.text(),
                resolution = self.wid_resolution.text(),
                offset_x   = self.offset_x.text(),
                offset_y   = self.offset_y.text(),
                offset_z   = self.offset_z.text()
            ), f)
        self.fileSaved.emit(os.path.split(fname)[-1])
        self.savefile = fname
            
    def load_polygons_click(self):
        fname, _ = QtGui.QFileDialog.getOpenFileName(self, 'Load')
        self.load_polygons(fname)

    def load_polygons(self, fname):
        import pickle
        if len(fname) == 0: return
        with open(fname, 'r') as f:
            self.wid_list.clear()
            self.wid_draw.clear()

            data = pickle.load(f)
            if type(data['polygons'].values()[0]) == QtGui.QPolygon: #enables loading old data
                seq = 0
                for name, poly in data['polygons'].iteritems():
                    poly_info = PolygonInfo(poly, name=name, uid='poly%s' % seq)
                    self.wid_draw.add_polygon(poly_info)
                    seq += 1
            else:
                for poly in data['polygons'].values():
                    self.wid_draw.add_polygon(poly) 

            self.wid_frame.setText(data['frame_id'])
            self.wid_resolution.setText(str(data['resolution']))
            self.offset_x.setText(str(data['offset_x']))
            self.offset_y.setText(str(data['offset_y']))
            self.offset_z.setText(str(data['offset_z']))
            
            self.wid_list.clear()

            for name in sorted(self.wid_draw.objects.keys()):
                self.polygonAdded(name)
        self.fileLoaded.emit(os.path.split(fname)[-1])
        self.savefile = fname
                    
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
            text_rect = QtPolyToROS(
                QtRectToPoly(poly_info.text_rect),
                '', x, y, z, res, self.wid_frame.text()
            )
            print ps
            self.polygon_proxy(uid, poly_info.name, ps, text_rect.polygon, colors.WHITE)
            markers.markers.append(toMarker(ps, np.int32(hash(uid))))
        self.polygon_viz.publish(markers)
        
    def startnode(self):
        self.but_send.setEnabled(True)
        from projector_interface.srv import DrawPolygon, ClearPolygons, DrawPolygonResponse, ClearPolygonsResponse
        from visualization_msgs.msg import MarkerArray
        from std_msgs.msg import Empty

        rospy.init_node('interface_builder', anonymous=True)
        self.but_startros.setEnabled(False)
        
        if self._standalone:
            rospy.loginfo('Starting dummy services')
            rospy.Service('draw_polygon', DrawPolygon, lambda x: DrawPolygonResponse())
            rospy.Service('clear_polygons', ClearPolygons, lambda x: ClearPolygonsResponse())

        self.polygon_proxy = rospy.ServiceProxy('/draw_polygon', DrawPolygon)
        rospy.loginfo("Waiting for polygon service")
        self.polygon_proxy.wait_for_service()
        rospy.loginfo("polygon service ready")
        rospy.loginfo("Waiting for polygon clear service")
        self.polygon_clear_proxy = rospy.ServiceProxy('/clear_polygons', ClearPolygons)
        rospy.loginfo("polygon clear service ready")
        self.polygon_viz = rospy.Publisher('/polygon_viz', MarkerArray)

        self.cursor_pub = rospy.Publisher('intersected_points', pointclouds.PointCloud2)
        self.click_pub = rospy.Publisher('/click', Empty)

        self.but_startros.setText('Node Running...')
        
    def resolutionChanged(self, text):
        self.wid_draw.set_resolution(float(text))
        try:
            FONT.setPointSize(30-float(text)*12000)
        except Exception:
            pass
        self.interfaceChanged.emit()
        
    def deleteClick(self):
        self.wid_draw.removeObject(self.wid_list.currentItem().text())
        self.wid_list.takeItem(self.wid_list.currentRow())
        self.interfaceChanged.emit()

    def polygonAdded(self, name):
        self.wid_list.addItem(name)
        self.interfaceChanged.emit()
        
    def mouseMoved(self, location):
        offset_x = float(self.offset_x.text())
        offset_y = float(self.offset_y.text())

        res = float(self.wid_resolution.text())
        # Qt defines 0,0 in the upper left, so, and down as +y, so flip the y
        x, y = location.x()*res+offset_x, -location.y()*res+offset_y
        message = '%s x=%0.4fm, y=%0.4f' % (self.draw_mode, x, y)
        self.window().statusBar().showMessage(message)

        if (self.wid_tabs.currentIndex() == MOUSE_MODE_TEST) and (self.but_send.isEnabled()):
            cloud = pointclouds.xyz_array_to_pointcloud2(np.array([[x, y, 0]]), frame_id=self.wid_frame.text())
            self.cursor_pub.publish(cloud)

    def mouseClick(self):
        if (self.wid_tabs.currentIndex() == MOUSE_MODE_TEST) and (self.but_send.isEnabled()):
            self.click_pub.publish()

    def modeUpdate(self, mode):
        self.draw_mode = mode

    def listItemSelectionChanged(self):
        self._changePoly = True
        item = self.wid_list.currentItem()
        poly = self.wid_draw.objects[item.text()]
        self.wid_draw.clear_active_point()
        self.wid_draw.setActive(item.text())
        self.wid_edit.setItem(0, 0, QtGui.QTableWidgetItem(poly.name))
        self.wid_edit.setItem(1, 0, QtGui.QTableWidgetItem(poly.id))
        points = poly.polygon
        self.wid_edit.setRowCount(2+len(points))
        res = float(self.wid_resolution.text())
        for px, point in enumerate(points):
            x = point.x()*res
            y = point.y()*res
            self.wid_edit.setItem(px+2, 0, QtGui.QTableWidgetItem('(%0.4f, %0.4f)' % (x, y)))
        self._changePoly = False

    def updateName(self, text):
        text = text.replace('\\n', '\n')
        if text != self.wid_list.currentItem().text():
            self.wid_draw.updateName(self.wid_list.currentItem().text(), text)
            return True
        return False

    def updateId(self, text):
        old_id = self.wid_list.currentItem().text()
        if old_id != text:
            self.wid_draw.updateId(old_id, text)
            self.wid_list.currentItem().setText(text)
            return True
        return False
        
    def keyPressEvent(self, event):
        if event.key() in (16777248, 16777216):
            self.wid_draw.keyPressEvent(event)
        
    def keyReleaseEvent(self, event):
        if event.key() in (16777248, 16777216):
            self.wid_draw.keyReleaseEvent(event)
    
    def cellChanged(self, row, col):
        item = self.wid_edit.item(row, col)
        changed = False
        if row == 0:
            changed = self.updateName(item.text())
        elif row == 1:
            changed = self.updateId(item.text())
        else:
            res = float(self.wid_resolution.text())
            try:
                exec('x,y=%s' % item.text())
                changed = self.wid_draw.updatePoint(self.wid_list.currentItem().text(), row-2, x/res, y/res)
                # self.wid_draw.update_active_point(QtCore.QPoint(x,y))
                self.wid_draw.clear_active_point()
            except Exception, e:
                print e
        if changed and not self._changePoly:
            self.interfaceChanged.emit()
                
    def itemSelectionChanged(self):
        item = self.wid_edit.currentItem()
        if self.wid_edit.currentRow() > 1:
            res = float(self.wid_resolution.text())
            try:
                exec('x,y=%s' % item.text())
                self.wid_draw.update_active_point(QtCore.QPoint(x/res, y/res))
            except Exception, e:
                print e
                 
class DrawWidget(QtGui.QGraphicsView):
    objects = dict()
    polygon_active = False
    current_poly = []
    polygonAdded = QtCore.Signal(str)
    mouseMoved = QtCore.Signal(QtCore.QPointF)
    mouseClick = QtCore.Signal()
    modeUpdate = QtCore.Signal(str)
    active_poly = ''
    snap = False
    active_point = None
    cursorx, cursory = 0, 0
    text_move = False
    drag_start = False
    otherSnap = None
    last_click = None
    _grid_step = 20
    res = 0.001
    POLYGON_PEN      = QtGui.QPen(QtGui.QColor(128, 128, 128))
    GRID_PEN         = QtGui.QPen(QtGui.QColor( 25,  25,  25))
    ACTIVE_POINT_PEN = QtGui.QPen(QtGui.QColor(  0, 255,   0), 3)
    ACTIVE_PEN       = QtGui.QPen(QtGui.QColor(255, 255, 255), 3)
    ACTIVE_EDIT_PEN  = QtGui.QPen(QtCore.Qt.gray, 1, QtCore.Qt.DashLine)

    def __init__(self):
        super(DrawWidget, self).__init__()
        self.setMouseTracking(True)
        self.scene = QtGui.QGraphicsScene()
        self.scene.setBackgroundBrush(QtGui.QColor(0, 0, 0))
        # self.setRenderHints(QtGui.QPainter.Antialiasing)

        self._ruler = self.scene.addText('', RULER_FONT)
        self._ruler.hide()
        self._ruler.setZValue(1000)

        self._cursor_point = self.scene.addEllipse(QtCore.QRect(0, 0, 0, 0), self.ACTIVE_POINT_PEN)
        self._cursor_point.hide()
        self._cursor_point.setZValue(1000)

        self.setScene(self.scene)
        self.draw_grid_lines()
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        self.mouseMode = MOUSE_MODE_EDIT

        self.insertMode = -1
        self.editMode = modes.EDIT_MODE_NONE

        self.centerOn(0, 0)
    
    def clear(self):
        for obj in self.objects.itervalues():
            self.scene.removeItem(obj.gfx_item)
            self.scene.removeItem(obj.text_item)
        self.objects.clear()
        self.active_poly = ''
        self.active_point = None
        self.polygon_active = False

    def set_resolution(self, res):
        self.res = res

    def add_polygon(self, poly_info):
        poly_info.set_item(self.scene.addPolygon(poly_info.polygon, pen=self.POLYGON_PEN))
        text_item = self.scene.addText(poly_info.name)
        text_item.setPos(poly_info.text_rect.topLeft())
        text_item.setFont(FONT)
        poly_info.set_text_item(text_item)
        self.objects[poly_info.id] = poly_info

    def update_active_point(self, point):
        self.clear_active_point()
        offset = QtCore.QPoint(1, 1)
        self.active_point = self.scene.addEllipse(QtCore.QRect(point-offset, point+offset), self.ACTIVE_POINT_PEN)
        self.active_point.setZValue(self.objects[self.active_poly].zValue()+1)

    def clear_active_point(self):
        if self.active_point:
            self.scene.removeItem(self.active_point)

    def setDrawModeRect(self):
        self.insertMode = modes.INSERT_MODE_RECT

    def setDrawModePolygon(self):
        self.insertMode = modes.INSERT_MODE_POLYGON

    def setEditModeObject(self):
        self.updateActivePen()
        self.editMode = modes.EDIT_MODE_OBJECT

    def setEditModePoint(self):
        self.updateActivePen()
        self.editMode = modes.EDIT_MODE_POINT

    def setEditModeText(self):
        self.updateActivePen()
        self.editMode = modes.EDIT_MODE_TEXT

    def updateActivePen(self):
        if self.active_poly and self.editMode == modes.EDIT_MODE_OBJECT:
            self.objects[self.active_poly].gfx_item.setPen(self.POLYGON_PEN)
        elif self.active_poly:
            self.objects[self.active_poly].gfx_item.setPen(self.ACTIVE_PEN)

    def setMouseMode(self, index):
        self.mouseMode = index

    def updateName(self, pid, newName):
        self.objects[pid].name = newName
        self.objects[pid].update_font_box()

    def updateId(self, oldId, newId):
        if oldId != newId:
            self.objects[newId] = self.objects[oldId]
            self.objects[newId].id = newId
            # self.removeObject(oldId)
        self.setActive(newId)

    def updatePoint(self, uid, point_index, x, y):
        pt = self.objects[uid].polygon[point_index]
        if not pt.x() == x or not pt.y() == y:
            pt.setX(x)
            pt.setY(y)
            self.objects[uid].polygon.replace(point_index, pt)
            self.objects[uid].gfx_item.setPolygon(self.objects[uid].polygon)
            return True
        return False

    def removeObject(self, name):
        obj = self.objects[name]
        self.scene.removeItem(obj.gfx_item)
        self.scene.removeItem(obj.text_item)
        self.clear_active_point()
        self.active_poly = ''

        del self.objects[name]

    def make_top(self, item):
        item.setZValue(max([sib.zValue() for sib in self.scene.items()])+1)

    def setActive(self, name):
        if self.active_poly:
            self.objects[self.active_poly].gfx_item.setPen(self.POLYGON_PEN)
            self.objects[self.active_poly].hide_bounding_box()
        self.active_poly = name
        if self.editMode and (modes.EDIT_MODE_OBJECT or modes.EDIT_MODE_POINT):
            self.objects[name].gfx_item.setPen(self.POLYGON_PEN)
            self.objects[self.active_poly].show_bounding_box(self.ACTIVE_EDIT_PEN)
        else:
            self.objects[name].gfx_item.setPen(self.ACTIVE_PEN)
        self.make_top(self.objects[name].gfx_item)

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
                if self.closeTo(v, p):
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

    def do_rect_click(self, event):
        cursor = self.mapToScene(event.pos())
        endpoint = self.get_line_endpoint(cursor, event.modifiers(), highlight=True)
        if not self.last_click:
            self.last_click = endpoint
            self.ins_rect = QtGui.QGraphicsRectItem(QtCore.QRectF(endpoint, endpoint))
            self.ins_rect.setPen(self.POLYGON_PEN)
            self.scene.addItem(self.ins_rect)
            self.polygon_active = True
        else:
            poly = QtGui.QPolygon.fromList([
                self.ins_rect.rect().topLeft().toPoint(),
                self.ins_rect.rect().topRight().toPoint(),
                self.ins_rect.rect().bottomRight().toPoint(),
                self.ins_rect.rect().bottomLeft().toPoint()
            ])
            poly_container = PolygonInfo(QtGui.QPolygon(poly), name=self.generate_name())
            self.add_polygon(poly_container)
            self.polygonAdded.emit(poly_container.id)
            self.polygon_active = False
            self.remove_active_poly_items()
            self.snap = False
            self.last_click = None
            self.ins_rect.scene().removeItem(self.ins_rect)

    def do_polygon_click(self, event):
        cursor = self.mapToScene(event.pos())
        # this lets us close a polygon by single-clicking on its first vertex
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
        endpoint = self.get_line_endpoint(cursor, event.modifiers(), highlight=True)
        if (event.button() == QtCore.Qt.MouseButton.LeftButton) and (self.polygon_active):
            # If this isn't the first point in the polygon
            self.reset_active_line(endpoint)
        elif (event.button() == QtCore.Qt.MouseButton.LeftButton) and (not self.polygon_active):
            # If this is the first point in the polygon
            pt = endpoint
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

    def update_cursor_point(self, pos):
        self._cursor_point.show()
        offset = QtCore.QPoint(1, 1)
        self._cursor_point.setRect(QtCore.QRectF(pos-offset, pos+offset))

    def get_line_endpoint(self, pos, modifiers, highlight=False):
        if modifiers == QtCore.Qt.ControlModifier and self.insertMode != modes.INSERT_MODE_RECT:
            return self.snapToAxis(pos)

        cta = self.closeToAny(pos)
        if cta is not None:
            if highlight:
                self.update_cursor_point(cta)
            return cta
        cts = self.closeToCurrent(pos)
        if cts is not None:
            if highlight:
                self.update_cursor_point(cts)
            return cts
        if modifiers & QtCore.Qt.ShiftModifier:
            x, y = pos.toTuple()
            x = int(self._grid_step*round(x/self._grid_step))
            y = int(self._grid_step*round(y/self._grid_step))
            grid_pt = QtCore.QPoint(x, y)
            if self.closeTo(pos, grid_pt):
                if highlight:
                    self.update_cursor_point(grid_pt)
                return grid_pt

        self._cursor_point.hide()
        return pos

    def do_text_click(self, event):
        if self.text_move:
            self.text_move = False
        else:
            self.text_move = True
            self.text_rect_orig = self.objects[self.active_poly].text_rect.translated(QtCore.QPoint())

    def mousePressEvent(self, event):
        if self.editMode and modes.EDIT_MODE_ANY:
            super(DrawWidget, self).mousePressEvent(event)
        else:
            if event.button() == QtCore.Qt.MouseButton.MiddleButton:
                self.setDragMode(QtGui.QGraphicsView.DragMode.ScrollHandDrag)
                super(DrawWidget, self).mousePressEvent(QtGui.QMouseEvent(
                    event.type(),
                    event.pos(),
                    QtCore.Qt.MouseButton.LeftButton,
                    event.buttons(),
                    event.modifiers()
                ))
            elif self.active_poly and self.objects[self.active_poly].in_text_box((self.cursorx, self.cursory)):
                self.do_text_click(event)
            elif self.insertMode == modes.INSERT_MODE_POLYGON:
                self.do_polygon_click(event)
            elif self.insertMode == modes.INSERT_MODE_RECT:
                self.do_rect_click(event)

    def mouseReleaseEvent(self, event):
        button = event.button()
        if event.button() == QtCore.Qt.MouseButton.MiddleButton:
            button = QtCore.Qt.MouseButton.LeftButton
            self.setDragMode(QtGui.QGraphicsView.DragMode.NoDrag)

        if (self.mouseMode == MOUSE_MODE_TEST) and (event.button() == QtCore.Qt.MouseButton.LeftButton):
            self.mouseClick.emit()

        super(DrawWidget, self).mouseReleaseEvent(QtGui.QMouseEvent(
            event.type(),
            event.pos(),
            button,
            event.buttons(),
            event.modifiers(),
        ))
        self.drag_start = False

        
    def mouseDoubleClickEvent(self, event):        
        if self.mouseMode == MOUSE_MODE_EDIT:
            # polygon-ize the line list
            if event.button() == QtCore.Qt.MouseButton.LeftButton:
                poly = QtGui.QPolygon.fromList([l.line().p1().toPoint() for l in self.current_poly])
                poly_container = PolygonInfo(QtGui.QPolygon(poly), name=self.generate_name())
                self.add_polygon(poly_container)
                self.polygonAdded.emit(poly_container.id)
                self.polygon_active = False
                self.remove_active_poly_items()
                self.snap = False
                self.last_click = None
                self._ruler.hide()
                
    def mouseMoveEvent(self, event):
        if self.mouseMode == MOUSE_MODE_EDIT:
            pos = self.mapToScene(event.pos())
            self.cursorx = pos.x()
            self.cursory = pos.y()
            endpoint = self.get_line_endpoint(pos, event.modifiers(), highlight=True)
            self.mouseMoved.emit(endpoint)

            if self.insertMode == modes.INSERT_MODE_POLYGON:
                if self.polygon_active:
                    # Note: line() returns a *copy* of the backing line, so changing it doesn't do anything
                    line = self.active_line.line()
                    line.setP2(endpoint)
                    self.active_line.setLine(line)

                    # update the ruler
                    self._ruler.setPos((line.p1() + line.p2())/2)
                    rise, run = line.dx(), line.dy()
                    if rise != 0:
                        angle = np.degrees(np.arctan(run/rise))
                        self._ruler.setRotation(angle)
                    dist = np.hypot(rise, run)*self.res
                    self._ruler.setPlainText('%0.4fm' % dist)
                    self._ruler.show()
            elif self.insertMode == modes.INSERT_MODE_RECT:
                if self.polygon_active:
                    rect = self.ins_rect.rect()
                    rect.setCoords(self.last_click.x(), self.last_click.y(), endpoint.x(), endpoint.y())
                    if   self.last_click.x() < endpoint.x() and self.last_click.y() < endpoint.y():
                        # self.last_click is top left, endpoint is bottom right
                        rect.setTopLeft(self.last_click)
                        rect.setBottomRight(endpoint)
                    elif self.last_click.x() < endpoint.x() and self.last_click.y() > endpoint.y():
                        # self.last_click is bottom left, endpoint is top right
                        rect.setBottomLeft(self.last_click)
                        rect.setTopRight(endpoint)
                    elif self.last_click.x() > endpoint.x() and self.last_click.y() < endpoint.y():
                        # self.last_click is top right, endpoint is bottom left
                        rect.setTopRight(self.last_click)
                        rect.setBottomLeft(endpoint)
                    elif self.last_click.x() > endpoint.x() and self.last_click.y() > endpoint.y():
                        # self.last_click is bottom right and endpoint is top left
                        rect.setBottomRight(self.last_click)
                        rect.setTopLeft(endpoint)

                    self.ins_rect.setRect(rect)
        elif self.mouseMode == MOUSE_MODE_TEST:
            # Qt defines 0,0 in the upper left, so, and down as +y, so flip the y
            pos = event.pos()
            # pos.setY(-pos.y())
            self.mouseMoved.emit(self.mapToScene(pos))
        super(DrawWidget, self).mouseMoveEvent(event)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Control:
            self.modeUpdate.emit(MODE_AXIS_ALIGN)
        if event.key() == QtCore.Qt.Key.Key_Escape:
            if self.text_move:
                self.text_move = False
                self.objects[self.active_poly].text_rect = self.text_rect_orig
            self.polygon_active = False
            self.remove_active_poly_items()
            self._ruler.hide()
            self.last_click = None
            try:
                self.ins_rect.scene().removeItem(self.ins_rect)
            except:
                pass # might not exist

            self.snap = False
        elif event.key() ==  QtCore.Qt.Key.Key_Shift:
            self.modeUpdate.emit(MODE_SNAP_TO_GRID)
        
    def keyReleaseEvent(self, event):
        self.modeUpdate.emit(MODE_NONE)

    def remove_active_poly_items(self):
        for line in self.current_poly:
            line.scene().removeItem(line)
            self.current_poly = []

    def draw_grid_lines(self):
        top    = -1000
        bottom =  1000
        left   = -1000
        right  =  1000
        for inc in range(left, right, self._grid_step):
            self.scene.addLine(
                QtCore.QLine(
                    QtCore.QPoint(inc, top),
                    QtCore.QPoint(inc, bottom)
                ), self.GRID_PEN)
        for inc in range(top, bottom, self._grid_step):
            self.scene.addLine(
                QtCore.QLine(
                    QtCore.QPoint(left, inc),
                    QtCore.QPoint(right, inc)
                ), self.GRID_PEN)

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--standalone', action='store_const',
        dest='standalone', const=True, default=False,
        help='standalone mode (startup fake service servers)'
    )
    parser.add_argument('savefile', nargs='?', help='path to saved interface file')
    args = parser.parse_args(args=rospy.myargv()[1:])

    gui = BuilderWindow(args.savefile, args.standalone)
    sys.exit(app.exec_())