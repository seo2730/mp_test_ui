#!/usr/bin/env python3
import os

from ament_index_python.resources import get_resource
### mgs 관련 커스텀 메시지 추가 ###
from geometry_msgs.msg import Pose # 임시
#################################
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer, QPointF
from python_qt_binding.QtGui import QKeySequence, QBrush, QColor, QPainter
from python_qt_binding.QtWidgets import QShortcut
from python_qt_binding.QtWidgets import QWidget, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem
import rclpy
from rclpy.qos import QoSProfile

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas


class MpTestWidget(QWidget):

    def __init__(self, node):
        super(MpTestWidget, self).__init__()
        self.setObjectName('MpTestUiWidget')

        self.node = node

        pkg_name = 'mp_test_ui'
        ui_filename = 'mp_test_ui.ui'
        
        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)

        #for draw graph
        self.fig = plt.Figure()
        self.canvas = FigureCanvas(self.fig)
        self.plotLayout.addWidget(self.canvas)

        # matplotlib 플롯을 초기화
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 100)
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')

        # 로봇 위치를 표시할 선/포인트 삭제 변수
        self.current_plot = None

        self.first=False

        # QOS
        qos=QoSProfile(depth=10)

        # mgs pub sub 선언
        self.pose = Pose()

        ### 임시 ###
        self.sub_pose = self.node.create_subscription(Pose, "/robot_pose",self.GetRobotPose,qos)
        ############

    def GetRobotPose(self, msg):
        self.pose = msg
        # 처음 정보 받는 swarm_info를 이용하여 그래프에 띄우자
        # 콜백함수는 정보만 업데이트하고 QTimer 이용해서 일정 주기로 업데이트하는 방식도 괜찮을듯
        if(self.current_plot!=None):
            self.current_plot.remove()

        x, y = self.pose.position.x , self.pose.position.y
        self.current_plot, = self.ax.plot(x, y, 'bo')  # 'bo'는 파란색 원을 표시
        self.canvas.draw()  # canvas 업데이트