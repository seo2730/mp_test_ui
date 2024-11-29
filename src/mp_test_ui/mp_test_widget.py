#!/usr/bin/env python3
import os

from ament_index_python.resources import get_resource
from geometry_msgs.msg import Twist
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut
from python_qt_binding.QtWidgets import QWidget
import rclpy
from rclpy.qos import QoSProfile
from std_srvs.srv import SetBool


class MpTestWidget(QWidget):

    def __init__(self, node):
        super(MpTestWidget, self).__init__()
        self.setObjectName('MpTestUiWidget')

        self.node = node

        self.REDRAW_INTERVAL = 30
        self.PUBLISH_INTERVAL = 100
        self.CMD_VEL_X_FACTOR = 1000.0
        self.CMD_VEL_YAW_FACTOR = -10.0

        pkg_name = 'mp_test_ui'
        ui_filename = 'mp_test_ui.ui'

        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)