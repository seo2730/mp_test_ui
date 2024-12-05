#!/usr/bin/env python3
import os

from ament_index_python.resources import get_resource
### 시험용 SW ICD ###
from mgs05_mp_msgs.msg import RobotPerformValues, RobotTypeNums, GroupPerformValues, MpTestStatic
from mgs05_base_msgs.msg import SwarmInfo, Rect, DynamicCustomPlan, Plans
####################
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer, QPointF
from python_qt_binding.QtGui import QKeySequence, QBrush, QColor, QPainter, QStandardItemModel, QStandardItem
from python_qt_binding.QtWidgets import QShortcut, QTableWidgetItem, QTreeView, QVBoxLayout, QTableWidget, QTabWidget
from python_qt_binding.QtWidgets import QWidget, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import yaml


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
        self.ax.set_xlim(0, 30)
        self.ax.set_ylim(0, 30)
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')

        # QTimer로 일정 주기로 로봇의 위치 업데이트
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.UpdateSwarmInfo)
        self.timer.start(1000)  # 1초마다 업데이트

        # 로봇 위치를 표시할 선/포인트 삭제 변수
        self.current_plot = {}

        # 표 설정
        self.max_id = 0
        self.max_group_id = 0
        self.robot_state_name = ['로봇 ID', '로봇 타입', '   로봇  위치   ', '배터리', '구동기', '연결']
        self.robot_table.setRowCount(30)   # 행 개수 설정
        self.robot_table.setColumnCount(6)   # 열 개수 설정
        self.robot_table.setHorizontalHeaderLabels(self.robot_state_name)
        for col in range(self.robot_table.columnCount()):
            self.robot_table.resizeColumnToContents(col)

        self.group_state_name = ['그룹 ID', '리더 ID', '   그룹  위치   ', '현재 과업', '대기 중인 과업들', '구성원']
        self.group_table.setRowCount(10)
        self.group_table.setColumnCount(6)
        self.group_table.setHorizontalHeaderLabels(self.group_state_name)
        for col in range(self.group_table.columnCount()):
            self.group_table.resizeColumnToContents(col)

        # self.robot_plan_name = ['로봇 ID', '성능값', '거리', '배터리'] # 과업 요구사항에 따른 다르게 표시하는 법 알아보자
        # self.robot_plan_table.setRowCount(30)

        self.group_plan_name = ['그룹 ID', '성능값', '거리'] # 과업 요구사항에 따른 다르게 표시하는 법 알아보자
        # self.group_plan.setRowCount(10)

        # 과업 요구사항 트리 설정
        self.task_perform_name = ["maneuver_performance", "operation_persistence_performance", "observation_recon"]

        self.former_row=None

        # QOS
        best_effort=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        reliable = QoSProfile(depth=10)

        # mgs pub sub 선언
        ### 시험용 SW ICD ###
        # 정적 임무 계획
        self.static_mission_plan = None
        # 동적 임무 계획
        self.dynamic_mission_plan = None
        # 로봇 상태 정보
        self.swarm_info = None
        # 임무 범위
        self.mission_range = Rect()
        # 각 로봇 별 성능값
        self.robot_perform_values = RobotPerformValues()
        # 타입별 개수 구성원
        self.robots_type_num = RobotTypeNums()
        # 그룹 성능값
        self.group_perform_values = GroupPerformValues()

        self.mp_plan={}

        ### 왜 통신 문제가 생기는겨????
        # self.sub_static_mission_plan = self.node.create_subscription(Plans, "/mission_order_r",self.GetStaticMissionPlan, best_effort)
        ###
        self.sub_dynamic_mission_plan = self.node.create_subscription(DynamicCustomPlan, "/dynamic_mission_change",self.GetDynamicMissionPlan, best_effort)
        self.sub_swarm_info = self.node.create_subscription(SwarmInfo, "/swarm_info",self.GetSwarmInfo, best_effort)
        self.sub_rect = self.node.create_subscription(Rect,"/mission_range",self.GetMissionRange, reliable)
        # self.sub_robots_performance_value = self.node.create_subscription(RobotPerformValues, "/robots_performance_value", self.GetRobotPerformanceValue, best_effort)
        # self.sub_robots_type_num = self.node.create_subscription(RobotTypeNums, "/robot_type_num", self.GetRobotTypeNum, best_effort)
        self.sub_groups_performance_value = self.node.create_subscription(RobotTypeNums, "/groups_performance_value", self.GetGroupPerformanceValue, best_effort)
        self.sub_mp_test_static = self.node.create_subscription(MpTestStatic, "/mp_test_static", self.GetRobotPerformanceValue, best_effort)
        ####################

        self.tasks_table.cellClicked.connect(self.OnTasksTableRowClicked)

    # 정적임무계획 수신
    def GetStaticMissionPlan(self, msg):
        print("@@@ Get Static Plan @@@")


    # 동적임무계획 수신
    def GetDynamicMissionPlan(self, msg):
        print("@@@ Get Dynamic Plan @@@")
        self.dynamic_mission_plan = msg

    # swarm info 값 수신
    def GetSwarmInfo(self, msg):
        self.swarm_info = msg

    # 타입 구성별 개수 수신
    def GetRobotTypeNum(self, msg):
        print("@@@ Get Robot Type Num @@@")
        self.robots_type_num = msg

    # 임무 범위 수신
    def GetMissionRange(self,msg):
        self.mission_range = msg

    # 로봇 성능값 수신
    def GetRobotPerformanceValue(self, msg):
        print("@@@ Get Robot Performance Value @@@")
        robot_plan_table_name = ['로봇 ID', '성능값']
        self.robot_plan_table.setRowCount(len(msg.robot_values))
        self.robot_plan_table.setColumnCount(2)
        self.robot_plan_table.setHorizontalHeaderLabels(robot_plan_table_name)
        rol = 0
        for robot_peform_value in msg.robot_values:
            self.robot_plan_table.setItem(rol,0,QTableWidgetItem(str(robot_peform_value.robot_id%100)))
            self.robot_plan_table.setItem(rol,1,QTableWidgetItem(str(robot_peform_value.performance_value)))
            rol += 1

        self.tasks_table.clear()
        self.tasks_table.setRowCount(len(msg.plans))

        num_task =0
        self.mp_plan.clear()
        for plan in msg.plans:
            task_name_kor, task_name_eng = self.TaskName(plan.task[0].task_name)
            print(task_name_kor + ", " + task_name_eng)
            file_path=os.path.join('src/mp_test_ui/src/mp_test_ui/config',"static_task_"+task_name_eng+".yaml")
            if os.path.isfile(file_path):
                print("File Exist")
                with open(file_path) as file:
                    config = yaml.load(file, Loader=yaml.FullLoader)

                param_name_set = []
                param_name_set.append("과업명")
                param_name_dict = {}
                param_name_dict["과업명"] = task_name_kor
                for performance_param in self.task_perform_name:
                    print(performance_param)
                    if performance_param in config:
                        print(config[performance_param])
                        for param_key, param_value in config[performance_param].items():
                            param_name = self.TaskParamName(param_key)
                            param_name_dict[param_name] = param_value
                            param_name_set.append(param_name)
                    else:
                        print("No requirement")
                        continue

                # print(param_name_dict)
                param_name_dict["group"] = plan.groups
                self.mp_plan["정적_과업_"+str(num_task)]=param_name_dict
                self.tasks_table.setColumnCount(1)
                self.tasks_table.setHorizontalHeaderLabels(["과업명"])

                for key, value in self.mp_plan.items():
                    self.tasks_table.setItem(num_task,0, QTableWidgetItem(str(value["과업명"])))

                num_task+=1
            else:
                print("No File Exist")
                continue

    # 그룹 성능값 수신
    def GetGroupPerformanceValue(self, msg):
        print("@@@ Get Group Performance Value @@@")
        group_plan_table_name = ['그룹 ID', '리더 ID', '성능값']
        self.group_plan_table.setRowCount(len(msg.robot_values))
        self.group_plan_table.setColumnCount(2)
        self.group_plan_table.setHorizontalHeaderLabels(group_plan_table_name)
        rol = 0
        for group_peform_value in msg.robot_values:
            pass

    # swarm info 값 QTimer로 업데이트
    def UpdateSwarmInfo(self):
        if self.swarm_info is not None:
            # 예비대가 있을 경우
            if self.swarm_info.swarm_group_status[0] is not None:
                prelimary_robot = self.swarm_info.swarm_group_status[0]
                for robot_state in prelimary_robot.platforms_state:
                    x,y = robot_state.robot_pose.position.x, robot_state.robot_pose.position.y
                    platform_id, robot_type = self.UpdateRobotIdType(robot_state)
                    if platform_id in self.current_plot:
                        self.current_plot[platform_id].set_position((x,y))
                    else:
                        # self.FindMaxRobotID(platform_id)
                        # 위치 도시화
                        # 지상 주행
                        if robot_type == 0:
                            self.current_plot[platform_id] = self.ax.text(x, y, str(platform_id), color='white', ha='center', va='center', fontsize=10,
                                                            bbox=dict(facecolor='black', edgecolor='none', boxstyle='circle'))
                        # 지상 도약
                        elif robot_type == 1:
                            self.current_plot[platform_id] = self.ax.text(x, y, str(platform_id), color='white', ha='center', va='center', fontsize=10,
                                                            bbox=dict(facecolor='black', edgecolor='none', boxstyle='square'))
                        # 공중 지상
                        elif robot_type == 2:
                            self.current_plot[platform_id] = self.ax.text(x, y, str(platform_id), color='white', ha='center', va='center', fontsize=10,
                                                            bbox=dict(facecolor='black', edgecolor='none', boxstyle='sawtooth'))
                        # 공중 벽면
                        elif robot_type == 3:
                            self.current_plot[platform_id] = self.ax.text(x, y, str(platform_id), color='white', ha='center', va='center', fontsize=10,
                                                            bbox=dict(facecolor='black', edgecolor='none', boxstyle='darrow'))
                    # 표 상태 도시화
                    self.RobotStatusUpdateTab(robot_state)

            # 그룹 안에 구성원들 정보 도시화 (위치는 리더만 도시화)
            if len(self.swarm_info.swarm_group_status)>1:
                for group in self.swarm_info.swarm_group_status[1:]:
                    # self.FindMaxGroupID(group.group_id)
                    self.GroupStatusUpdateTab(group)
                    for robot_state in group.platforms_state:
                        platform_id, robot_type = self.UpdateRobotIdType(robot_state)
                        if group.leader_id == robot_state.platform_id:
                            x,y = robot_state.robot_pose.position.x, robot_state.robot_pose.position.y
                            if platform_id in self.current_plot:
                                self.current_plot[platform_id].get_bbox_patch().set_facecolor('blue')
                                self.current_plot[platform_id].set_position((x,y)) # 리더만 위치 도시화
                            else:
                                # self.FindMaxRobotID(platform_id)
                                # 위치 도시화
                                # 지상 주행
                                if robot_type == 0:
                                    self.current_plot[platform_id] = self.ax.text(x, y, str(platform_id), color='white', ha='center', va='center', fontsize=10,
                                                                    bbox=dict(facecolor='blue', edgecolor='none', boxstyle='circle'))
                                # 지상 도약
                                elif robot_type == 1:
                                    self.current_plot[platform_id] = self.ax.text(x, y, str(platform_id), color='white', ha='center', va='center', fontsize=10,
                                                                    bbox=dict(facecolor='blue', edgecolor='none', boxstyle='square'))
                                # 공중 지상
                                elif robot_type == 2:
                                    self.current_plot[platform_id] = self.ax.text(x, y, str(platform_id), color='white', ha='center', va='center', fontsize=10,
                                                                    bbox=dict(facecolor='blue', edgecolor='none', boxstyle='sawtooth'))
                                # 공중 벽면
                                elif robot_type == 3:
                                    self.current_plot[platform_id] = self.ax.text(x, y, str(platform_id), color='white', ha='center', va='center', fontsize=10,
                                                                    bbox=dict(facecolor='blue', edgecolor='none', boxstyle='darrow'))
                        else:
                            if platform_id in self.current_plot:
                                self.current_plot[platform_id].set_position((-100000,-100000)) # 팔로워는 위치 표시 제외
                            else:
                                if robot_type == 0:
                                    self.current_plot[platform_id] = self.ax.text(-100000, -100000, str(platform_id), color='white', ha='center', va='center', fontsize=10,
                                                                    bbox=dict(facecolor='black', edgecolor='none', boxstyle='circle'))
                                # 지상 도약
                                elif robot_type == 1:
                                    self.current_plot[platform_id] = self.ax.text(-100000, -100000, str(platform_id), color='white', ha='center', va='center', fontsize=10,
                                                                    bbox=dict(facecolor='black', edgecolor='none', boxstyle='square'))
                                # 공중 지상
                                elif robot_type == 2:
                                    self.current_plot[platform_id] = self.ax.text(-100000, -100000, str(platform_id), color='white', ha='center', va='center', fontsize=10,
                                                                    bbox=dict(facecolor='black', edgecolor='none', boxstyle='sawtooth'))
                                # 공중 벽면
                                elif robot_type == 3:
                                    self.current_plot[platform_id] = self.ax.text(-100000, -100000, str(platform_id), color='white', ha='center', va='center', fontsize=10,
                                                                    bbox=dict(facecolor='black', edgecolor='none', boxstyle='darrow'))
            self.canvas.draw()  # canvas 업데이트

    # 로봇 아이디, 타입 추출
    def UpdateRobotIdType(self, robot_state):
        robot_id = robot_state.platform_id%100
        robot_type = robot_state.platform_id//100
        return robot_id, robot_type

    # 로봇 상태정보 탭 업데이트
    def RobotStatusUpdateTab(self, robot_state):
        x,y = robot_state.robot_pose.position.x, robot_state.robot_pose.position.y
        platform_id, robot_type = self.UpdateRobotIdType(robot_state)
        self.robot_table.setItem(platform_id-1, 0, QTableWidgetItem(str(platform_id)))
        # 지상 주행
        if robot_type == 0:
           self.robot_table.setItem(platform_id-1, 1, QTableWidgetItem('지상주행'))
        # 지상 도약
        elif robot_type == 1:
            self.robot_table.setItem(platform_id-1, 1, QTableWidgetItem('지상도약'))
        # 공중 지상
        elif robot_type == 2:
            self.robot_table.setItem(platform_id-1, 1, QTableWidgetItem('공중지상'))
        # 공중 벽면
        elif robot_type == 3:
            self.robot_table.setItem(platform_id-1, 1, QTableWidgetItem('공중벽면'))

        self.robot_table.setItem(platform_id-1, 2, QTableWidgetItem('('+str(x)+', '+str(y)+')'))
        self.robot_table.setItem(platform_id-1, 3, QTableWidgetItem(str(robot_state.battery_percentage)))
        self.robot_table.setItem(platform_id-1, 4, QTableWidgetItem(str(robot_state.actuator_state.data)))
        self.robot_table.setItem(platform_id-1, 5, QTableWidgetItem(str(robot_state.network_connection.data)))

    # 그룹 정보 상태 정보 탭 업데이트
    def GroupStatusUpdateTab(self, group):
        x,y = 0,0
        leader_id = 0
        platform_id, robot_type = 0,0
        group_id = group.group_id
        current_task = group.group_current_task
        group_member = []
        for robot_state in group.platforms_state:
            if group.leader_id == robot_state.platform_id:
                self.UpdateRobotIdType(robot_state)
            else:
                platform_id, robot_type= self.UpdateRobotIdType(robot_state)
                group_member.append(platform_id)

        self.group_table.setItem(group_id-1, 0, QTableWidgetItem(str(group_id)))
        self.group_table.setItem(group_id-1, 1, QTableWidgetItem(str(leader_id)))

        self.group_table.setItem(group_id-1, 2, QTableWidgetItem('('+str(x)+', '+str(y)+')'))
        self.group_table.setItem(group_id-1, 3, QTableWidgetItem(str(current_task)))
        self.group_table.setItem(group_id-1, 4, QTableWidgetItem(str('임시 데이터')))
        self.group_table.setItem(group_id-1, 5, QTableWidgetItem(str(group_member)))

    # 로봇 ID 최댓값 찾기
    def FindMaxRobotID(self,robot_id):
        if self.max_id < robot_id:
            self.robot_table.setRowCount(robot_id)

    # 그룹 ID 최댓값 찾기
    def FindMaxGroupID(self,group_id):
        if self.max_group_id < group_id:
            self.robot_table.setRowCount(group_id)

    # 과업 별 이름 도시
    def TaskName(self, task_name):
        if task_name==1:
            return "고정 감시", "fixed_monitoring"
        elif task_name==2:
            return "이동 감시", "moving_monitoring"
        elif task_name==3:
            return "지도 작성", "mapping"
        elif task_name==4:
            return "진입로 찾기", "finding_gate"
        elif task_name==5:
            return "통신 중계", "relay"
        else:
            return "없는 과업", "no_task"

    def TaskParamName(self, param_name):
        if param_name=="mean_ground_velocity":
            return "평균 속도"
        elif param_name=="max_ground_velocity":
            return "최대 속도"
        elif param_name=="mean_fight_velocity":
            return "비행 평균 속도"
        elif param_name=="max_fight_velocity":
            return "비행 최대 속도"
        elif param_name=="jumping":
            return "도약 여부"
        elif param_name=="wall_stick":
            return "벽 부착 여부"
        elif param_name=="battery_spec":
            return "배터리 사양"
        elif param_name=="electronic_efficiency":
            return "전비"
        elif param_name=="observation_distance":
            return "감시 거리"
        elif param_name=="FOV":
            return "FOV"
        elif param_name=="communication_distance":
            return "통신 거리"
        elif param_name=="communication_power":
            return "통신 세기"
        else:
            return "없는 과업"

    def OnTasksTableRowClicked(self, row, col):
        # clicked_data = self.tasks_table.item(row, 0).text()
        print(str(row) + " Row Clicked")
        if self.former_row is not None:
            self.tasks_table.item(self.former_row,0).setBackground(QColor(255,255,255))
        self.former_row = row
        for i in range(self.tasks_table.columnCount(),0,-1):
            self.tasks_table.removeColumn(i)
        self.tasks_table.item(row,0).setBackground(QColor(255,192,203))
        if "정적_과업_0" in self.mp_plan:
            for key, value in self.mp_plan["정적_과업_"+str(row)].items():
                if key == '과업명' or key == 'group':
                    continue
                else:
                    print(key)
                    self.tasks_table.insertColumn(self.tasks_table.columnCount())
                    self.tasks_table.setHorizontalHeaderItem(self.tasks_table.columnCount()-1,QTableWidgetItem(key))
                    self.tasks_table.setItem(row,self.tasks_table.columnCount()-1,QTableWidgetItem(str(value)))
                    self.tasks_table.item(row,self.tasks_table.columnCount()-1).setBackground(QColor(255,192,203))

        elif "동적_과업_0" in self.mp_plan:
            pass
