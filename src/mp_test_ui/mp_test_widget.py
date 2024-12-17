#!/usr/bin/env python3
import os

from ament_index_python.resources import get_resource
### 시험용 SW ICD ###
from mgs05_mp_msgs.msg import RobotPerformValues, RobotTypeNums, GroupPerformValues, MpTestStatic, MpTestDynamic
from mgs05_base_msgs.msg import SwarmInfo, Rect, DynamicCustomPlan, Plans
from std_msgs.msg import Float32, UInt8
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
from matplotlib import rc, font_manager

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
        self.font_path = "/usr/share/fonts/truetype/msttcorefonts/arial.ttf"
        self.font_name = font_manager.FontProperties(fname=self.font_path).get_name()
        rc('font', family=self.font_name)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlim(0, 15)
        self.ax.set_ylim(0, 12)
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')

        self.ax.text(11, 13, " " ,color='white', ha='center' ,va='center', fontsize=8, bbox=dict(facecolor='black', edgecolor='none', boxstyle='circle'))
        self.ax.text(12, 13, "  Ground" ,color='black', ha='center' ,va='center', fontsize=8)

        self.ax.text(11, 12.5, "   " ,color='white', ha='center', va='center', fontsize=8, bbox=dict(facecolor='black', edgecolor='none', boxstyle='square'))
        self.ax.text(12, 12.5, "  Jumping" ,color='black', ha='center' ,va='center', fontsize=8)

        self.ax.text(14, 13, "   " ,color='white', ha='center', va='center', fontsize=8, bbox=dict(facecolor='black', edgecolor='none', boxstyle='sawtooth'))
        self.ax.text(15, 13, "  Fly" ,color='black', ha='center' ,va='center', fontsize=8)

        self.ax.text(14, 12.5, "    " ,color='white', ha='center', va='center', fontsize=8, bbox=dict(facecolor='black', edgecolor='none', boxstyle='darrow'))
        self.ax.text(15, 12.5, "  Wall" ,color='black', ha='center' ,va='center', fontsize=8)


        # QTimer로 일정 주기로 로봇의 위치 업데이트
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.UpdateSwarmInfo)
        self.timer.start(1000)  # 1초마다 업데이트

        # 로봇 위치를 표시할 선/포인트 삭제 변수
        self.current_plot = {}
        self.task_plot = {}

        self.robot_table_color={}
        self.dynamic_robot_plan_color={}
        self.dynamic_group_plan_color={}

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
        self.mp_test_static={}
        self.mp_test_dynamic={}

        ### 왜 통신 문제가 생기는겨????
        # self.sub_static_mission_plan = self.node.create_subscription(Plans, "/mission_order_r",self.GetStaticMissionPlan, best_effort)
        ###
        self.sub_mp_name = self.node.create_subscription(UInt8, "/mission_name",self.GetMissionName, best_effort)
        self.sub_dynamic_mission_plan = self.node.create_subscription(DynamicCustomPlan, "/dynamic_mission_change",self.GetDynamicMissionPlan, best_effort)
        self.sub_swarm_info = self.node.create_subscription(SwarmInfo, "/swarm_info",self.GetSwarmInfo, best_effort)
        self.sub_rect = self.node.create_subscription(Rect,"/mission_range",self.GetMissionRange, best_effort)
        self.sub_groups_performance_value = self.node.create_subscription(MpTestDynamic, "/mp_test_dynamic", self.GetGroupPerformanceValue, best_effort)
        self.sub_mp_test_static = self.node.create_subscription(MpTestStatic, "/mp_test_static", self.GetRobotPerformanceValue, best_effort)

        self.sub_mp_time = self.node.create_subscription(Float32, "/mp_time", self.GetMissionTime, best_effort)
        ####################

        self.tasks_table.cellClicked.connect(self.OnTasksTableRowClicked)

    # 할당 수행 알고리즘 수행 시간
    def GetMissionTime(self, msg):
        self.mp_time_browser.append(str(round(msg.data,4))+" s")

    # 임무명
    def GetMissionName(self, msg):
        if msg.data == 0:
            self.mp_name_browser.append("감시")
        elif msg.data == 1:
            self.mp_name_browser.append("정찰")

    # 동적임무계획 수신
    def GetDynamicMissionPlan(self, msg):
        print("@@@ Get Dynamic Plan @@@")
        self.dynamic_mission_plan = msg

    # swarm info 값 수신
    def GetSwarmInfo(self, msg):
        self.swarm_info = msg

    # 임무 범위 수신
    def GetMissionRange(self,msg):
        print("@@@ Mission Area @@@")
        width = abs(msg.end.x - msg.start.x)
        height = abs(msg.end.y - msg.start.y)
        self.mp_rect_browser.append(str(width) + " x " + str(height))

    # 로봇 성능값 수신
    def GetRobotPerformanceValue(self, msg):
        print("@@@ Get Robot Performance Value @@@")
        num_task=0
        self.mp_plan.clear()
        self.mp_test_static.clear()
        self.tasks_table.clear()
        self.tasks_table.setRowCount(len(msg.mp_test_static))
        for static_plan in msg.mp_test_static:
            self.mp_test_static["정적_과업_"+str(num_task)] = [static_plan.robot_values, static_plan.robot_type_nums,static_plan.plan]
            task_name_kor, task_name_eng = self.TaskName(static_plan.plan.task[0].task_name)
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
                param_name_dict["group"] = static_plan.plan.groups
                self.mp_plan["정적_과업_"+str(num_task)]=param_name_dict
                self.tasks_table.setColumnCount(1)
                self.tasks_table.setHorizontalHeaderLabels(["과업명"])

                for key, value in self.mp_plan.items():
                    self.tasks_table.setItem(num_task,0, QTableWidgetItem(str(value["과업명"])))

                num_task+=1
                print("@@@ END @@@")
            else:
                print("No File Exist")
                continue

    # 그룹 성능값 수신
    def GetGroupPerformanceValue(self, msg):
        print("@@@ Get Group Performance Value @@@")
        num_task=0
        self.mp_plan.clear()
        self.mp_test_dynamic.clear()
        self.tasks_table.clear()
        self.tasks_table.setRowCount(len(msg.mp_test_dynamic))
        for dynamic_plan in msg.mp_test_dynamic:
            self.mp_test_dynamic["동적_과업_"+str(num_task)] = [dynamic_plan.group_values, dynamic_plan.robot_values, dynamic_plan.robot_type_nums, dynamic_plan.plan]
            task_name_kor, task_name_eng = self.TaskName(dynamic_plan.plan.task[0].task_name)
            file_path_static=os.path.join('src/mp_test_ui/src/mp_test_ui/config',"static_task_"+task_name_eng+".yaml")
            file_path_dynamic=os.path.join('src/mp_test_ui/src/mp_test_ui/config',"dynamic_task_"+task_name_eng+".yaml")
            if os.path.isfile(file_path_dynamic) and os.path.isfile(file_path_static):
                print("Dynamic File Exist")
                with open(file_path_dynamic) as file:
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

                # # print(param_name_dict)
                param_name_dict["group"] = dynamic_plan.plan.groups
                self.mp_plan["동적_과업_그룹_"+str(num_task)]=param_name_dict
                self.tasks_table.setColumnCount(1)
                self.tasks_table.setHorizontalHeaderLabels(["과업명"])

                for key, value in self.mp_plan.items():
                    self.tasks_table.setItem(num_task,0, QTableWidgetItem(str(value["과업명"])))

                with open(file_path_static) as file:
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

                # # print(param_name_dict)
                param_name_dict["group"] = dynamic_plan.plan.groups
                self.mp_plan["동적_과업_로봇_"+str(num_task)]=param_name_dict

                num_task+=1
                print("@@@ Dynamic END @@@")
            else:
                print("No File Exist")
                continue

        for plan in msg.dynamic_plans:
            # 대기 중인 과업 표시
            task_name = []
            for task in plan.task:
                name = self.TaskName(task_name)
                task_name.append(name)

            self.group_table.setItem(plan.groups.group_id-1, 4, QTableWidgetItem(str(task_name)))

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

        if platform_id in self.robot_table_color:
            for i in range(self.robot_table.columnCount()):
                self.robot_table.item(platform_id-1,i).setBackground(self.robot_table_color[platform_id])

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
                x,y = robot_state.robot_pose.position.x, robot_state.robot_pose.position.y
                leader_id = self.UpdateRobotIdType(robot_state)
            else:
                platform_id, robot_type= self.UpdateRobotIdType(robot_state)
                group_member.append(platform_id)
            # 그룹화된 로봇 정보 도시화 여부
            # self.RobotStatusUpdateTab(robot_state)

        self.group_table.setItem(group_id-1, 0, QTableWidgetItem(str(group_id)))
        self.group_table.setItem(group_id-1, 1, QTableWidgetItem(str(leader_id)))

        self.group_table.setItem(group_id-1, 2, QTableWidgetItem('('+str(x)+', '+str(y)+')'))
        self.group_table.setItem(group_id-1, 3, QTableWidgetItem(str(current_task)))
        self.group_table.setItem(group_id-1, 4, QTableWidgetItem(''))
        self.group_table.setItem(group_id-1, 5, QTableWidgetItem(str(group_member)))

        # if platform_id in self.dynamic_robot_plan_color:
        #     for i in range(self.dynamic_robot_plan_table.columnCount()):
        #         self.dynamic_robot_plan_table.item(platform_id-1,i).setBackground(self.dynamic_robot_plan_color[platform_id])

        # if group_id in self.dynamic_group_plan_color:
        #     for i in range(self.dynamic_group_plan_table.columnCount()):
        #         self.dynamic_robot_plan_table.item(group_id-1,i).setBackground(self.dynamic_group_plan_color[group_id])

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
        elif param_name=="moving_able":
            return "이동 가능성"
        elif param_name=="task_runnig_time":
            return "과업 수행 시간"
        elif param_name=="task_progress":
            return "과업 진척도"
        elif param_name=="distance":
            return "거리"
        elif param_name=="member_num":
            return "구성원 개수"
        else:
            return "없는 파라미터"

    def RobotTableBackGroundAllWhite(self):
        if self.robot_table_color:
            for key, value in self.robot_table_color.items():
                self.robot_table_color[key] = QColor(255,255,255)

    def RobotPoseBackGroundAllBlack(self):
        for key, value in self.current_plot.items():
            self.current_plot[key].get_bbox_patch().set_facecolor('black')

    def TaskPoseReset(self):
        for key, value in self.task_plot.items():
            self.task_plot[key].set_position((-10000,-10000))

    def OnTasksTableRowClicked(self, row, col):
        # clicked_data = self.tasks_table.item(row, 0).text()
        self.RobotTableBackGroundAllWhite()
        self.RobotPoseBackGroundAllBlack()
        self.TaskPoseReset()
        self.robot_plan_table.clear()
        self.dynamic_robot_plan_table.clear()
        self.dynamic_group_plan_table.clear()
        robot_plan_table_name = ['로봇 ID', '성능값']
        group_plan_table_name = ['그룹 ID', '리더 ID', '성능값']
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

            if self.mp_test_static["정적_과업_"+str(row)][2].groups.group_id in self.task_plot:
                x = self.mp_test_static["정적_과업_"+str(row)][2].task[0].target_position[0].pose.position.x
                y = self.mp_test_static["정적_과업_"+str(row)][2].task[0].target_position[0].pose.position.y
                self.task_plot[self.mp_test_static["정적_과업_"+str(row)][2].groups.group_id].set_position((x,y))
            else:
                x = self.mp_test_static["정적_과업_"+str(row)][2].task[0].target_position[0].pose.position.x
                y = self.mp_test_static["정적_과업_"+str(row)][2].task[0].target_position[0].pose.position.y
                self.task_plot[self.mp_test_static["정적_과업_"+str(row)][2].groups.group_id] = self.ax.text(x, y, "X", weight='bold',color='green', ha='center', va='center', fontsize=15)

            self.robot_plan_table.setRowCount(len(self.mp_test_static["정적_과업_"+str(row)][0]))

            self.robot_plan_table.setColumnCount(2)
            self.robot_plan_table.setHorizontalHeaderLabels(robot_plan_table_name)
            rol = 0
            first=False
            for robot_perform_value in self.mp_test_static["정적_과업_"+str(row)][0]:
                self.robot_plan_table.setItem(rol,0,QTableWidgetItem(str(robot_perform_value.robot_id%100)))
                self.robot_plan_table.setItem(rol,1,QTableWidgetItem(str(robot_perform_value.performance_value)))
                params={}
                for param in robot_perform_value.param:
                    name = self.TaskParamName(param.param_name)
                    params[name] = param.param_value

                if first is False:
                    self.robot_plan_table.insertColumn(self.robot_plan_table.columnCount())
                    self.robot_plan_table.setHorizontalHeaderItem(self.robot_plan_table.columnCount()-1,QTableWidgetItem("거리"))
                    self.robot_plan_table.setItem(rol,self.robot_plan_table.columnCount()-1,QTableWidgetItem(str(round(params["거리"],2))))
                else:
                    self.robot_plan_table.setItem(rol,2,QTableWidgetItem(str(round(params["거리"],2))))
                j=3
                for key, value in self.mp_plan["정적_과업_"+str(row)].items():
                    if key == '과업명' or key == 'group':
                        continue
                    else:
                        if first is False:
                            self.robot_plan_table.insertColumn(self.robot_plan_table.columnCount())
                            self.robot_plan_table.setHorizontalHeaderItem(self.robot_plan_table.columnCount()-1,QTableWidgetItem(key))
                            self.robot_plan_table.setItem(rol,self.robot_plan_table.columnCount()-1,QTableWidgetItem(str(round(params[key],2))))
                        else:
                            self.robot_plan_table.setItem(rol,j,QTableWidgetItem(str(round(params[key],2))))
                            j+=1
                first=True

                for i in range(len(self.mp_test_static["정적_과업_"+str(row)][2].groups.registrations)):
                    if self.mp_test_static["정적_과업_"+str(row)][2].groups.registrations[i].platform_id == robot_perform_value.robot_id:
                        for j in range(self.robot_plan_table.columnCount()):
                            self.robot_plan_table.item(rol,j).setBackground(QColor(255,192,203))

                        self.current_plot[robot_perform_value.robot_id%100].get_bbox_patch().set_facecolor('red')
                        for i in range(self.robot_table.columnCount()):
                            # self.robot_table.item(robot_peform_value.robot_id-1,i).setBackground(QColor(255,192,203))
                            self.robot_table_color[robot_perform_value.robot_id%100] = QColor(255,192,203)
                rol += 1


        elif "동적_과업_그룹_0" in self.mp_plan:
            for key, value in self.mp_plan["동적_과업_그룹_"+str(row)].items():
                if key == '과업명' or key == 'group':
                    continue
                else:
                    self.tasks_table.insertColumn(self.tasks_table.columnCount())
                    self.tasks_table.setHorizontalHeaderItem(self.tasks_table.columnCount()-1,QTableWidgetItem(key))
                    self.tasks_table.setItem(row,self.tasks_table.columnCount()-1,QTableWidgetItem(str(value)))
                    self.tasks_table.item(row,self.tasks_table.columnCount()-1).setBackground(QColor(255,192,203))

            # Task 위치 도시화
            if self.mp_test_dynamic["동적_과업_"+str(row)][3].groups.group_id in self.task_plot:
                x = self.mp_test_dynamic["동적_과업_"+str(row)][3].task[0].target_position[0].pose.position.x
                y = self.mp_test_dynamic["동적_과업_"+str(row)][3].task[0].target_position[0].pose.position.y
                self.task_plot[self.mp_test_dynamic["동적_과업_"+str(row)][3].groups.group_id].set_position((x,y))
            else:
                x = self.mp_test_dynamic["동적_과업_"+str(row)][3].task[0].target_position[0].pose.position.x
                y = self.mp_test_dynamic["동적_과업_"+str(row)][3].task[0].target_position[0].pose.position.y
                self.task_plot[self.mp_test_dynamic["동적_과업_"+str(row)][3].groups.group_id] = self.ax.text(x, y, "X", weight='bold', color='green', ha='center', va='center', fontsize=15)

            self.dynamic_robot_plan_table.setRowCount(len(self.mp_test_dynamic["동적_과업_"+str(row)][1]))
            self.dynamic_robot_plan_table.setColumnCount(2)
            self.dynamic_robot_plan_table.setHorizontalHeaderLabels(robot_plan_table_name)
            rol = 0
            first=False
            for robot_perform_value in self.mp_test_dynamic["동적_과업_"+str(row)][1]:
                self.dynamic_robot_plan_table.setItem(rol,0,QTableWidgetItem(str(robot_perform_value.robot_id%100)))
                self.dynamic_robot_plan_table.setItem(rol,1,QTableWidgetItem(str(robot_perform_value.performance_value)))
                params={}
                for param in robot_perform_value.param:
                    name = self.TaskParamName(param.param_name)
                    params[name] = param.param_value

                if first is False:
                    self.dynamic_robot_plan_table.insertColumn(self.dynamic_robot_plan_table.columnCount())
                    self.dynamic_robot_plan_table.setHorizontalHeaderItem(self.dynamic_robot_plan_table.columnCount()-1,QTableWidgetItem("거리"))
                    self.dynamic_robot_plan_table.setItem(rol,self.dynamic_robot_plan_table.columnCount()-1,QTableWidgetItem(str(round(params["거리"],2))))
                else:
                    self.dynamic_robot_plan_table.setItem(rol,2,QTableWidgetItem(str(round(params["거리"],2))))
                j=3
                for key, value in self.mp_plan["동적_과업_로봇_"+str(row)].items():
                    if key == '과업명' or key == 'group':
                        continue
                    else:
                        if first is False:
                            self.dynamic_robot_plan_table.insertColumn(self.dynamic_robot_plan_table.columnCount())
                            self.dynamic_robot_plan_table.setHorizontalHeaderItem(self.dynamic_robot_plan_table.columnCount()-1,QTableWidgetItem(key))
                            self.dynamic_robot_plan_table.setItem(rol,self.dynamic_robot_plan_table.columnCount()-1,QTableWidgetItem(str(round(params[key],2))))
                        else:
                            self.dynamic_robot_plan_table.setItem(rol,j,QTableWidgetItem(str(round(params[key],2))))
                            j+=1

                first=True
                for group in self.mp_test_dynamic["동적_과업_"+str(row)][0]:
                    for i in range(len(group.registrations)):
                        if group.registrations[i].platform_id == robot_perform_value.robot_id:
                            for j in range(self.dynamic_robot_plan_table.columnCount()):
                                self.dynamic_robot_plan_table.item(rol,j).setBackground(QColor(255,192,203))
                            self.current_plot[robot_perform_value.robot_id%100].get_bbox_patch().set_facecolor('red')
                            for i in range(self.dynamic_robot_plan_table.columnCount()):
                                # self.robot_table.item(robot_peform_value.robot_id-1,i).setBackground(QColor(255,192,203))
                                self.dynamic_robot_plan_color[robot_perform_value.robot_id%100] = QColor(255,192,203)
                rol += 1

            self.dynamic_group_plan_table.setRowCount(len(self.mp_test_dynamic["동적_과업_"+str(row)][1]))
            self.dynamic_group_plan_table.setColumnCount(3)
            self.dynamic_group_plan_table.setHorizontalHeaderLabels(group_plan_table_name)
            rol = 0
            first=False
            for group_perform_value in self.mp_test_dynamic["동적_과업_"+str(row)][0]:
                self.dynamic_group_plan_table.setItem(rol,0,QTableWidgetItem(str(group_perform_value.group_id)))
                self.dynamic_group_plan_table.setItem(rol,1,QTableWidgetItem(str(group_perform_value.leader_id%100)))
                self.dynamic_group_plan_table.setItem(rol,2,QTableWidgetItem(str(group_perform_value.performance_value)))
                params={}
                for param in group_perform_value.param:
                    name = self.TaskParamName(param.param_name)
                    params[name] = param.param_value

                if first is False:
                    self.dynamic_group_plan_table.insertColumn(self.dynamic_group_plan_table.columnCount())
                    self.dynamic_group_plan_table.setHorizontalHeaderItem(self.dynamic_group_plan_table.columnCount()-1,QTableWidgetItem("거리"))
                    self.dynamic_group_plan_table.setItem(rol,self.dynamic_group_plan_table.columnCount()-1,QTableWidgetItem(str(round(params["거리"],2))))
                else:
                    self.dynamic_group_plan_table.setItem(rol,3,QTableWidgetItem(str(round(params["거리"],2))))
                j=4
                for key, value in self.mp_plan["동적_과업_그룹_"+str(row)].items():
                    if key == '과업명' or key == 'group':
                        continue
                    else:
                        if first is False:
                            self.dynamic_group_plan_table.insertColumn(self.dynamic_group_plan_table.columnCount())
                            self.dynamic_group_plan_table.setHorizontalHeaderItem(self.dynamic_group_plan_table.columnCount()-1,QTableWidgetItem(key))
                            self.dynamic_group_plan_table.setItem(rol,self.dynamic_group_plan_table.columnCount()-1,QTableWidgetItem(str(round(params[key],2))))
                        else:
                            self.dynamic_group_plan_table.setItem(rol,j,QTableWidgetItem(str(round(params[key],2))))
                            j+=1
                first=True
                for i in range(len(self.mp_test_dynamic["동적_과업_"+str(row)][3].groups.registrations)):
                    if self.mp_test_dynamic["동적_과업_"+str(row)][3].groups.group_id == group_perform_value.group_id:
                        for j in range(self.dynamic_group_plan_table.columnCount()):
                            self.dynamic_group_plan_table.item(rol,j).setBackground(QColor(255,192,203))
                        self.current_plot[group_perform_value.leader_id%100].get_bbox_patch().set_facecolor('orange')
                        for i in range(self.dynamic_robot_plan_table.columnCount()):
                            # self.robot_table.item(robot_peform_value.robot_id-1,i).setBackground(QColor(255,192,203))
                            self.dynamic_group_plan_color[group_perform_value.group_id] = QColor(255,192,203)
                rol += 1

        for col in range(self.robot_plan_table.columnCount()):
            self.robot_plan_table.resizeColumnToContents(col)

        for col in range(self.dynamic_robot_plan_table.columnCount()):
            self.dynamic_robot_plan_table.resizeColumnToContents(col)

        for col in range(self.dynamic_group_plan_table.columnCount()):
            self.dynamic_group_plan_table.resizeColumnToContents(col)
