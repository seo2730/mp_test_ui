ros2 topic pub --once /robots_performance_value mgs05_mp_msgs/msg/RobotPerformValues "{robot_values: [{robot_id: 1, performance_value: 35}, {robot_id: 2, performance_value: 35}], plans: [{groups: {group_id: 1}, task: [{task_name: 1}]}, {groups: {group_id: 1}, task: [{task_name: 2}]}, {groups: {group_id: 1}, task: [{task_name: 3}]}, {groups: {group_id: 1}, task: [{task_name: 4}]}, {groups: {group_id: 1}, task: [{task_name: 5}]}]}"
