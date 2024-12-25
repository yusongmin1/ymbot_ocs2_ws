# Flattened controller: A serialized controller

# define controllerType Enum values
uint8 CONTROLLER_UNKNOWN=0 # safety mechanism: message initalization to zero
uint8 CONTROLLER_FEEDFORWARD=1
uint8 CONTROLLER_LINEAR=2

uint8                   controllerType         # 控制器状态

mpc_observation         initObservation        # 当前状态

mpc_target_trajectories planTargetTrajectories # mpc 的目标位置
mpc_state[]             stateTrajectory        # 模型预测控制预测的包含了当前一段时间内的系统状态 
mpc_input[]             inputTrajectory        # 模型预测控制预测的当前一段时间内的系统采取的输入
float64[]               timeTrajectory         # 当前一段时间内
uint16[]                postEventIndices       # array of indices indicating the index of post-event time in the trajectories
mode_schedule           modeSchedule           # 状态机变量，在四组机器人中可以使用

controller_data[]       data                   # 不知到是干啥的内容

mpc_performance_indices performanceIndices     # 评估指标，比如cost函数等
