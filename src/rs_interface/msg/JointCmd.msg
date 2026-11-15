# 命令流：deploy -> bridge，QoS reliable / depth 1（只保留最新命令）
# command: "mit" | "enable" | "disable" | "stop" | "damp" | "set_zero" | "status"
# name 为空表示使用约定的 0..22 固定顺序；数组长度统一为 23。
# kp/kd 为空时由 bridge 使用 motor_config.yaml 的 motor_kps/motor_kds。
string command
builtin_interfaces/Time stamp
string[] name
float64[] position
float64[] velocity
float64[] torque
float64[] kp
float64[] kd
