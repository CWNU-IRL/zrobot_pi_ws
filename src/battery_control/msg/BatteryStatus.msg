# 电池实时状态（话题 /battery/status，每台电池一条消息）
# 状态帧（0x03F4/0x02F4）与心跳帧（0x18FF50F4）的数据合并；
# 遥测帧阵发时 voltage/soc 来自心跳（voltage_from_heartbeat=true）。

int32 address                    # 设备地址偏移（单电池/主机=0）
bool online                      # 2 秒内收到周期帧
string role                      # "master" / "slave"
int32 soc                        # %
float32 voltage                  # V
bool voltage_from_heartbeat      # true=电压/SOC 来自心跳帧（电流未知）
float32 discharge_current        # A（仅状态帧）
float32 charge_current           # A（仅状态帧）
float32 cell_temperature         # ℃
float32 mos_temperature          # ℃
string[] active_faults           # 激活故障键名（空=无故障）
string[] active_commands         # 主动指令（MOS状态 等）
string[] passive_echo            # BMS 回显的生效命令（空=无）
uint64 rx_count                  # 累计接收帧数
uint32 link_errors               # CAN 错误帧计数（链路质量）
