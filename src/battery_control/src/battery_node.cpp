// battery_node：因克斯电池监控与控制 ROS2 节点。
//
// 话题:  /battery/status        (battery_control/msg/BatteryStatus)
// 服务:  /battery/get_status    (battery_control/srv/GetStatus)
//        /battery/force_shutdown(battery_control/srv/ForceShutdown, 需 confirm=true)
//        /battery/clear_fault   (battery_control/srv/ClearFault)
//
// 参数:  channel        (string, 默认 "can4")
//        device_address (int,    默认 0)
//        publish_rate   (double, 默认 2.0 Hz)
//        wake           (bool,   默认 true — 电池休眠时自动唤醒)
//
// 内部持有 bms::BatteryClient（复用 cpp/ 核心库，SocketCAN 直连，
// 后台线程收帧+自动唤醒），节点只做状态映射与服务封装。

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "bms_can/battery_client.h"
#include "battery_control/msg/battery_status.hpp"
#include "battery_control/srv/clear_fault.hpp"
#include "battery_control/srv/force_shutdown.hpp"
#include "battery_control/srv/get_status.hpp"

namespace {

using Msg = battery_control::msg::BatteryStatus;
using GetStatus = battery_control::srv::GetStatus;
using ForceShutdown = battery_control::srv::ForceShutdown;
using ClearFault = battery_control::srv::ClearFault;

std::vector<std::string> faultKeys(const bms::FaultBits& f) {
  std::vector<std::string> v;
  if (f.charge_forbidden) v.push_back("charge_forbidden");
  if (f.discharge_forbidden) v.push_back("discharge_forbidden");
  if (f.low_battery_warning) v.push_back("low_battery_warning");
  if (f.cont_discharge_overcur) v.push_back("cont_discharge_overcur");
  if (f.inst_discharge_overcur) v.push_back("inst_discharge_overcur");
  if (f.charge_overcurrent) v.push_back("charge_overcurrent");
  if (f.cell_overtemp) v.push_back("cell_overtemp");
  if (f.mos_overtemp) v.push_back("mos_overtemp");
  if (f.comm_lost) v.push_back("comm_lost");
  if (f.external_estop) v.push_back("external_estop");
  return v;
}

std::vector<std::string> activeCmdKeys(const bms::ActiveCmdBits& a) {
  std::vector<std::string> v;
  if (a.shutdown_request) v.push_back("shutdown_request");
  if (a.discharge_request) v.push_back("discharge_request");
  if (a.force_shutdown_bc) v.push_back("force_shutdown_broadcast");
  if (a.charge_allowed) v.push_back("charge_allowed");
  if (a.fault_stop_bc) v.push_back("fault_stop_broadcast");
  if (a.mos_state) v.push_back("mos_on");
  return v;
}

std::vector<std::string> passiveEchoKeys(const bms::PassiveEchoBits& p) {
  std::vector<std::string> v;
  if (p.force_shutdown) v.push_back("force_shutdown");
  if (p.fault_stop) v.push_back("fault_stop");
  if (p.clear_fault) v.push_back("clear_fault");
  return v;
}

Msg toMsg(const bms::DeviceState& st, uint64_t link_errors) {
  Msg m;
  m.address = st.addr;
  m.online = st.isAlive(bms::nowSecs());
  m.role = (st.status1 && st.status1->is_slave) ? "slave" : "master";
  if (st.status1) {
    m.soc = st.status1->soc;
    m.voltage = st.status1->voltage;
    m.voltage_from_heartbeat = false;
    m.discharge_current = st.status1->discharge_a;
    m.charge_current = st.status1->charge_a;
  } else if (st.heartbeat) {
    m.soc = st.heartbeat->soc;
    m.voltage = st.heartbeat->voltage;
    m.voltage_from_heartbeat = true;
  }
  if (st.status2) {
    m.cell_temperature = st.status2->cell_temp_c;
    m.mos_temperature = st.status2->mos_temp_c;
  }
  if (st.fault) m.active_faults = faultKeys(*st.fault);
  if (st.active_cmd) m.active_commands = activeCmdKeys(*st.active_cmd);
  if (st.passive_echo) m.passive_echo = passiveEchoKeys(*st.passive_echo);
  m.rx_count = st.rx_count;
  m.link_errors = static_cast<uint32_t>(link_errors);
  return m;
}

// 人类可读摘要（与 CLI 输出同风格）
std::string reportOf(const bms::DeviceState& st) {
  char buf[192];
  Msg m = toMsg(st, 0);
  std::string out = "电池[地址 " + std::to_string(st.addr) + "] " +
                    (m.online ? "在线" : "离线") + "（" + m.role + "） ";
  snprintf(buf, sizeof(buf), "SOC=%d%% 电压=%.2fV", m.soc, m.voltage);
  out += buf;
  if (m.voltage_from_heartbeat) {
    out += "（心跳帧，电流未知）";
  } else {
    snprintf(buf, sizeof(buf), " 放电=%.2fA 充电=%.2fA", m.discharge_current,
             m.charge_current);
    out += buf;
  }
  if (m.cell_temperature != 0.0f || m.mos_temperature != 0.0f) {
    snprintf(buf, sizeof(buf), " 电芯=%.1f℃ MOS=%.1f℃", m.cell_temperature,
             m.mos_temperature);
    out += buf;
  }
  std::string faults;
  for (const auto& k : m.active_faults) {
    if (!faults.empty()) faults += ",";
    faults += k;
  }
  out += " 故障: " + (faults.empty() ? std::string("无") : faults);
  snprintf(buf, sizeof(buf), "（已收 %llu 帧）",
           static_cast<unsigned long long>(st.rx_count));
  out += buf;
  return out;
}

class BatteryControlNode : public rclcpp::Node {
 public:
  explicit BatteryControlNode(const rclcpp::NodeOptions& options)
      : Node("battery_control_node", options) {
    const std::string channel = declare_parameter<std::string>("channel", "can4");
    const int addr = declare_parameter<int>("device_address", 0);
    const bool wake = declare_parameter<bool>("wake", true);
    const double rate = declare_parameter<double>("publish_rate", 2.0);

    bms::BatteryClient::Options opt;
    opt.channel = channel;
    opt.device_addr = addr;
    opt.wake = wake;
    try {
      client_ = std::make_unique<bms::BatteryClient>(opt);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "连接电池失败: %s", e.what());
      throw;  // 启动失败直接退出，避免带病运行
    }
    default_addr_ = addr;
    RCLCPP_INFO(get_logger(),
                "电池节点已启动: channel=%s addr=%d（话题 /battery/status，"
                "服务 get_status / force_shutdown / clear_fault）",
                channel.c_str(), addr);

    pub_ = create_publisher<Msg>("/battery/status", 10);

    get_status_srv_ = create_service<GetStatus>(
        "/battery/get_status",
        [this](const GetStatus::Request::SharedPtr req,
               GetStatus::Response::SharedPtr res) {
          const int addr = req->address < 0 ? default_addr_ : req->address;
          auto st = client_->state(addr);
          if (!st) {
            res->success = false;
            res->report = "未发现地址 " + std::to_string(addr) + " 的电池";
            return;
          }
          res->success = true;
          res->report = reportOf(*st);
          res->status = toMsg(*st, client_->linkErrors());
        });

    force_shutdown_srv_ = create_service<ForceShutdown>(
        "/battery/force_shutdown",
        [this](const ForceShutdown::Request::SharedPtr req,
               ForceShutdown::Response::SharedPtr res) {
          if (!req->confirm) {
            res->success = false;
            res->message =
                "已拒绝：强制关机会切断电池输出，请显式设置 confirm: true";
            res->frames_sent = 0;
            return;
          }
          RCLCPP_WARN(get_logger(), "收到强制关机指令，正在下发…");
          try {
            res->frames_sent = client_->forceShutdown();
            res->success = true;
            res->message = "强制关机指令已下发（0x04EF bit3，"
                           "持续 1s 后自动释放），请确认电池输出状态";
          } catch (const std::exception& e) {
            res->success = false;
            res->message = std::string("下发失败: ") + e.what();
          }
        });

    clear_fault_srv_ = create_service<ClearFault>(
        "/battery/clear_fault",
        [this](const ClearFault::Request::SharedPtr /*req*/,
               ClearFault::Response::SharedPtr res) {
          try {
            res->frames_sent = client_->clearFault();
            res->success = true;
            res->message = "清除故障指令已下发";
          } catch (const std::exception& e) {
            res->success = false;
            res->message = std::string("下发失败: ") + e.what();
          }
        });

    using namespace std::chrono_literals;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(rate, 0.1)));
    timer_ = create_wall_timer(period, [this] { publishStatus(); });
  }

  ~BatteryControlNode() override {
    if (client_) client_->shutdown();
  }

 private:
  void publishStatus() {
    for (const auto& [addr, st] : client_->devices()) {
      Msg m = toMsg(st, client_->linkErrors());
      pub_->publish(m);
    }
  }

  int default_addr_ = 0;
  std::unique_ptr<bms::BatteryClient> client_;
  rclcpp::Publisher<Msg>::SharedPtr pub_;
  rclcpp::Service<GetStatus>::SharedPtr get_status_srv_;
  rclcpp::Service<ForceShutdown>::SharedPtr force_shutdown_srv_;
  rclcpp::Service<ClearFault>::SharedPtr clear_fault_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // 多线程执行器：force_shutdown 下发约 1.3s，避免阻塞话题发布
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<BatteryControlNode>(rclcpp::NodeOptions());
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
