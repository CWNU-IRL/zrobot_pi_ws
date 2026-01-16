#include "zrobot_deploy/FSM.h"
#include "zrobot_deploy/FixStand.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// 键盘输入相关函数
int getch()
{
    int ch;
    struct termios oldt, newt;
    
    // 获取当前终端设置
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    
    // 设置为非规范模式，不回显
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    
    ch = getchar();
    
    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    
    return ch;
}

int kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    
    ch = getchar();
    
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    
    if(ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    
    return 0;
}

void printMenu()
{;
    std::cout << "========================================\n";
    std::cout << "  [F] - 启动 FixStand 状态机 (移动到机械零位)\n";
    std::cout << "  [S] - 停止当前状态机\n";
    std::cout << "  [Q] - 退出程序\n";
    std::cout << "========================================\n\n";
}

int main(int argc, char** argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<rclcpp::Node>("zrobot_fsm_controller");
    
    RCLCPP_INFO(node->get_logger(), "ZRobot FSM Controller started");
    
    // 当前运行的状态机
    std::shared_ptr<FSM> current_fsm = nullptr;
    
    // 控制频率 (100 Hz)
    rclcpp::Rate rate(100);
    
    // 显示菜单
    printMenu();
    
    bool running = true;
    
    while (rclcpp::ok() && running) {
        // 检查键盘输入
        if (kbhit()) {
            int key = getch();
            
            switch (key) {
                case 'f':
                case 'F':
                {
                    // 如果有正在运行的状态机，先退出
                    if (current_fsm) {
                        RCLCPP_INFO(node->get_logger(), "Stopping current FSM...");
                        current_fsm->exit();
                        current_fsm.reset();  // 重置智能指针
                    }
                    
                    // 创建并初始化 FixStand 状态机
                    RCLCPP_INFO(node->get_logger(), "Starting FixStand FSM...");
                    current_fsm = std::make_shared<FixStand>(node);
                    current_fsm->initialize();
                    
                    std::cout << "\n>>> FixStand 状态机已启动 <<<\n\n";
                    break;
                }
                
                case 's':
                case 'S':
                {
                    // 停止当前状态机
                    if (current_fsm) {
                        RCLCPP_INFO(node->get_logger(), "Stopping current FSM...");
                        current_fsm->exit();
                        current_fsm.reset();
                        std::cout << "\n>>> 状态机已停止 <<<\n\n";
                    } else {
                        std::cout << "\n>>> 没有正在运行的状态机 <<<\n\n";
                    }
                    break;
                }
                
                case 'q':
                case 'Q':
                {
                    // 退出程序
                    RCLCPP_INFO(node->get_logger(), "Exiting program...");
                    if (current_fsm) {
                        current_fsm->exit();
                        current_fsm.reset();
                    }
                    running = false;
                    break;
                }
                
                default:
                    std::cout << "\n未知命令，请使用 F/S/Q\n";
                    break;
            }
        }
        
        // 运行当前状态机
        if (current_fsm) {
            current_fsm->run();
        }
        
        // ROS2 spin once
        rclcpp::spin_some(node);
        
        // 控制循环频率，根据循环实际耗时自动计算休眠时间
        rate.sleep();
    }
    
    RCLCPP_INFO(node->get_logger(), "ZRobot FSM Controller stopped");
    
    // 关闭ROS2
    rclcpp::shutdown();
    
    return 0;
}

