#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <chrono>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <erob_master/srv/configure_motor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// CANopen COB-ID基础值
#define COB_NMT      0x000
#define COB_SYNC     0x080

#define COB_RSDO     0x600
#define COB_TSDO     0x580

#define COB_RPDO1    0x200
#define COB_RPDO2    0x300
#define COB_RPDO3    0x400
#define COB_RPDO4    0x500

#define COB_TPDO1    0x180
#define COB_TPDO2    0x280
#define COB_TPDO3    0x380
#define COB_TPDO4    0x480

// NMT命令
#define NMT_START_REMOTE_NODE    0x01
#define NMT_STOP_REMOTE_NODE     0x02
#define NMT_RESET_NODE           0x81
#define NMT_RESET_COMM           0x82

// CiA402控制字
#define CONTROL_SHUTDOWN            0x06
#define CONTROL_SWITCH_ON           0x07
#define CONTROL_ENABLE_OPERATION    0x0F
#define CONTROL_DISABLE_VOLTAGE     0x00
#define CONTROL_FAULT_RESET         0x80
#define CONTROL_NEW_SET_POINT       0x10  // Bit 4 for new set point

// CiA402操作模式
#define MODE_PROFILE_POSITION       1
#define MODE_VELOCITY               2
#define MODE_PROFILE_VELOCITY       3
#define MODE_PROFILE_TORQUE         4
#define MODE_CYCLIC_POSITION        8
#define MODE_CYCLIC_VELOCITY        9
#define MODE_CYCLIC_TORQUE          10

// 对象字典索引
#define OD_CONTROL_WORD             0x6040
#define OD_STATUS_WORD              0x6041
#define OD_OPERATION_MODE           0x6060
#define OD_OPERATION_MODE_DISPLAY   0x6061

//PP
#define OD_TARGET_POSITION          0x607A
#define OD_ACTUAL_POSITION          0x6064
#define OD_PROFILE_VELOCITY         0x6081
#define OD_PROFILE_ACCELERATION     0x6083
#define OD_PROFILE_DECELERATION     0x6084

//PV
#define OD_TARGET_VELOCITY          0x60FF
#define OD_ACTUAL_VELOCITY          0x606C
#define OD_MAX_PROFILE_VELOCITY     0x607F

//PT
#define OD_TARGET_TORQUE            0x6071
#define OD_ACTUAL_TORQUE            0x6077
#define OD_MAX_PROFILE_TORQUE       0x6072
#define OD_TORQUE_OFFSET            0x60B2
#define OD_TORQUE_SLOPE             0x6087

#define OD_SYNC_MANAGER             0x1006

// 编码器分辨率
#define ENCODER_RESOLUTION       524288

class CANopenROS2 : public rclcpp::Node
{
public:
    CANopenROS2() : Node("simple_erob_control")
    {
        // 初始化参数
        can_interface_ = "can0";
        node_id_ = 6;
        
        RCLCPP_INFO(this->get_logger(), "初始化Simple eRob Control，CAN接口=%s，节点ID=%d", 
                    can_interface_.c_str(), node_id_);
        
        // 初始化CAN套接字
        init_can_socket();
        
        if (can_socket_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "CAN套接字初始化失败，无法继续");
            return;
        }
        
        // 创建定时器，用于接收CAN帧
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&CANopenROS2::receive_can_frames, this));
        
        // 初始化节点
        initialize_node(node_id_);
        
        // 配置PDO映射
        configure_pdo(node_id_);
        
        // 等待一段时间
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 启动节点
        start_node(node_id_);
        
        // 设置立即生效
        set_immediate_effect(node_id_, true);
        
        // 清除故障
        clear_fault(node_id_);
        
        // 使能电机
        enable_motor(node_id_);
        
        // 设置目标位置（例如，移动到90度）
        // go_to_position(0.0);
        
        // 创建状态定时器
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&CANopenROS2::publish_status, this));
        
        // 创建发布器
        status_pub_ = this->create_publisher<std_msgs::msg::String>("erob_status", 10);
        position_pub_ = this->create_publisher<std_msgs::msg::Float32>("erob_position", 10);
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("erob_velocity", 10);
        
        // 创建订阅器
        // position_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        //     "target_position", 10, std::bind(&CANopenROS2::position_callback, this, std::placeholders::_1));
        position_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "target_position", 10, std::bind(&CANopenROS2::position_callback, this, std::placeholders::_1));
        velocity_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "target_velocity", 10, std::bind(&CANopenROS2::velocity_callback, this, std::placeholders::_1));
        
        // 创建服务
        start_service_ = this->create_service<erob_master::srv::ConfigureMotor>(
            "start_erob", std::bind(&CANopenROS2::handle_start, this, std::placeholders::_1, std::placeholders::_2));
        stop_service_ = this->create_service<erob_master::srv::ConfigureMotor>(
            "stop_erob", std::bind(&CANopenROS2::handle_stop, this, std::placeholders::_1, std::placeholders::_2));
        reset_service_ = this->create_service<erob_master::srv::ConfigureMotor>(
            "reset_erob", std::bind(&CANopenROS2::handle_reset, this, std::placeholders::_1, std::placeholders::_2));
        set_mode_service_ = this->create_service<erob_master::srv::ConfigureMotor>(
            "set_erob_mode", std::bind(&CANopenROS2::handle_set_mode, this, std::placeholders::_1, std::placeholders::_2));
    }
    
    ~CANopenROS2()
    {
        // 停止电机
        stop_motor(node_id_);
        
        // 关闭CAN套接字
        if (can_socket_ >= 0)
        {
            close(can_socket_);
        }
    }
    
private:
    void init_can_socket()
    {
        // 创建套接字
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "无法创建CAN套接字");
            return;
        }
        
        // 获取接口索引
        struct ifreq ifr;
        strcpy(ifr.ifr_name, can_interface_.c_str());
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "无法获取CAN接口索引");
            close(can_socket_);
            can_socket_ = -1;
            return;
        }
        
        // 绑定套接字
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "无法绑定CAN套接字");
            close(can_socket_);
            can_socket_ = -1;
            return;
        }
        
        // 设置非阻塞模式
        int flags = fcntl(can_socket_, F_GETFL, 0);
        if (flags < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "无法获取套接字标志");
            close(can_socket_);
            can_socket_ = -1;
            return;
        }
        
        flags |= O_NONBLOCK;
        if (fcntl(can_socket_, F_SETFL, flags) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "无法设置非阻塞模式");
            close(can_socket_);
            can_socket_ = -1;
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "CAN套接字初始化成功");
    }
    
    void initialize_node(int node_id)
    {
        // 发送NMT停止命令
        send_nmt_command(node_id, NMT_STOP_REMOTE_NODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 发送NMT重置命令
        send_nmt_command(node_id, NMT_RESET_NODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 先使能电机，再设置操作模式
        // 关闭（Shutdown）
        write_sdo(node_id, OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 准备开启（Switch on）
        write_sdo(node_id, OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 使能操作（Enable operation）
        write_sdo(node_id, OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 读取状态字，确认电机已使能
        int32_t status_word = read_sdo(node_id, OD_STATUS_WORD, 0x00);
        RCLCPP_INFO(this->get_logger(), "使能后状态字: 0x%04X", status_word);
        
        // 现在尝试设置操作模式
        write_sdo(node_id, OD_OPERATION_MODE, 0x00, MODE_PROFILE_POSITION, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 验证操作模式
        int32_t mode = read_sdo(node_id, OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "当前操作模式: %d", mode);
        
        // 如果操作模式仍然不是位置模式，尝试使用PDO设置
        if (mode != MODE_PROFILE_POSITION)
        {
            RCLCPP_WARN(this->get_logger(), "使用SDO设置操作模式失败，尝试使用PDO");
            
            // 使用PDO设置操作模式
            struct can_frame frame;
            frame.can_id = COB_RPDO1 + node_id_;
            frame.can_dlc = 3;  // 控制字(2字节) + 操作模式(1字节)
            frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;  // 控制字低字节
            frame.data[1] = (CONTROL_ENABLE_OPERATION >> 8) & 0xFF;  // 控制字高字节
            frame.data[2] = MODE_PROFILE_POSITION;  // 操作模式
            
            if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
            {
                RCLCPP_ERROR(this->get_logger(), "发送PDO操作模式失败");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "PDO操作模式已发送");
            }
            
            send_sync_frame();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            
            // 再次验证操作模式
            mode = read_sdo(node_id, OD_OPERATION_MODE_DISPLAY, 0x00);
            RCLCPP_INFO(this->get_logger(), "PDO设置后操作模式: %d", mode);
        }
        
        // 设置轮廓速度
        set_profile_velocity(node_id,5);
        
        // 设置轮廓加速度
        set_profile_acceleration(node_id, 5);
        
        // 设置轮廓减速度
        set_profile_deceleration(node_id,5);
        
        // 禁用同步生成器
        write_sdo(node_id, OD_SYNC_MANAGER, 0x00, 0, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 设置通信周期为1000微秒
        write_sdo(node_id, OD_SYNC_MANAGER, 0x00, 1000, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        RCLCPP_INFO(this->get_logger(), "节点初始化完成");
    }
    
    void configure_pdo(int node_id)
    {
        RCLCPP_INFO(this->get_logger(), "配置PDO映射...");
        
        // 进入预操作状态
        send_nmt_command(node_id, NMT_STOP_REMOTE_NODE);
        RCLCPP_INFO(this->get_logger(), "已进入预操作状态");
        
        // 配置TxPDO1
        RCLCPP_INFO(this->get_logger(), "开始配置TxPDO1");
        
        // 1. 禁用TxPDO1
        uint32_t txpdo1_cob_id = COB_TPDO1 + node_id;
        write_sdo(node_id, 0x1800, 0x01, txpdo1_cob_id | 0x80000000, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 2. 设置传输类型
        write_sdo(node_id, 0x1800, 0x02, 0x01, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 3. 清除TxPDO1映射
        write_sdo(node_id, 0x1A00, 0x00, 0x00, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 4. 设置映射对象：状态字
        write_sdo(node_id, 0x1A00, 0x01, 0x60410010, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 5. 设置映射对象：实际位置
        write_sdo(node_id, 0x1A00, 0x02, 0x60640020, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 6. 设置TxPDO1映射对象数量为2
        write_sdo(node_id, 0x1A00, 0x00, 0x02, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 7. 设置传输类型并启用TxPDO1
        write_sdo(node_id, 0x1800, 0x02, 0xFF, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        write_sdo(node_id, 0x1800, 0x01, txpdo1_cob_id, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 配置RxPDO1
        RCLCPP_INFO(this->get_logger(), "开始配置RxPDO1");
        
        // 1. 禁用RxPDO1
        uint32_t rxpdo1_cob_id = COB_RPDO1 + node_id;
        write_sdo(node_id, 0x1400, 0x01, rxpdo1_cob_id | 0x80000000, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 2. 设置传输类型
        write_sdo(node_id, 0x1400, 0x02, 0x01, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 3. 清除RxPDO1映射
        write_sdo(node_id, 0x1600, 0x00, 0x00, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 4. 设置映射对象：控制字
        write_sdo(node_id, 0x1600, 0x01, 0x60400010, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 5. 设置映射对象：目标位置
        write_sdo(node_id, 0x1600, 0x02, 0x607A0020, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 6. 设置RxPDO1映射对象数量为2
        write_sdo(node_id, 0x1600, 0x00, 0x02, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 7. 设置传输类型并启用RxPDO1
        write_sdo(node_id, 0x1400, 0x02, 0xFF, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        write_sdo(node_id, 0x1400, 0x01, rxpdo1_cob_id, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        RCLCPP_INFO(this->get_logger(), "PDO配置完成");
    }
    
    void start_node(int node_id)
    {
        RCLCPP_INFO(this->get_logger(), "启动节点...");
        
        // 发送NMT启动命令
        send_nmt_command(node_id, NMT_START_REMOTE_NODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 获取实际位置
        int32_t actual_position = read_sdo(node_id, OD_ACTUAL_POSITION, 0x00);
        float actual_angle = position_to_angle(actual_position);
        RCLCPP_INFO(this->get_logger(), "实际位置: %.2f°", actual_angle);
        
        // 发送同步帧
        send_sync_frame();
        
        RCLCPP_INFO(this->get_logger(), "节点启动完成");
    }
    
    void set_immediate_effect(int node_id, bool immediate)
    {
        RCLCPP_INFO(this->get_logger(), "设置%s效果", immediate ? "立即" : "非立即");
        
        // 读取当前控制字
        int32_t controlword = read_sdo(node_id, OD_CONTROL_WORD, 0x00);
        
        if (immediate)
        {
            controlword |= (1 << 5);  // 设置位5为1（立即生效）
        }
        else
        {
            controlword &= ~(1 << 5);  // 设置位5为0（非立即生效）
        }
        
        // 写入新的控制字
        write_sdo(node_id, OD_CONTROL_WORD, 0x00, controlword, 2);
        
        RCLCPP_INFO(this->get_logger(), "控制字已更新为: 0x%04X", controlword);
    }
    
    void clear_fault(int node_id)
    {
        RCLCPP_INFO(this->get_logger(), "清除故障...");
        
        // 发送故障复位命令
        set_control_word(node_id, CONTROL_FAULT_RESET);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 进入就绪状态
        set_control_word(node_id, CONTROL_SHUTDOWN);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        RCLCPP_INFO(this->get_logger(), "故障已清除");
    }
    
    void enable_motor(int node_id)
    {
        RCLCPP_INFO(this->get_logger(), "使能电机...");
        
        // 读取当前状态字
        int32_t status_word = read_sdo(node_id, OD_STATUS_WORD, 0x00);
        RCLCPP_INFO(this->get_logger(), "当前状态字: 0x%04X", status_word);
        
        // 先使用SDO设置控制字
        // 关闭（Shutdown）
        write_sdo(node_id, OD_CONTROL_WORD, 0x00, CONTROL_SHUTDOWN, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 准备开启（Switch on）
        write_sdo(node_id, OD_CONTROL_WORD, 0x00, CONTROL_SWITCH_ON, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 使能操作（Enable operation）
        write_sdo(node_id, OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 再次读取状态字，确认电机已使能
        status_word = read_sdo(node_id, OD_STATUS_WORD, 0x00);
        RCLCPP_INFO(this->get_logger(), "使能后状态字: 0x%04X", status_word);
        
        // 然后使用PDO发送控制字
        set_control_word(node_id, CONTROL_SHUTDOWN);  // 关机
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        set_control_word(node_id, CONTROL_SWITCH_ON);  // 开启
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        set_control_word(node_id, CONTROL_ENABLE_OPERATION);  // 使能操作
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        RCLCPP_INFO(this->get_logger(), "电机已使能");
    }
    
    void stop_motor(int node_id)
    {
        RCLCPP_INFO(this->get_logger(), "停止电机...");
        
        // 设置目标速度为0
        set_target_velocity(node_id, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 禁用操作
        set_control_word(node_id, CONTROL_SWITCH_ON);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 关闭电机
        set_control_word(node_id, CONTROL_SHUTDOWN);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 禁用电压
        set_control_word(node_id, CONTROL_DISABLE_VOLTAGE);
        
        RCLCPP_INFO(this->get_logger(), "电机已停止");
    }
    
    void send_nmt_command(int node_id, uint8_t command)
    {
        struct can_frame frame;
        frame.can_id = COB_NMT;
        frame.can_dlc = 2;
        frame.data[0] = command;
        frame.data[1] = node_id;
        
        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            RCLCPP_ERROR(this->get_logger(), "发送NMT命令失败");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "NMT命令已发送: 0x%02X", command);
        }
    }
    
    void set_operation_mode(int node_id, uint8_t mode)
    {
        RCLCPP_INFO(this->get_logger(), "开始切换操作模式到: %d", mode);
        
        // 步骤1: 先读取当前状态字
        int32_t status_word = read_sdo(node_id, OD_STATUS_WORD, 0x00);
        RCLCPP_INFO(this->get_logger(), "当前状态字: 0x%04X", status_word);
        
        // 步骤2: 禁用电机操作
        RCLCPP_INFO(this->get_logger(), "禁用电机操作");
        write_sdo(node_id_, OD_CONTROL_WORD, 0x00, 0x0000, 2);  // 禁用电压
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 步骤3: 尝试设置操作模式
        
        // 使用标准的操作模式对象 (0x6060)
        RCLCPP_INFO(this->get_logger(), "使用标准操作模式对象 (0x6060)");
        write_sdo(node_id_, OD_OPERATION_MODE, 0x00, mode, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // 检查操作模式
        int32_t current_mode = read_sdo(node_id, OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "设置后的操作模式: %d", current_mode);
        
        // 如果操作模式设置失败，尝试使用PDO
        if (current_mode != mode)
        {
            // 尝试使用PDO设置操作模式
            RCLCPP_INFO(this->get_logger(), "尝试使用PDO设置操作模式");
            
            struct can_frame frame;
            frame.can_id = COB_RPDO1 + node_id;
            frame.can_dlc = 3;  // 控制字(2字节) + 操作模式(1字节)
            frame.data[0] = 0x00;  // 控制字低字节 (禁用)
            frame.data[1] = 0x00;  // 控制字高字节
            frame.data[2] = mode;  // 操作模式
            
            if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
            {
                RCLCPP_ERROR(this->get_logger(), "发送PDO操作模式失败");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "PDO操作模式已发送: %d", mode);
            }
            
            send_sync_frame();
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            
            // 再次检查操作模式
            current_mode = read_sdo(node_id, OD_OPERATION_MODE_DISPLAY, 0x00);
            RCLCPP_INFO(this->get_logger(), "PDO设置后的操作模式: %d", current_mode);
        }
        
        // 步骤4: 重新使能电机
        RCLCPP_INFO(this->get_logger(), "重新使能电机");
        
        // 状态机转换: 禁用电压 -> 准备好切换开启 -> 切换开启 -> 操作使能
        write_sdo(node_id, OD_CONTROL_WORD, 0x00, 0x0006, 2);  // 关机
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        write_sdo(node_id, OD_CONTROL_WORD, 0x00, 0x0007, 2);  // 切换开启
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        write_sdo(node_id, OD_CONTROL_WORD, 0x00, 0x000F, 2);  // 使能操作
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 步骤5: 最终确认状态
        status_word = read_sdo(node_id, OD_STATUS_WORD, 0x00);
        RCLCPP_INFO(this->get_logger(), "最终状态字: 0x%04X", status_word);
        
        current_mode = read_sdo(node_id, OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "最终操作模式: %d", current_mode);
        
        if (current_mode == mode)
        {
            RCLCPP_INFO(this->get_logger(), "操作模式切换成功: %d", mode);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "操作模式切换失败，当前模式: %d, 期望模式: %d", 
                        current_mode, mode);
            
            // 如果所有方法都失败，我们可能需要查看电机的文档
            RCLCPP_ERROR(this->get_logger(), "无法切换操作模式。请查看电机文档，了解正确的操作模式切换方法。");
            
            // 尝试直接使用当前模式
            RCLCPP_INFO(this->get_logger(), "尝试使用当前模式: %d", current_mode);
        }
    }
    
    void set_profile_velocity(int node_id, int32_t velocity_rpm)
    {
        int32_t velocity_pulse = velocity_to_pulse(velocity_rpm);
        write_sdo(node_id, OD_PROFILE_VELOCITY, 0x00, velocity_pulse, 4);
        RCLCPP_INFO(this->get_logger(), "轮廓速度已设置: %d°/s", velocity_rpm);
    }
    
    void set_profile_acceleration(int node_id, int32_t acceleration_rpm2)
    {
        int32_t acceleration_pulse = acceleration_to_pulse(acceleration_rpm2);
        write_sdo(node_id, OD_PROFILE_ACCELERATION, 0x00, acceleration_pulse, 4);
        RCLCPP_INFO(this->get_logger(), "轮廓加速度已设置: %d°/s²", acceleration_rpm2);
    }
    
    void set_profile_deceleration(int node_id, int32_t deceleration_rpm2)
    {
        int32_t deceleration_pulse = acceleration_to_pulse(deceleration_rpm2);
        write_sdo(node_id, OD_PROFILE_DECELERATION, 0x00, deceleration_pulse, 4);
        RCLCPP_INFO(this->get_logger(), "轮廓减速度已设置: %d°/s²", deceleration_rpm2);
    }
    
    void set_profile_parameters(int node_id, int32_t velocity_rpm, int32_t acceleration_rpm2, int32_t deceleration_rpm2)
    {
        set_profile_velocity(node_id, velocity_rpm);
        set_profile_acceleration(node_id, acceleration_rpm2);
        set_profile_deceleration(node_id, deceleration_rpm2);
        RCLCPP_INFO(this->get_logger(), "轮廓参数设置完成 - 速度: %d°/s, 加速度: %d°/s², 减速度: %d°/s²", 
                   velocity_rpm, acceleration_rpm2, deceleration_rpm2);
    }
    
    void set_control_word(int node_id, uint16_t control_word)
    {
        // 使用PDO发送控制字
        struct can_frame frame;
        frame.can_id = COB_RPDO1 + node_id;
        frame.can_dlc = 2;
        frame.data[0] = control_word & 0xFF;  // 控制字低字节
        frame.data[1] = (control_word >> 8) & 0xFF;  // 控制字高字节
        
        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            RCLCPP_ERROR(this->get_logger(), "发送控制字失败");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "控制字已发送: 0x%04X", control_word);
        }
    }
    
    void set_target_velocity(int node_id, int32_t velocity)
    {
        write_sdo(node_id, OD_TARGET_POSITION, 0x00, velocity, 4);
        RCLCPP_INFO(this->get_logger(), "目标速度已设置: %d", velocity);
    }
    
    void go_to_position(int node_id, float angle)
    {
        RCLCPP_INFO(this->get_logger(), "移动到位置: %.2f°", angle);
        
        int32_t position = angle_to_position(angle);
        RCLCPP_INFO(this->get_logger(), "目标位置脉冲值: %d", position);
        
        // 先使用SDO设置目标位置
        write_sdo(node_id, OD_TARGET_POSITION, 0x00, position, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 使用SDO设置控制字，触发位置命令
        write_sdo(node_id, OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION | CONTROL_NEW_SET_POINT, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 重置控制字
        write_sdo(node_id, OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 然后使用PDO发送目标位置
        struct can_frame frame;
        frame.can_id = COB_RPDO1 + node_id;
        frame.can_dlc = 6;  // 控制字(2字节) + 目标位置(4字节)
        frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;  // 控制字低字节
        frame.data[1] = (CONTROL_ENABLE_OPERATION >> 8) & 0xFF;  // 控制字高字节
        frame.data[2] = position & 0xFF;  // 目标位置低字节
        frame.data[3] = (position >> 8) & 0xFF;
        frame.data[4] = (position >> 16) & 0xFF;
        frame.data[5] = (position >> 24) & 0xFF;  // 目标位置高字节
        
        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            RCLCPP_ERROR(this->get_logger(), "发送目标位置失败");
            return;
        }
        
        send_sync_frame();
        
        // 先重置命令触发位（位4）
        set_control_word(node_id, CONTROL_ENABLE_OPERATION);
        send_sync_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 设置命令触发位，创建上升沿
        set_control_word(node_id, CONTROL_ENABLE_OPERATION | CONTROL_NEW_SET_POINT);
        send_sync_frame();
        
        RCLCPP_INFO(this->get_logger(), "位置命令已发送");
        
        // 监控目标位置是否到达
        int retry = 0;
        while (retry < 10)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            
            // 读取状态字
            int32_t status_word = read_sdo(node_id, OD_STATUS_WORD, 0x00);
            
            // 检查目标到达位（位10）
            if (status_word & 0x0400)
            {
                RCLCPP_INFO(this->get_logger(), "目标位置已到达");
                break;
            }
            
            retry++;
        }
        
        if (retry >= 10)
        {
            RCLCPP_WARN(this->get_logger(), "等待目标位置到达超时");
        }
    }
    
    void send_sync_frame()
    {
        struct can_frame frame;
        frame.can_id = COB_SYNC;
        frame.can_dlc = 0;
        
        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            RCLCPP_ERROR(this->get_logger(), "发送同步帧失败");
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "同步帧已发送");
        }
    }
    
    void write_sdo(int node_id, uint16_t index, uint8_t subindex, int32_t data, uint8_t size)
    {
        struct can_frame frame;
        frame.can_id = COB_RSDO + node_id;
        frame.can_dlc = 8;
        
        // 命令字节
        uint8_t command = 0x22;  // 下载请求
        if (size == 1)
        {
            command |= 0x0F;  // 1字节
        }
        else if (size == 2)
        {
            command |= 0x0B;  // 2字节
        }
        else if (size == 4)
        {
            command |= 0x03;  // 4字节
        }
        
        frame.data[0] = command;
        frame.data[1] = index & 0xFF;  // 索引低字节
        frame.data[2] = (index >> 8) & 0xFF;  // 索引高字节
        frame.data[3] = subindex;  // 子索引
        frame.data[4] = data & 0xFF;  // 数据低字节
        frame.data[5] = (data >> 8) & 0xFF;
        frame.data[6] = (data >> 16) & 0xFF;
        frame.data[7] = (data >> 24) & 0xFF;  // 数据高字节
        
        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            RCLCPP_ERROR(this->get_logger(), "写入SDO失败: 索引=0x%04X, 子索引=0x%02X", index, subindex);
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "SDO已写入: 索引=0x%04X, 子索引=0x%02X, 数据=0x%08X", index, subindex, data);
        }
    }
    
    int32_t read_sdo(int node_id, uint16_t index, uint8_t subindex)
    {
        struct can_frame frame;
        frame.can_id = COB_RSDO + node_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x40;  // 上传请求
        frame.data[1] = index & 0xFF;  // 索引低字节
        frame.data[2] = (index >> 8) & 0xFF;  // 索引高字节
        frame.data[3] = subindex;  // 子索引
        frame.data[4] = 0;
        frame.data[5] = 0;
        frame.data[6] = 0;
        frame.data[7] = 0;
        
        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            RCLCPP_ERROR(this->get_logger(), "读取SDO请求失败: 索引=0x%04X, 子索引=0x%02X", index, subindex);
            return -1;
        }
        
        // 等待响应
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 注意：这里我们没有实际等待和处理响应
        // 在实际应用中，应该在receive_can_frames函数中处理SDO响应
        // 并返回读取到的值
        
        return 0;  // 这里简化处理，实际应该返回读取到的值
    }
    
    void receive_can_frames()
    {
        struct can_frame frame;
        ssize_t nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
        
        if (nbytes < 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
                RCLCPP_ERROR(this->get_logger(), "接收CAN帧失败: %s", strerror(errno));
            }
            return;
        }
        
        // 处理接收到的CAN帧
        uint32_t cob_id = frame.can_id & 0x780;  // 提取功能码
        uint8_t node_id = frame.can_id & 0x7F;  // 提取节点ID
        
        if (node_id != node_id_)
        {
            return;  // 不是我们关心的节点
        }
        
        RCLCPP_DEBUG(this->get_logger(), "接收到CAN帧: ID=0x%03X, DLC=%d, Data=0x%02X%02X%02X%02X%02X%02X%02X%02X",
            frame.can_id, frame.can_dlc,
            frame.data[0], frame.data[1], frame.data[2], frame.data[3],
            frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
        
        if (cob_id == COB_TSDO)
        {
            // 处理SDO响应
            uint8_t command = frame.data[0];
            uint16_t index = frame.data[1] | (frame.data[2] << 8);
            uint8_t subindex = frame.data[3];
            
            if (command == 0x80)  // SDO中止
            {
                uint32_t abort_code = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
                RCLCPP_ERROR(this->get_logger(), "SDO中止: 索引=0x%04X, 子索引=0x%02X, 错误码=0x%08X", index, subindex, abort_code);
            }
            else if (index == OD_STATUS_WORD && subindex == 0x00)  // 状态字
            {
                uint16_t status_word = frame.data[4] | (frame.data[5] << 8);
                status_word_ = status_word;
                
                // 检查目标到达位
                if (status_word & 0x0400)
                {
                    RCLCPP_INFO(this->get_logger(), "目标位置已到达");
                }
            }
            else if (index == OD_ACTUAL_POSITION && subindex == 0x00)  // 实际位置
            {
                int32_t position = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
                position_ = position;
                float angle = position_to_angle(position);
                
                // 发布位置
                auto msg = std_msgs::msg::Float32();
                msg.data = angle;
                position_pub_->publish(msg);
            }
        }
        else if (cob_id == COB_TPDO1)
        {
            // 处理TPDO1响应
            if (frame.can_dlc >= 6)  // 状态字(2字节) + 实际位置(4字节)
            {
                uint16_t status_word = frame.data[0] | (frame.data[1] << 8);
                int32_t position = frame.data[2] | (frame.data[3] << 8) | (frame.data[4] << 16) | (frame.data[5] << 24);
                
                status_word_ = status_word;
                position_ = position;
                
                float angle = position_to_angle(position);
                
                // 发布位置
                auto pos_msg = std_msgs::msg::Float32();
                pos_msg.data = angle;
                position_pub_->publish(pos_msg);
                
                // 检查目标到达位
                if (status_word & 0x0400)
                {
                    RCLCPP_INFO(this->get_logger(), "目标位置已到达");
                }
            }
        }
    }
    
    void publish_status()
    {
        // 发布状态信息
        auto status_msg = std_msgs::msg::String();
        
        // 根据状态字解析状态
        std::string status_str = "未知";
        if (status_word_ & 0x0008)  // 故障
        {
            status_str = "故障";
        }
        else if ((status_word_ & 0x006F) == 0x0027)  // 操作已启用
        {
            status_str = "操作已启用";
        }
        else if ((status_word_ & 0x006F) == 0x0023)  // 已开启
        {
            status_str = "已开启";
        }
        else if ((status_word_ & 0x006F) == 0x0021)  // 准备开启
        {
            status_str = "准备开启";
        }
        else if ((status_word_ & 0x004F) == 0x0040)  // 禁止开启
        {
            status_str = "禁止开启";
        }
        
        status_msg.data = status_str;
        status_pub_->publish(status_msg);
        
        // 发布位置
        auto pos_msg = std_msgs::msg::Float32();
        pos_msg.data = position_to_angle(position_);
        position_pub_->publish(pos_msg);
    }
    
    // 回调函数：处理目标位置
    void position_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        int node_id = 0;
        for (auto i : msg->position){
            // float angle = msg->position[node_id];
            RCLCPP_INFO(this->get_logger(), "收到目标位置: %.2f°", i);
        
            // 添加更多调试信息
            RCLCPP_INFO(this->get_logger(), "当前CAN套接字: %d", can_socket_);
            RCLCPP_INFO(this->get_logger(), "当前节点ID: %d", node_id);
            
            // 读取当前状态字
            int32_t status_word = read_sdo(node_id, OD_STATUS_WORD, 0x00);
            RCLCPP_INFO(this->get_logger(), "当前状态字: 0x%04X", status_word);
            
            // 读取当前操作模式
            int32_t mode = read_sdo(node_id, OD_OPERATION_MODE_DISPLAY, 0x00);
            RCLCPP_INFO(this->get_logger(), "当前操作模式: %d", mode);
            // go_to_position(node_id, angle);
            go_to_position(node_id, i);
            node_id++;
        }
    }
    
    // 回调函数：处理目标速度
    void velocity_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {   
        int node_id = 0;
        for(auto i : msg->velocity){
            // float velocity = msg->velocity[node_id];
            RCLCPP_INFO(this->get_logger(), "收到目标速度: %.2f°/s", i);
            
            // 尝试使用PDO设置速度
            set_velocity_pdo(node_id,i);
            node_id++;
        }
        
    }
    
    // 服务回调函数：启动
    void handle_start(const std::shared_ptr<erob_master::srv::ConfigureMotor::Request> request,
                     std::shared_ptr<erob_master::srv::ConfigureMotor::Response> response)
    {
        
        RCLCPP_INFO(this->get_logger(), "收到启动请求");
        int node_id = request->node_id;
        try
        {
            initialize_motor(node_id);
            response->success = true;
            response->message = "EROB电机已启动";
        }
        catch (const std::exception& e)
        {
            response->success = false;
            response->message = "启动失败: " + std::string(e.what());
        }
    }
    
    // 服务回调函数：停止
    void handle_stop(const std::shared_ptr<erob_master::srv::ConfigureMotor::Request> request,
                    std::shared_ptr<erob_master::srv::ConfigureMotor::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "收到停止请求");
        int node_id = request->node_id;
        try
        {
            stop_motor(node_id);
            response->success = true;
            response->message = "EROB电机已停止";
        }
        catch (const std::exception& e)
        {
            response->success = false;
            response->message = "停止失败: " + std::string(e.what());
        }
    }
    
    // 服务回调函数：重置
    void handle_reset(const std::shared_ptr<erob_master::srv::ConfigureMotor::Request> request,
                     std::shared_ptr<erob_master::srv::ConfigureMotor::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "收到重置请求");
        int node_id = request->node_id;
        try
        {
            // 发送NMT重置命令
            send_nmt_command(node_id, NMT_RESET_NODE);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // 重新初始化电机
            initialize_motor(node_id);
            
            response->success = true;
            response->message = "EROB电机已重置";
        }
        catch (const std::exception& e)
        {
            response->success = false;
            response->message = "重置失败: " + std::string(e.what());
        }
    }
    
    // 服务回调函数：设置模式
    void handle_set_mode(const std::shared_ptr<erob_master::srv::ConfigureMotor::Request> request,
                        std::shared_ptr<erob_master::srv::ConfigureMotor::Response> response)
    {   
        // RCLCPP_INFO(this->get_logger(), "收到设置模式请求: %s", request->operation_mode ? "位置模式" : "速度模式");
        
        int node_id = request->node_id;
        try
        {
            // 读取当前操作模式
            int32_t mode = read_sdo(node_id, OD_OPERATION_MODE_DISPLAY, 0x00);
            RCLCPP_INFO(this->get_logger(), "当前操作模式: %d", mode);
            
            // 无论当前模式如何，都设置相应的参数
            if(request->operation_mode == "PPM"){
                // 设置位置模式参数
                set_profile_parameters(node_id, 5, 5, 5);
                
                // 设置目标位置为当前位置，防止电机立即运动
                int32_t current_position = read_sdo(node_id, OD_ACTUAL_POSITION, 0x00);
                write_sdo(node_id, OD_TARGET_POSITION, 0x00, current_position, 4);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                response->message = "已设置位置模式参数";
            }else if(request->operation_mode == "PVM"){
                // 设置速度模式参数
                set_profile_velocity(node_id, 5);  // 默认速度
                
                // 设置目标速度为0，防止电机立即运动
                write_sdo(node_id, 0x60FF, 0x00, 0, 4);  // 0x60FF是目标速度对象
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                response->message = "已设置速度模式参数";
            }
            else{
                throw std::runtime_error("未知的操作模式: " + request->operation_mode);
            }
            
            response->success = true;
        }
        catch (const std::exception& e)
        {
            response->success = false;
            response->message = "设置模式参数失败: " + std::string(e.what());
        }
    }
    
    // 辅助函数：角度转位置脉冲
    int32_t angle_to_position(float angle)
    {
        int32_t position = static_cast<int32_t>((angle / 360.0) * ENCODER_RESOLUTION);
        return position;
    }
    
    // 辅助函数：位置脉冲转角度
    float position_to_angle(int32_t position)
    {
        float angle = (static_cast<float>(position) / ENCODER_RESOLUTION) * 360.0;
        return angle;
    }
    
    // 辅助函数：速度转脉冲
    int32_t velocity_to_pulse(int32_t velocity_rpm)
    {
        int32_t velocity_pulse_per_sec = static_cast<int32_t>((velocity_rpm / 60.0) * ENCODER_RESOLUTION);
        return velocity_pulse_per_sec;
    }
    
    // 辅助函数：加速度转脉冲
    int32_t acceleration_to_pulse(int32_t acceleration_rpm2)
    {
        int32_t acceleration_pulse_per_sec2 = static_cast<int32_t>((acceleration_rpm2 / 60.0) * ENCODER_RESOLUTION);
        return acceleration_pulse_per_sec2;
    }
    
    void initialize_motor(int node_id)
    {
        // 初始化节点
        initialize_node(node_id);
        
        // 配置PDO映射
        configure_pdo(node_id);
        
        // 等待一段时间
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 启动节点
        start_node(node_id);
        
        // 设置立即生效
        set_immediate_effect(node_id, true);
        
        // 清除故障
        clear_fault(node_id);
        
        // 使能电机
        enable_motor(node_id);
        
        RCLCPP_INFO(this->get_logger(), "电机初始化完成");
    }
    
    void set_velocity(int node_id, float velocity_deg_per_sec)
    {
        RCLCPP_INFO(this->get_logger(), "设置速度: %.2f°/s", velocity_deg_per_sec);
        
        // 读取当前操作模式
        int32_t mode = read_sdo(node_id, OD_OPERATION_MODE_DISPLAY, 0x00);
        
        if (mode != MODE_PROFILE_VELOCITY && mode != MODE_VELOCITY)
        {
            RCLCPP_WARN(this->get_logger(), "当前不是速度模式，无法设置速度。当前模式: %d", mode);
            return;
        }
        
        // 转换为电机内部单位
        int32_t velocity_pulse = velocity_to_pulse(static_cast<int32_t>(velocity_deg_per_sec));
        
        // 设置目标速度
        write_sdo(node_id, 0x60FF, 0x00, velocity_pulse, 4);  // 0x60FF是目标速度对象
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 使能操作
        write_sdo(node_id, OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
        
        RCLCPP_INFO(this->get_logger(), "速度已设置: %.2f°/s (脉冲值: %d)", velocity_deg_per_sec, velocity_pulse);
    }
    
    void set_velocity_pdo(int node_id, float velocity_deg_per_sec)
    {
        RCLCPP_INFO(this->get_logger(), "使用PDO设置速度: %.2f°/s", velocity_deg_per_sec);
        
        // 读取当前操作模式
        int32_t mode = read_sdo(node_id, OD_OPERATION_MODE_DISPLAY, 0x00);
        RCLCPP_INFO(this->get_logger(), "当前操作模式: %d", mode);
        
        // 设置轮廓速度参数（无论当前模式如何）
        set_profile_velocity(node_id, static_cast<int32_t>(velocity_deg_per_sec));
        
        // 转换为电机内部单位
        int32_t velocity_pulse = velocity_to_pulse(static_cast<int32_t>(velocity_deg_per_sec));
        
        // 方法1：使用SDO设置目标速度
        write_sdo(node_id, 0x60FF, 0x00, velocity_pulse, 4);  // 0x60FF是目标速度对象
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 方法2：使用SDO设置控制字
        write_sdo(node_id, OD_CONTROL_WORD, 0x00, CONTROL_ENABLE_OPERATION, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        RCLCPP_INFO(this->get_logger(), "速度命令已发送");
        
        // 如果速度命令不起作用，尝试使用PDO
        if (mode == 0)
        {
            RCLCPP_INFO(this->get_logger(), "在模式0下尝试使用PDO发送速度命令");
            
            // 使用PDO发送速度命令
            struct can_frame frame;
            frame.can_id = COB_RPDO1 + node_id;
            frame.can_dlc = 6;  // 控制字(2字节) + 目标速度(4字节)
            frame.data[0] = CONTROL_ENABLE_OPERATION & 0xFF;  // 控制字低字节
            frame.data[1] = (CONTROL_ENABLE_OPERATION >> 8) & 0xFF;  // 控制字高字节
            frame.data[2] = velocity_pulse & 0xFF;  // 速度低字节
            frame.data[3] = (velocity_pulse >> 8) & 0xFF;
            frame.data[4] = (velocity_pulse >> 16) & 0xFF;
            frame.data[5] = (velocity_pulse >> 24) & 0xFF;  // 速度高字节
            
            if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
            {
                RCLCPP_ERROR(this->get_logger(), "发送PDO速度命令失败");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "PDO速度命令已发送");
            }
            
            send_sync_frame();
        }
    }
    
    std::string can_interface_;
    int node_id_;
    int can_socket_ = -1;
    uint16_t status_word_ = 0;
    int32_t position_ = 0;

    sensor_msgs::msg::JointState joint_state_real;
    sensor_msgs::msg::JointState joint_state_cmd;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr velocity_sub_;
    rclcpp::Service<erob_master::srv::ConfigureMotor>::SharedPtr start_service_;
    rclcpp::Service<erob_master::srv::ConfigureMotor>::SharedPtr stop_service_;
    rclcpp::Service<erob_master::srv::ConfigureMotor>::SharedPtr reset_service_;
    rclcpp::Service<erob_master::srv::ConfigureMotor>::SharedPtr set_mode_service_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CANopenROS2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
