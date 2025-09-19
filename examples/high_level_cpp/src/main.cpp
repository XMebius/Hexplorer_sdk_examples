#include <chrono>
#include <memory>
#include <string>

/**
 * 3.ROS2模块头文件声明区域
 */
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>


/**
 * 4.第三方库头文件声明区域
 */
#include <custom_msg/msg/robot_command.hpp>
#include <custom_msg/msg/robot_state.hpp>


/**
 * 5.宏定义声明区域
 */
#define PROGRAM_MAJOR_VERSION 1
#define PROGRAM_MINOR_VERSION 0

class CFunctionNode : public rclcpp::Node {
private:
  typedef enum RobotStateEnum {
    PASSIVE = 0,
    STAND_DOWN = 1,
    STAND_UP,
    BALANCE_STAND,
    WALK
  } T_RobotStateEnum;

private:
  int m_iMajorVersion = PROGRAM_MAJOR_VERSION;
  int m_iMinorVersion = PROGRAM_MINOR_VERSION;
  int m_iHandleButtonCount[8] = {0};

  std::string m_strNodeName = "robot_control_example"; // default
  std::string m_strNodeDirectory = "./";
  std::string m_strRobotStateTopicName = "/robot_state";
  std::string m_strRobotCommandTopicName = "/robot_cmd";
  std::string m_strRobotControlCMDTopicName = "/vel_cmd";
  std::string m_strHandleTopicName = "/joy";
  std::string m_strNodeVersion = "V" + std::to_string(m_iMajorVersion) +
                                 std::string(".") +
                                 std::to_string(m_iMinorVersion);

private:
  geometry_msgs::msg::Twist m_msgRobotControl;
  custom_msg::msg::RobotState m_msgRobotState;

private:
  rclcpp::TimerBase::SharedPtr m_pPubluishTopicTimer = nullptr;
  rclcpp::Subscription<custom_msg::msg::RobotState>::SharedPtr
      m_pRobotStateSubscriber = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_pJoyTopicSubscriber =
      nullptr;
  rclcpp::Publisher<custom_msg::msg::RobotCommand>::SharedPtr
      m_pRobotCommandPublisher = nullptr;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      m_pRobotControllCMDPublisher = nullptr;

public:
  /**
   * @fn CFunctionNode()
   * @brief: 构造函数声明
   * @Author: zhangchunyang
   * @Date: 20250423
   * @param [in]: null;
   * @param [out]: null;
   * @return: null;
   */
  CFunctionNode(std::string strNodeName) : Node(strNodeName) {
    using namespace std::literals::chrono_literals;
    using std::placeholders::_1;

    m_strNodeName = strNodeName;
    m_pPubluishTopicTimer = this->create_wall_timer(
        100ms, std::bind(&CFunctionNode::publishRobotControlMsgFunc, this));
    if (m_pPubluishTopicTimer == nullptr) {
      RCLCPP_FATAL(this->get_logger(),
                   "Fatal Error For Init Publish Topic Timer!");
      return;
    }

    m_pRobotControllCMDPublisher =
        this->create_publisher<geometry_msgs::msg::Twist>(
            m_strRobotControlCMDTopicName, 1);
    if (m_pRobotControllCMDPublisher == nullptr) {
      RCLCPP_FATAL(this->get_logger(),
                   "Fatal Error For Init Robot Control CMD  Topic Publisher!");
      return;
    }

    m_pRobotCommandPublisher =
        this->create_publisher<custom_msg::msg::RobotCommand>(
            m_strRobotCommandTopicName, 1);
    if (m_pRobotCommandPublisher == nullptr) {
      RCLCPP_FATAL(this->get_logger(),
                   "Fatal Error For Init Robot Command Topic Publisher!");
      return;
    }

    m_pRobotStateSubscriber =
        this->create_subscription<custom_msg::msg::RobotState>(
            m_strRobotStateTopicName, 1,
            std::bind(&CFunctionNode::parseRobotStateMsgFunc, this, _1));
    if (m_pRobotStateSubscriber == nullptr) {
      RCLCPP_FATAL(this->get_logger(),
                   "Fatal Error For Init Robot State Topic Subscriber!");
      return;
    }

    m_pJoyTopicSubscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        m_strHandleTopicName, rclcpp::SensorDataQoS(),
        std::bind(&CFunctionNode::parseJoyNodeMsgFunc, this,
                  std::placeholders::_1));
    return;
  }

private:
  /**
   * @fn void publishRobotControlMsgFunc(void)
   * @brief: 机器人控制命令发布函数
   * @Author: zhangchunyang
   * @Date: 20250423
   * @param [in]: null;
   * @param [out]: null;
   * @return: null;
   */
  void publishRobotControlMsgFunc(void);

  /** 解析机器人状态函数
   * @fn void parseRobotStateMsgFunc(const
   * custom_msg::msg::RobotState::SharedPtr msg)
   * @brief: 构造函数声明
   * @Author: zhangchunyang
   * @Date: 20250423
   * @param [in]: msg // 机器人状态消息 RobotState
   * @param [out]: null;
   * @return: null;
   */
  void parseRobotStateMsgFunc(const custom_msg::msg::RobotState::SharedPtr msg);

  /**
   * @fn void parseJoyNodeMsgFunc(const sensor_msgs::msg::Joy::SharedPtr msg)
   * @brief: 机器人手柄消息解析函数
   * @Author: zhangchunyang
   * @Date: 20250423
   * @param [in]: msg // 机器人手柄信息
   * @param [out]: null;
   * @return: null;
   */
  void parseJoyNodeMsgFunc(const sensor_msgs::msg::Joy::SharedPtr msg);
};

void CFunctionNode::parseJoyNodeMsgFunc(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "msg->axes[2] = %f, msg->axes[5] = %f",
              msg->axes[2], msg->axes[5]);
  RCLCPP_INFO(this->get_logger(), "msg->buttons[1] = %d, msg->buttons[3] = %d",
              msg->buttons[1], msg->buttons[3]);

  if (msg->axes[2] == -1 && msg->axes[5] == -1) {
    m_iHandleButtonCount[0]++;
    if (m_iHandleButtonCount[0] >= 5) {
      m_iHandleButtonCount[0] = 0;
      // TODO for handle logic
      // examole: for handle joystick logic
    }
  } else if (msg->buttons[1] == 1 && msg->buttons[3] == 1) {
    m_iHandleButtonCount[1]++;
    if (m_iHandleButtonCount[1] >= 5) {
      m_iHandleButtonCount[1] = 0;
      // TODO
      // examole: for handle buttons logic
    }
  }

  return;
}

void CFunctionNode::parseRobotStateMsgFunc(
    const custom_msg::msg::RobotState::SharedPtr msg) {
  m_msgRobotState.temp[10] = msg->temp[10]; // Robot State Get Byte
  RCLCPP_INFO(this->get_logger(),
              "Subscribe Robot State Msg, m_msgRobotState.temp[10] = %f",
              m_msgRobotState.temp[10]);
}

void CFunctionNode::publishRobotControlMsgFunc(void) {
  if (m_msgRobotState.temp[10] < WALK) {
    custom_msg::msg::RobotCommand cmdMsg;
    cmdMsg.target_state = m_msgRobotState.temp[10] + 1;
    m_pRobotCommandPublisher->publish(cmdMsg);
    sleep(1); // sleep for robot state changing
  } else {
    auto controlMsg = geometry_msgs::msg::Twist();

    float fXSpeed = 0.4; //
    float fYSpeed = 0.0;
    float fYawSpeed = 0.0;

    controlMsg.linear.x = m_msgRobotControl.linear.x * 0.5 + fXSpeed * 0.5;
    controlMsg.linear.y = m_msgRobotControl.linear.y * 0.8 + fYSpeed * 0.2;
    controlMsg.angular.z =
        m_msgRobotControl.angular.z * 0.65 + fYawSpeed * 0.35;
    m_pRobotControllCMDPublisher->publish(controlMsg);
  }

  return;
}

int main(int argc, char *argv[]) {
  if (argc > 1) {
    if (std::string(argv[1]) == "-v") {
      std::string version = std::to_string(PROGRAM_MAJOR_VERSION) +
                            std::string(".") +
                            std::to_string(PROGRAM_MINOR_VERSION);
      std::cout << version << std::endl;
      return 0;
    }
  }

  rclcpp::init(argc, argv);

  auto node = std::make_shared<CFunctionNode>("robot_control_example");
  RCLCPP_INFO(node->get_logger(), "Starting publisher node...");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}