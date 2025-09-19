#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <custom_msg/msg/robot_state.hpp>
#include <custom_msg/msg/robot_command.hpp>

#define PROGRAM_MAJOR_VERSION 1
#define PROGRAM_MINOR_VERSION 0

class walkBackFuncNode : public rclcpp::Node
{
public:
    walkBackFuncNode(const std::string &strNodeName) : Node(strNodeName)
    {
        using namespace std::literals::chrono_literals;
        using std::placeholders::_1;

        auto qos = rclcpp::QoS(1);

        try
        {
            m_pPublishTopicTimer = this->create_wall_timer(20ms, std::bind(&bodyTwistFuncNode::walkBackControlMsgFunc, this));

            m_pRobotCommandPublisher = this->create_publisher<custom_msg::msg::RobotCommand>(m_strRobotCommandTopicName, 1);

            m_pRobotControllVelCmdPublisher = this->create_publisher<geometry_msgs::msg::Twist>(m_strVelCmdTopicName, 1);

            m_pRobotStateSubscriber = this->create_subscription<custom_msg::msg::RobotState>(m_strRobotStateTopicName, 1, std::bind(&bodyTwistFuncNode::parseRobotStateMsgFunc, this, _1));
        }
        catch (const rclcpp::exceptions::RCLError &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Fatal error during initialization: %s", e.what());
            rclcpp::shutdown();
            return;
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Standard exception during IO initialization: %s", e.what());
            rclcpp::shutdown();
            return;
        }
    }

private:
    typedef enum RobotStateEnum
    {
        PASSIVE = 0,
        STAND_DOWN = 1,
        STAND_UP,
        BALANCE_STAND,
        WALK
    } T_RobotStateEnum;

    std::string m_strRobotCommandTopicName = "/robot_cmd";
    std::string m_strHandleTopicName = "/joy";
    std::string m_strRobotStateTopicName = "/robot_state";
    std::string m_strVelCmdTopicName = "/vel_cmd";
    std::string m_strNodeName = "node_name_to_be_defined"; // default
    std::string m_strNodeVersion = "V" + std::to_string(PROGRAM_MAJOR_VERSION) + std::string(".") + std::to_string(PROGRAM_MINOR_VERSION);

    rclcpp::TimerBase::SharedPtr m_pPublishTopicTimer = nullptr;
    rclcpp::Publisher<custom_msg::msg::RobotCommand>::SharedPtr m_pRobotCommandPublisher = nullptr;
    rclcpp::Subscription<custom_msg::msg::RobotState>::SharedPtr m_pRobotStateSubscriber = nullptr;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pRobotControllVelCmdPublisher = nullptr;
    custom_msg::msg::RobotState m_msgRobotState;

    void walkBackControlMsgFunc(void);
    void parseRobotStateMsgFunc(const custom_msg::msg::RobotState::SharedPtr msg);
};

void walkBackFuncNode::parseRobotStateMsgFunc(const custom_msg::msg::RobotState::SharedPtr msg)
{
    m_msgRobotState.temp[10] = msg->temp[10]; // Robot State Get Byte
    RCLCPP_INFO(this->get_logger(), "Subscribe Robot State Msg, m_msgRobotState.temp[10] = %f", m_msgRobotState.temp[10]);
}

void walkBackFuncNode::walkBackControlMsgFunc(void)
{
    if (m_msgRobotState.temp[10] < WALK) // Robot State Check Byte
    {
        custom_msg::msg::RobotCommand cmdMsg;
        cmdMsg.target_state = m_msgRobotState.temp[10] + 1;
        m_pRobotCommandPublisher->publish(cmdMsg);
        sleep(1); // sleep for robot state changing
    }
    else
    {
        auto controlMsg = geometry_msgs::msg::Twist();
        // 先往前走在往左右走再回来

        m_pRobotControllVelCmdPublisher->publish(controlMsg);
    }
}

int main(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (std::string(argv[1]) == "-v")
        {
            std::string version = std::to_string(PROGRAM_MAJOR_VERSION) +
                                  std::string(".") +
                                  std::to_string(PROGRAM_MINOR_VERSION);
            std::cout << version << std::endl;
            return 0;
        }
    }

    rclcpp::init(argc, argv);

    auto node = std::make_shared<walkBackFuncNode>("walk_back_control_publisher");
    RCLCPP_INFO(node->get_logger(), "Starting publisher node...");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}