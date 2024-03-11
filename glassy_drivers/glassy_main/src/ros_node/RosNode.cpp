#include <MavsdkNode.h>
#include <RosNode.h>


using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;
using namespace std::placeholders;


//-------------------------------------------
//          Class Constructor
//-------------------------------------------
RosNode::RosNode(std::shared_ptr<rclcpp::Node> node) : ros_node(node)
{
    std::cout << "Creating RosNode ...\n";
}


//-------------------------------------------
//          Subscribers Callback
//-------------------------------------------
void RosNode::manual_actuator_control_callback(const glassy_interfaces::msg::ManualActuatorSignals::SharedPtr msg){
    this->mav_node->manual_mode_actuator_control(msg->steering, msg->throttle);
}

void RosNode::offboard_direct_control_callback(const glassy_interfaces::msg::OffboardDirectControl::SharedPtr msg){
    std::cout<<msg->rudder;
    std::cout<<msg->thrust;
    this->mav_node->offboard_direct_control(msg->rudder, msg->thrust);
}

void RosNode::offboard_attitude_rate_control_callback(const glassy_interfaces::msg::Offboardattituderate::SharedPtr msg){
        std::cout<<msg->yaw_rate;
    std::cout<<msg->thrust;
    this->mav_node->offboard_attitude_rate_control(msg->yaw_rate, msg->thrust);
}




//-------------------------------------------
//          Service Handling
//-------------------------------------------
void RosNode::arm_disarm(const std::shared_ptr<glassy_interfaces::srv::Arm::Request> request, std::shared_ptr<glassy_interfaces::srv::Arm::Response> response)
{
    (void)response;

    std::string mode;
    switch (request->mode)
    {
    case 1:
        mode = "ARMING";
        break;
    case 0:
        mode = "DISARMING";
        break;
    default:
        mode = "UNKNOWN";
        break;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming ARM request - %s",
                mode.c_str());

    this->mav_node->arm_disarm(request->mode);
}


void RosNode::offboard_start_stop(const std::shared_ptr<glassy_interfaces::srv::Arm::Request> request, std::shared_ptr<glassy_interfaces::srv::Arm::Response> response)
{
    (void)response;
    std::string mode;
    switch (request->mode)
    {
    case 1:
        this->mav_node->enter_offboard(1);
        mode = "START";
        break;
    case 0:
        this->mav_node->stop_offboard();
        mode = "STOP";
        break;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming OFFBOARD request %s",
                mode.c_str());
}


//-------------------------------------------
//          Node Initialization
//-------------------------------------------
void RosNode::init()
{

    // get and set parameters from yaml
    this->ros_node->declare_parameter("publishers.state", true);
    this->ros_node->declare_parameter("subscriptions.offboard_direct", true);
    this->ros_node->declare_parameter("subscriptions.offboard_attitude_rate", false);
    this->ros_node->declare_parameter("subscriptions.manual_control", false);
    this->state_publishing = this->ros_node->get_parameter("publishers.state").as_bool();
    this->direct_offboard_subscription = this->ros_node->get_parameter("subscriptions.offboard_direct").as_bool();
    this->attitude_rate_offboard_subscription = this->ros_node->get_parameter("subscriptions.offboard_attitude_rate").as_bool();
    this->manual_actuators_subscription = this->ros_node->get_parameter("subscriptions.manual_control").as_bool();

    // binding arming and disarming service to the node
    this->arm_disarm_service =
        this->ros_node->create_service<glassy_interfaces::srv::Arm>("arm_disarm", std::bind(&RosNode::arm_disarm, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Arm Disarm Service Ready...");

    this->offboard_start_stop_service = this->ros_node->create_service<glassy_interfaces::srv::Arm>("start_stop_offboard", std::bind(&RosNode::offboard_start_stop, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start/Stop Offboard Service Ready...");


    // setup publishers and subscribers...
    if(this->manual_actuators_subscription)
    {
        this->manual_actuator_subscriber = this->ros_node->create_subscription<glassy_interfaces::msg::ManualActuatorSignals>("manual_actuator_signals", 1, std::bind(&RosNode::manual_actuator_control_callback, this, _1));
    }
    if(this->attitude_rate_offboard_subscription)
    {
        this->offboard_attitude_rate_subscriber = this->ros_node->create_subscription<glassy_interfaces::msg::Offboardattituderate>("offboard_attitude_rate_signals", 1, std::bind(&RosNode::offboard_attitude_rate_control_callback, this, _1));
    }
    if (direct_offboard_subscription)
    {
        this->offboard_direct_subscriber = this->ros_node->create_subscription<glassy_interfaces::msg::OffboardDirectControl>("offboard_direct_signals", 1, std::bind(&RosNode::offboard_direct_control_callback, this, _1));
    }

    if (this->state_publishing)
    {
        this->state_publisher = this->ros_node->create_publisher<glassy_interfaces::msg::State>("state_vehicle", 1);
    }
}
