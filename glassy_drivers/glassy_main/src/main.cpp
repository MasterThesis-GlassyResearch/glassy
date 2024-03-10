#include <iostream>
#include "MavsdkNode.h"
#include <RosNode.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <chrono>
#include <thread>
#include <future>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;



int main(int argc, char **argv)
{
    // initialize rclcpp
    rclcpp::init(argc, argv);

    //create ros node to be used 
    std::cout << "starting main....\n";
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("glassy_main_node");

    // initialize an object of both MavlinkNode and RosNode 
    std::shared_ptr<RosNode> ros_com = std::make_shared<RosNode>(nh);
    std::shared_ptr<MavsdkNode> mav_com = std::make_shared<MavsdkNode>();


    // point teach object to each other to be able to call each others methods
    mav_com->ros_node = ros_com;
    ros_com->mav_node = mav_com;

    // initialize publishers in ros node and suscribers in mav node
    ros_com->init();


    bool forward = true;    
    std::string port = "udp://:14540";

    // Initialize parameters in main
    ros_com->ros_node->declare_parameter("port", "udp://:14540");
    ros_com->ros_node->declare_parameter("forwarding", false);
    ros_com->ros_node->declare_parameter("data_rates.position_ned", 20.0);
    ros_com->ros_node->declare_parameter("data_rates.position_gps", 1.0);
    ros_com->ros_node->declare_parameter("data_rates.attitude", 20.0);
    ros_com->ros_node->declare_parameter("data_rates.odometry", 20.0);


    // Get the values of the necessary parameters
    port = ros_com->ros_node->get_parameter("port").as_string();
    forward = ros_com->ros_node->get_parameter("forwarding").as_bool();



    // initialize proper rates...
    mav_com->set_att_rate(ros_com->ros_node->get_parameter("data_rates.attitude").as_double());
    mav_com->set_gps_rate(ros_com->ros_node->get_parameter("data_rates.position_gps").as_double());
    mav_com->set_ned_rate (ros_com->ros_node->get_parameter("data_rates.position_ned").as_double());
    mav_com->set_odom_rate(ros_com->ros_node->get_parameter("data_rates.odometry").as_double());




    //rx_connection = 'serial:///dev/ttyUSB0:460800'
    //rx_connection = 'serial:///dev/ttyACM0:57600'
    
    // port = "serial:///dev/ttyACM0:57600";
    mav_com->init(port, forward);


    // spin node 
    rclcpp::spin(nh);
    rclcpp::shutdown();

}
