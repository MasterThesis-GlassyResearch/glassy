#include <MavsdkNode.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vehicle_interfaces/srv/arm.hpp"    
#include "vehicle_interfaces/msg/test.hpp"    
#include "vehicle_interfaces/msg/globalpos.hpp"    
#include "vehicle_interfaces/msg/nedpos.hpp"    
#include "vehicle_interfaces/msg/attitude.hpp"    


using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

using namespace std::placeholders;


//#####################################################################
//--------------------------------------------------------------------#
//          Public Methods -------------------------------------------#
//--------------------------------------------------------------------#
//#####################################################################

MavsdkNode::MavsdkNode()
{
    std::cout << "contructing mavsdk_node...\n";
}

void MavsdkNode::init(std::string port, bool fowarding)
{
    
    
     // check this parameters later

    (void)fowarding; // for now to prevent warnings
    std::cout << "Trying to connect to server..." << std::endl;
    mavsdk = std::make_shared<mavsdk::Mavsdk>();
    mavsdk::ConnectionResult connection_result = this->mavsdk->add_any_connection(port);

    if (connection_result != mavsdk::ConnectionResult::Success)
    {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return;
    }

    system = this->get_system();
    // this->ros_node->init();
}



//////////////////////////////////////////////
//          Arm and Disarm
//////////////////////////////////////////////
void MavsdkNode::arm_disarm(int mode) //turn into async (TODO)
{
    if (!this->action)
        return; // check if action plugin has been initialized
    std::cout << "Arming..." << '\n';

    if (mode == 1)
    {
        const mavsdk::Action::Result arm_result = this->action->arm();

        if (arm_result != mavsdk::Action::Result::Success)
        {
            std::cout << "Arming failed:" << arm_result << '\n';
            return; // Exit if arming fails
        }
    }
    if (mode == 0)
    {
        const mavsdk::Action::Result disarm_result = this->action->kill();

        if (disarm_result != mavsdk::Action::Result::Success)
        {
            std::cout << "Killing failed:" << disarm_result << '\n';
            return; // Exit if killing fails
        }
    }
}






//#####################################################################
//--------------------------------------------------------------------#
//          Private Methods ------------------------------------------#
//--------------------------------------------------------------------#
//#####################################################################


//////////////////////////////////////////////
//          Publish Global Position
//////////////////////////////////////////////

void MavsdkNode::publish_global_position(mavsdk::Telemetry::Position position){
 auto message = vehicle_interfaces::msg::Globalpos();
 message.longitude = position.longitude_deg;
 message.latitude = position.latitude_deg;
 message.altitude = position.relative_altitude_m;
//  RCLCPP_INFO(this->ros_node->ros_node->get_logger(), "Publishing: '%d'", message.num);    // CHANGE
 this->ros_node->global_position_publisher->publish(message);
 return;        
}

//////////////////////////////////////////////
//          Publish Local Position (NED)
//////////////////////////////////////////////
void MavsdkNode::publish_ned_position(mavsdk::Telemetry::PositionNed position){
 auto message = vehicle_interfaces::msg::Nedpos();
 message.north = position.north_m;
 message.east = position.east_m;
 message.down = position.down_m;
//  RCLCPP_INFO(this->ros_node->ros_node->get_logger(), "Publishing: '%d'", message.num);    // CHANGE
 this->ros_node->ned_position_publisher->publish(message);
 return;        
}

//////////////////////////////////////////////
//          Publish Odometry
//////////////////////////////////////////////
void MavsdkNode::publish_odometry(mavsdk::Telemetry::Odometry odometry){
 (void) odometry;


//  std::cout<< odometry.time_usec << std::endl;
 std::cout<< "Velocity:" << std::endl;
 std::cout<< "  x="<< odometry.velocity_body.x_m_s << std::endl;
 std::cout<< "  y="<< odometry.velocity_body.y_m_s << std::endl;
 std::cout<< "  z="<< odometry.velocity_body.z_m_s << std::endl;
 std::cout<< "Position:" << std::endl;
 std::cout<< "  x="<< odometry.position_body.x_m << std::endl;
 std::cout<< "  y="<< odometry.position_body.y_m << std::endl;
 std::cout<< "  z="<< odometry.position_body.z_m << std::endl;



//  auto message = vehicle_interfaces::msg::Nedpos();
//  message.north = position.north_m;
//  message.east = position.east_m;
//  message.down = position.down_m;
// //  RCLCPP_INFO(this->ros_node->ros_node->get_logger(), "Publishing: '%d'", message.num);    // CHANGE
//  this->ros_node->ned_position_publisher->publish(message);
//  return;        
}



void MavsdkNode::publish_attitude(mavsdk::Telemetry::EulerAngle euler_angles){
 std::cout<< "Angles:" << std::endl;
 std::cout<< "  pitch="<< euler_angles.roll_deg  << std::endl;
 std::cout<< "  roll="<< euler_angles.pitch_deg  << std::endl;
 std::cout<< "  yaw="<< euler_angles.yaw_deg  << std::endl;

 auto message = vehicle_interfaces::msg::Attitude();
 message.pitch = euler_angles.pitch_deg;
 message.roll = euler_angles.roll_deg;
 message.yaw = euler_angles.yaw_deg;
//  RCLCPP_INFO(this->ros_node->ros_node->get_logger(), "Publishing: '%f' '%f' '%f'", message.pitch, message.roll, message.yaw);    // CHANGE
 this->ros_node->attitude_publisher->publish(message);
//  std::cout<<"fuck";
 return;        

}











//////////////////////////////////////////////
//          Subscibe to telemetry streams
//////////////////////////////////////////////


void subscribe_telemetry(const std::vector<std::string> &subscriptions){
     (void) subscriptions;
    // for 
    std::cout<<"subcribing";

}











//////////////////////////////////////////////
//          Get mavlink system
//////////////////////////////////////////////

void MavsdkNode::usage_info(const std::string &bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<mavsdk::System> MavsdkNode::get_system()
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk->subscribe_on_new_system([this, &prom]()
                                   {
        auto system = this->mavsdk->systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            this->mavsdk->subscribe_on_new_system(nullptr);
            this->initialize_system();
            prom.set_value(system);
        } });

    // only fufill promise when heartbeat detected
    return fut.get();
}


void MavsdkNode::initialize_system(){
    std::thread th([this] {



    this->telemetry = std::make_shared<mavsdk::Telemetry>(system);
    this->offboard = std::make_shared<mavsdk::Offboard>(system);
    this->action = std::make_shared<mavsdk::Action>(system);



    // ------------------ Setting rates of telemetry
    const mavsdk::Telemetry::Result set_rate_result_position_global = this->telemetry->set_rate_position(20.0);
    if (set_rate_result_position_global != mavsdk::Telemetry::Result::Success)
    {
        // handle rate-setting failure (in this case print error)
        std::cout << "Setting rate failed:" << set_rate_result_position_global << '\n';
    }

    const mavsdk::Telemetry::Result set_rate_result_odometry = this->telemetry->set_rate_odometry(1.0);
    if (set_rate_result_odometry != mavsdk::Telemetry::Result::Success)
    {
        // handle rate-setting failure (in this case print error)
        std::cout << "Setting rate failed:" << set_rate_result_odometry << '\n';
    }

    const mavsdk::Telemetry::Result set_rate_result_attitude = this->telemetry->set_rate_attitude(1.0);
    if (set_rate_result_attitude != mavsdk::Telemetry::Result::Success)
    {
        // handle rate-setting failure (in this case print error)
        std::cout << "Setting rate failed:" << set_rate_result_attitude << '\n';
    }

    this->telemetry->subscribe_position(std::bind(&MavsdkNode::publish_global_position, this, _1));
    this->telemetry->subscribe_odometry(std::bind(&MavsdkNode::publish_odometry, this, _1));
    this->telemetry->subscribe_attitude_euler(std::bind(&MavsdkNode::publish_attitude, this, _1));


    });

    th.detach();
}