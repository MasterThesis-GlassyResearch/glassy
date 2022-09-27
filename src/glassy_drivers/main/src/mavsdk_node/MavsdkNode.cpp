#include <MavsdkNode.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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
{ // check this parameters later

    (void)fowarding; // for now to prevent warnings
    std::cout << "Trying to connect to server..." << std::endl;
    mavsdk::Mavsdk mavsdk;
    mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection(port);

    if (connection_result != mavsdk::ConnectionResult::Success)
    {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return;
    }

    system = this->get_system(mavsdk);
    if (!system)
    {
        return;
    }

    telemetry = std::make_shared<mavsdk::Telemetry>(system);
    offboard = std::make_shared<mavsdk::Offboard>(system);
    action = std::make_shared<mavsdk::Action>(system);

    // ------------------ Setting rates of telemetry
    const mavsdk::Telemetry::Result set_rate_result = this->telemetry->set_rate_position(1.0);
    if (set_rate_result != mavsdk::Telemetry::Result::Success)
    {
        // handle rate-setting failure (in this case print error)
        std::cout << "Setting rate failed:" << set_rate_result << '\n';
    }

    //---------------------subscribe to telemetry streams
     //std::bind(&RosNode::arm_disarm, this, _1, _2)
    this->telemetry->subscribe_position(std::bind(&MavsdkNode::publish_global_position, this, _1));
    this->telemetry->subscribe_odometry(std::bind(&MavsdkNode::publish_odometry, this, _1));
    // this->telemetry->subscribe_positionned(std::bind(&MavsdkNode::publish_global_position, this, _1));

    // Set up callback to monitor flight mode 'changes'
    mavsdk::Telemetry::FlightMode oldFlightMode = mavsdk::Telemetry::FlightMode::Unknown;
    this->telemetry->subscribe_flight_mode([&oldFlightMode](mavsdk::Telemetry::FlightMode flightMode)
                                           {
        if (oldFlightMode != flightMode) {
        //Flight mode changed. Print!
        std::cout << "FlightMode: " << flightMode << '\n';
        oldFlightMode=flightMode; //Save updated mode.
    } });

    this->ros_node->init();
    //   return 0;
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


//////////////////////////////////////////////
//          Change Flight Mode  ---------------------TODO
//////////////////////////////////////////////
void MavsdkNode::change_flight_mode(std::string flight_mode) //turn into async (TODO)
{
    (void) flight_mode;
}


//////////////////////////////////////////////
//          Use Mavsdk Telemetry Object to Subscribe to Telemetry Data
//////////////////////////////////////////////
void MavsdkNode::subscribe_info(){
 return;        //------------------------------- todo
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
//  (void) position;
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
//  (void) position;
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















void MavsdkNode::usage_info(const std::string &bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<mavsdk::System> MavsdkNode::get_system(mavsdk::Mavsdk &mavsdk)
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]()
                                   {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        } });

    // only fufill promise when heartbeat detected
    return fut.get();
}