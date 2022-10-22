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
}



//////////////////////////////////////////////
//          Service callbacks
//////////////////////////////////////////////

// Arm and Disarm Service Callback
void MavsdkNode::arm_disarm(int mode) 
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
        }
    }
    if (mode == 0)
    {
        const mavsdk::Action::Result disarm_result = this->action->kill();

        if (disarm_result != mavsdk::Action::Result::Success)
        {
            std::cout << "Killing failed:" << disarm_result << '\n';
        }
    }
}






//#####################################################################
//--------------------------------------------------------------------#
//          Private Methods ------------------------------------------# 
//--------------------------------------------------------------------#
//#####################################################################



//////////////////////////////////////////////
//     Offboard Mode
//////////////////////////////////////////////


// ----------- Enter Offboard Mode
void MavsdkNode::enter_offboard(){
    std::cout<<"Entering Offboard...";
    this->offboard_actuator_control(0.0, 0.0); //send signal for offboard to be accepted
    mavsdk::Offboard::Result offboard_result = this->offboard->start();
    if (offboard_result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard start failed.... try again" << offboard_result << '\n';
    }
    std::cout << "Offboard started\n";
    return;
}


// ----------- Publish to vehicle actuator signals
void MavsdkNode::offboard_actuator_control(float steering_signal, float throttle_signal){

    this->actuator_msg.roll_deg = steering_signal;
    this->actuator_msg.thrust_value = throttle_signal;
    this->offboard->set_attitude(actuator_msg);
    std::cout<<"sending actuator signals";
    return;
}



//////////////////////////////////////////////
//     State Of Vehicle Subscription Callbacks
//////////////////////////////////////////////

// ------------- Publish Global Position
void MavsdkNode::publish_global_position(mavsdk::Telemetry::Position position){
 this->state_message.altitude = position.relative_altitude_m;
 this->state_message.latitude = position.latitude_deg;
 this->state_message.longitude = position.longitude_deg;

 this->print_state();

 this->ros_node->state_publisher->publish(this->state_message);      
}

// ------------- Publish Local Position (NED)
void MavsdkNode::publish_ned_position(mavsdk::Telemetry::PositionNed position){

 this->state_message.north = position.north_m;
 this->state_message.east = position.east_m;
 this->state_message.down = position.down_m;

 this->print_state();

 this->ros_node->state_publisher->publish(this->state_message);
    
}

// ------------- Publish Odometry
void MavsdkNode::publish_odometry(mavsdk::Telemetry::Odometry odometry){

 this->state_message.surge_velocity = odometry.velocity_body.x_m_s;
 this->state_message.sway_velocity = odometry.velocity_body.y_m_s;

 this->state_message.yaw_rate = odometry.angular_velocity_body.yaw_rad_s;
 this->state_message.roll_rate = odometry.angular_velocity_body.roll_rad_s;
 this->state_message.pitch_rate = odometry.angular_velocity_body.pitch_rad_s;

 this->print_state();

 this->ros_node->state_publisher->publish(this->state_message);

}


// ------------- Publish Attitude
void MavsdkNode::publish_attitude(mavsdk::Telemetry::EulerAngle euler_angles){

 this->state_message.pitch = euler_angles.pitch_deg;
 this->state_message.roll = euler_angles.roll_deg;
 this->state_message.yaw = euler_angles.yaw_deg;

 this->print_state();

 this->ros_node->state_publisher->publish(this->state_message);
}








/*
 ------------------------------------------------------------
 Initialization of Mavsdk System, aswell as some helper functions
 -------------------------------------------------------------
*/


//print current state information
void MavsdkNode::print_state(){

 std::cout<< "Velocity:" << std::endl;
 std::cout<< "  surge ="<< this->state_message.surge_velocity << std::endl;
 std::cout<< "  sway  ="<< this->state_message.sway_velocity << std::endl;
 std::cout<< "Position:" << std::endl;
 std::cout<< "  north ="<< this->state_message.north << std::endl;
 std::cout<< "  east  ="<< this->state_message.east << std::endl;
 std::cout<< "  down  ="<< this->state_message.down<< std::endl;
 std::cout<< "Angles:" << std::endl;
 std::cout<< "  pitch ="<< this->state_message.pitch  << std::endl;
 std::cout<< "  roll  ="<< this->state_message.roll  << std::endl;
 std::cout<< "  yaw   ="<< this->state_message.yaw  << std::endl;
 std::cout<< "Angular Rates:" << std::endl;
 std::cout<< "  yaw_rate ="<< this->state_message.yaw_rate  << std::endl;
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


