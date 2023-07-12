#include <mavsdk/mavsdk.h>
#include <MavsdkNode.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/manual_control/manual_control.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "glassy_interfaces/srv/arm.hpp"

using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

using namespace std::placeholders;

// #####################################################################
//--------------------------------------------------------------------#
//           Public Methods -------------------------------------------#
//--------------------------------------------------------------------#
// #####################################################################

MavsdkNode::MavsdkNode()
{
    std::cout << "contructing mavsdk_node...\n";
}


// FIXME IMPORTANT, -> NEED TO ADD A CONFIG FILE AND STUFF TO CONNECT TO EITHER REAL SYSTEM OR SIMULATION
void MavsdkNode::init(std::string port, bool forwarding)
{

    std::cout << "Trying to connect to server..." << std::endl;
    mavsdk = std::make_shared<mavsdk::Mavsdk>();

    mavsdk::ForwardingOption foward;
    if (forwarding)
    {
        foward = mavsdk::ForwardingOption::ForwardingOn;
        // "udp://127.0.0.1:14550"
        this->mavsdk->add_any_connection("udp://127.0.0.1:14550", mavsdk::ForwardingOption::ForwardingOn);
    }
    else
    {
        foward = mavsdk::ForwardingOption::ForwardingOff;
    }



    mavsdk::ConnectionResult connection_result = this->mavsdk->add_any_connection(port, foward);

    if (connection_result != mavsdk::ConnectionResult::Success)
    {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return;
    }

    std::cout << "Getting system..." << std::endl;

    system = this->get_system();
}

/*
 ------------------------------------------------------------
 Service Call Backs
 -------------------------------------------------------------
*/

// Arm and Disarm Service Callback
void MavsdkNode::arm_disarm(int mode)
{
    // check if action has been initialized
    if (!this->action)
        return;
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

// #####################################################################
//--------------------------------------------------------------------#
//           Private Methods ------------------------------------------#
//--------------------------------------------------------------------#
// #####################################################################

/*
 ------------------------------------------------------------
 Offboard Mode Handling
 -------------------------------------------------------------
*/

// ----------- Enter Offboard Mode
void MavsdkNode::enter_offboard(int mode)
{
    std::cout << "Entering Offboard...";

    // enter off board using set attitude
    if (mode == 1)
    {
        this->direct_control_msg.yaw_deg = 0.0;
        this->direct_control_msg.thrust_value = 0.0;
        this->offboard->set_attitude(direct_control_msg);
    }

    // enter off board using set attitude rate
    if (mode == 2)
    {
        this->attitude_rate_msg.yaw_deg_s = 0.0;
        this->attitude_rate_msg.thrust_value = 0.0;
        this->offboard->set_attitude_rate(attitude_rate_msg);
    }
    mavsdk::Offboard::Result offboard_result = this->offboard->start();
    if (offboard_result != mavsdk::Offboard::Result::Success)
    {
        std::cerr << "Offboard start failed.... try again" << offboard_result << '\n';
    }
    else
    {
        std::cout << "Offboard started\n";
    }
    return;
}

// Leave offboard mode
void MavsdkNode::stop_offboard()
{
    // Stop offboard mode
    mavsdk::Offboard::Result offboard_result = this->offboard->stop();
    if (offboard_result != mavsdk::Offboard::Result::Success)
    {
        std::cerr << "Offboard::stop() failed: " << offboard_result << '\n';
    }
    else
    {
        std::cout << "Exited Offboard Mode Successfully\n";
    }
}

// ----------- Publish to attitude setpoints and thrust
void MavsdkNode::offboard_direct_control(float yaw, float thrust)
{

    this->direct_control_msg.yaw_deg = yaw;
    this->direct_control_msg.thrust_value = thrust;
    this->offboard->set_attitude(this->direct_control_msg);
}

// ----------- Publish to attitude rate setpoints and thrust
void MavsdkNode::offboard_attitude_rate_control(float yaw_rate, float thrust)
{

    this->attitude_rate_msg.yaw_deg_s = yaw_rate;
    this->attitude_rate_msg.thrust_value = thrust;
    this->offboard->set_attitude_rate(this->attitude_rate_msg);
}

/*
 ------------------------------------------------------------
 Manual Mode Handling
 -------------------------------------------------------------
*/

// ----------- Publish to vehicle manual control actuators
void MavsdkNode::manual_mode_actuator_control(float steering_signal, float throttle_signal)
{

    const float roll = steering_signal; // can be made more efficient by passing values directly to function but for now stays like this
    const float pitch = 0.f;
    const float yaw = 0.0f;
    float throttle = throttle_signal;
    this->manual_control->set_manual_control_input(pitch, roll, throttle, yaw);
}

/*
 ------------------------------------------------------------
 State Of Vehicle Subscription Callbacks
 -------------------------------------------------------------
*/

// ------------- Publish Global Position
void MavsdkNode::publish_global_position(mavsdk::Telemetry::Position position)
{
    this->state_message.altitude = position.relative_altitude_m;
    this->state_message.latitude = position.latitude_deg;
    this->state_message.longitude = position.longitude_deg;
    this->state_message.header.stamp = this->ros_node->ros_node->get_clock()->now();
    this->print_state();

    this->ros_node->state_publisher->publish(this->state_message);
}

// ------------- Publish Local Position (NED)
void MavsdkNode::publish_ned_position(mavsdk::Telemetry::PositionVelocityNed position_velocity)
{

    this->state_message.north = position_velocity.position.north_m;
    this->state_message.east = position_velocity.position.east_m;
    this->state_message.down = position_velocity.position.down_m;
    this->state_message.header.stamp = this->ros_node->ros_node->get_clock()->now();

    this->print_state();

    this->ros_node->state_publisher->publish(this->state_message);
}

// ------------- Publish Odometry
void MavsdkNode::publish_odometry(mavsdk::Telemetry::Odometry odometry)
{

    this->state_message.surge_velocity = odometry.velocity_body.x_m_s;
    this->state_message.sway_velocity = odometry.velocity_body.y_m_s;

    this->state_message.yaw_rate = odometry.angular_velocity_body.yaw_rad_s;
    this->state_message.roll_rate = odometry.angular_velocity_body.roll_rad_s;
    this->state_message.pitch_rate = odometry.angular_velocity_body.pitch_rad_s;
    this->state_message.header.stamp = this->ros_node->ros_node->get_clock()->now();

    this->print_state();

    this->ros_node->state_publisher->publish(this->state_message);
}

// ------------- Publish Attitude
void MavsdkNode::publish_attitude(mavsdk::Telemetry::EulerAngle euler_angles)
{

    this->state_message.pitch = euler_angles.pitch_deg;
    this->state_message.roll = euler_angles.roll_deg;
    this->state_message.yaw = euler_angles.yaw_deg;
    this->state_message.header.stamp = this->ros_node->ros_node->get_clock()->now();

    this->print_state();
    this->ros_node->state_publisher->publish(this->state_message);
}

/*
 ------------------------------------------------------------
 Initialization of Mavsdk System, aswell as some helper functions
 -------------------------------------------------------------
*/

// print current state information
void MavsdkNode::print_state()
{
    std::cout << "Velocity:" << std::endl;
    std::cout << "  surge =" << this->state_message.surge_velocity << std::endl;
    std::cout << "  sway  =" << this->state_message.sway_velocity << std::endl;
    std::cout << "Position:" << std::endl;
    std::cout << "  north =" << this->state_message.north << std::endl;
    std::cout << "  east  =" << this->state_message.east << std::endl;
    std::cout << "  down  =" << this->state_message.down << std::endl;
    std::cout << "Angles:" << std::endl;
    std::cout << "  pitch =" << this->state_message.pitch << std::endl;
    std::cout << "  roll  =" << this->state_message.roll << std::endl;
    std::cout << "  yaw   =" << this->state_message.yaw << std::endl;
    std::cout << "Angular Rates:" << std::endl;
    std::cout << "  yaw_rate =" << this->state_message.yaw_rate << std::endl;
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
    std::cout << "Waiting to discover system..." << std::endl;
    auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk->subscribe_on_new_system([this, &prom]()
                                    {
        // auto system = this->mavsdk->systems()[0];
        auto system = this->mavsdk->systems().back();
        for (auto system : this->mavsdk->systems()) {
            if (system->has_autopilot()) {
                prom.set_value(system);
                std::cout << "Discovered autopilot" <<std::endl;

                // Unsubscribe again as we only want to find one system.
                this->mavsdk->subscribe_on_new_system(nullptr);
                this->initialize_system();
                break;
            }
        }
    });

    // only fufill promise when heartbeat detected
    return fut.get();
}

void MavsdkNode::initialize_system()
{
    std::thread th([this]
                   {
                       this->telemetry = std::make_shared<mavsdk::Telemetry>(system);
                       this->offboard = std::make_shared<mavsdk::Offboard>(system);
                       this->action = std::make_shared<mavsdk::Action>(system);
                       this->manual_control = std::make_shared<mavsdk::ManualControl>(system);

                       // ------------------ Setting rates of telemetry
                       const mavsdk::Telemetry::Result set_rate_result_position_global = this->telemetry->set_rate_position(20.0);
                       if (set_rate_result_position_global != mavsdk::Telemetry::Result::Success)
                       {
                           // handle rate-setting failure (in this case print error)
                           std::cout << "Setting rate failed:" << set_rate_result_position_global << std::endl;
                       }

                       const mavsdk::Telemetry::Result set_rate_result_odometry = this->telemetry->set_rate_odometry(20.0);
                       if (set_rate_result_odometry != mavsdk::Telemetry::Result::Success)
                       {
                           // handle rate-setting failure (in this case print error)
                           std::cout << "Setting rate failed:" << set_rate_result_odometry << std::endl;
                       }

                       const mavsdk::Telemetry::Result set_rate_result_attitude = this->telemetry->set_rate_attitude(20.0);
                       if (set_rate_result_attitude != mavsdk::Telemetry::Result::Success)
                       {
                           // handle rate-setting failure (in this case print error)
                           std::cout << "Setting rate failed:" << set_rate_result_attitude << std::endl;
                       }

                       // subscribe to the topics
                       this->telemetry->subscribe_position(std::bind(&MavsdkNode::publish_global_position, this, _1));
                       this->telemetry->subscribe_position_velocity_ned(std::bind(&MavsdkNode::publish_ned_position, this, _1));
                       this->telemetry->subscribe_odometry(std::bind(&MavsdkNode::publish_odometry, this, _1));
                       this->telemetry->subscribe_attitude_euler(std::bind(&MavsdkNode::publish_attitude, this, _1));
                   });

    th.detach();
}
