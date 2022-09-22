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


//#####################################################################
//--------------------------------------------------------------------#
//          Public Methods -------------------------------------------#
//--------------------------------------------------------------------#
//#####################################################################



MavsdkNode::MavsdkNode(){
 std::cout<< "contructing mavsdk_node...\n";
}

void MavsdkNode::init(std::string port,bool fowarding){    //check this parameters later

     (void) fowarding;   //for now to prevent warnings
     std::cout<<"Trying to connect to server..."<< std::endl;
     mavsdk::Mavsdk mavsdk;
     mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection(port);
 
    if (connection_result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return;
    }

    system = this->get_system(mavsdk);
    if (!system) {
        return;
    }

    auto telemetry = mavsdk::Telemetry{system};

    // ------------------ Setting rates of telemetry 
    const mavsdk::Telemetry::Result set_rate_result = telemetry.set_rate_position(1.0);
    if (set_rate_result != mavsdk::Telemetry::Result::Success) {
        // handle rate-setting failure (in this case print error)
        std::cout << "Setting rate failed:" << set_rate_result << '\n';
    }



    // to be finished


    //---------------------subscribe to telemetry streams

    telemetry.subscribe_position([](mavsdk::Telemetry::Position position) {
    std::cout << "Altitude: " << position.relative_altitude_m << " m" << std::endl
              << "Latitude: " << position.latitude_deg << std::endl
              << "Longitude: " << position.longitude_deg << '\n';
    });

    // Set up callback to monitor flight mode 'changes'
    mavsdk::Telemetry::FlightMode oldFlightMode= mavsdk::Telemetry::FlightMode::Unknown;
    telemetry.subscribe_flight_mode([&oldFlightMode](mavsdk::Telemetry::FlightMode flightMode) {
        if (oldFlightMode != flightMode) {
        //Flight mode changed. Print!
        std::cout << "FlightMode: " << flightMode << '\n';
        oldFlightMode=flightMode; //Save updated mode.
    }
}); 
    

}



void MavsdkNode::print(){
    std::cout<< "hello!!!";
}







//#####################################################################
//--------------------------------------------------------------------#
//          Private Methods ------------------------------------------#
//--------------------------------------------------------------------#
//#####################################################################


void MavNode::arm_disarm(int mode){
    
}




void MavsdkNode::usage_info(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}




std::shared_ptr<mavsdk::System> MavsdkNode::get_system(mavsdk::Mavsdk& mavsdk)
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    // only fufill promise when heartbeat detected
    return fut.get();
}