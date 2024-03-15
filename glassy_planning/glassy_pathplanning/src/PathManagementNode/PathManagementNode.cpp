#include "PathManagementNode.h"


using namespace std::chrono_literals;
using namespace std::placeholders;

/*----------------------------------
    Constructors
-----------------------------------*/

// class constructor 
PathManagementNode::PathManagementNode(std::shared_ptr<rclcpp::Node> node): pathmanagement_node(node)
{
    std::cout<<"Creating Path Management Controller Node...\n";
    this->init();
}


/*----------------------------------
    Path Setting Logic
-----------------------------------*/

void PathManagementNode::setPath(std::string file_location){

    this->requested_surge.clear();
    this->path_segments.clear();
    this->path_index = 0;


    this->correct_home_position();
    std::ifstream myfile(file_location.c_str());
    std::cout<<"my file var created "<<std::endl;
    
    std::string line;
    std::vector<std::string> FileContents;
    try
    {
        if (myfile.is_open())
        {
            while(getline(myfile, line)) {
                FileContents.push_back(line);
            }
            myfile.close();
            std::cout << "File opened and contents extracted..."<<std::endl; 

        }
        else{
            std::cout << "Unable to open file" <<std::endl;
            this->deactivate();
            return;
        } 

    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

  //* Now that the file is parsed, lets extract the information
    std::string test_str;
    for(int i =0; i<(int) FileContents.size(); i++){
            std::cout << "Starting to go through file contents..."<<std::endl; 
            std::vector<std::string> line_bits;
            std::stringstream ss(FileContents[i]);
            while (ss.good()) {
                std::getline( ss, test_str, ';' );
                line_bits.push_back(test_str);
            }
            if(line_bits[0]=="arc"){
                try {

                    this->path_segments.push_back(std::make_shared<Arc>(
                        Arc(Eigen::Vector2d(std::stof(line_bits[1])+this->x_correction, std::stof(line_bits[2])+this->y_correction), 
                        Eigen::Vector2d(std::stof(line_bits[3])+this->x_correction,
                        std::stof(line_bits[4])+this->y_correction), std::stof(line_bits[5])*M_PI/180)
                        ));
                    this->requested_surge.push_back(std::stof(line_bits[6]));

                } catch (...) {
                    // Conversion failed,
                    std::cout<< "path segment failed to be parsed..." << std::endl;
                }
            } else if(line_bits[0]=="line"){
                try {

                    this->path_segments.push_back(std::make_shared<Line>(
                        Line(Eigen::Vector2d(std::stof(line_bits[1])+this->x_correction, std::stof(line_bits[2])+this->y_correction),
                        Eigen::Vector2d(std::stof(line_bits[3])+this->x_correction,
                        std::stof(line_bits[4])+this->y_correction))
                        ));
                    this->requested_surge.push_back(std::stof(line_bits[5]));

                } catch (...) {
                    // Conversion failed,
                    std::cout<< "path segment failed to be parsed..." << std::endl;
                }
            }
            else if(line_bits[0]=="loop"){
                this->loop = true;
            }
    }

    std::cout<< "finished......." << std::endl;
    std::cout<< "size of path: "<<this->path_segments.size() <<" : " << this->requested_surge.size()<< std::endl;

    this->path_index = 0;
    this->path_segments[this->path_index]->activate();
    this->path_is_set=true;
};

void PathManagementNode::setPath(){
    this->path_segments.push_back(std::make_shared<Line>(Line(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0, 100))));
    this->requested_surge.push_back(2.f);
    this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(0, 100), Eigen::Vector2d(-20, 100), M_PI)));
    this->requested_surge.push_back(2.f);
    this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(-40, 100), Eigen::Vector2d(-20, 100), M_PI)));
    this->requested_surge.push_back(2.f);
    this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(0, 100), Eigen::Vector2d(-20, 100), M_PI)));
    this->requested_surge.push_back(2.f);
    this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(-40, 100), Eigen::Vector2d(-20, 100), M_PI)));
    this->requested_surge.push_back(2.f);

    this->path_segments[0]->activate();
    this->path_index = 0;
    this->path_is_set=true;
};


/*----------------------------------
    Path Info publishing
-----------------------------------*/


void PathManagementNode::ref_publish(){
    if(!path_is_set || !this->is_active){
        return;
    }

    if(!this->path_segments[this->path_index]->is_active){
        if(this->path_index<(int) (this->path_segments.size()-1)){
            this->path_index+=1;
            this->path_segments[this->path_index]->activate();
        } 
        else if(this->loop){
            this->path_index=0;
            this->path_segments[this->path_index]->activate();
        }
        else{
            this->pathref_msg.x_ref = 0.0;
            this->pathref_msg.y_ref = 0.0;   
            this->pathref_msg.is_active = 0;
            this->pathref_msg.path_vel = 0.0;
            this->path_publisher->publish(this->pathref_msg);
            
            this->deactivate();
            auto request_pf = std::make_shared<std_srvs::srv::SetBool::Request>();
            request_pf->data=false;
            this->activate_deactivate_pathfollowing_client->async_send_request(request_pf);

            std::cout<<"*****************\nPATH TERMINATED\n*****************"<<std::endl;
            return;
        }
    }
    Eigen::Vector2d result = this->path_segments[this->path_index]->getClosestPoint(this->current_pose);
    
    this->pathref_msg.path_segment_index = this->path_index;
    this->pathref_msg.x_ref = result(0);
    this->pathref_msg.y_ref = result(1);
    this->pathref_msg.is_active=1;
    this->pathref_msg.path_vel = this->requested_surge[this->path_index];
    
    std::cout<< "path_seg = " << this->path_index <<std::endl;


    this->pathref_msg.tangent_heading = this->path_segments[this->path_index]->getTangHeading(this->current_pose);

    this->pathref_msg.header.stamp = this->pathmanagement_node->get_clock()->now();

    // publish message
    this->path_publisher->publish(this->pathref_msg);
}   


/*----------------------------------
    Necessary Logic
-----------------------------------*/

bool PathManagementNode::correct_home_position(){
    if(!this->lat){
        std::cout<<"no state available..."<<std::endl;
        return false;
    }

    // define earths radius in meters
    float R_earth = 6378.137*1000;

    double lat_dif = M_PI/180*(this->home_lat-this->lat);
    double lon_dif = M_PI/180*(this->home_lon-this->lon);

    std::cout<<"Latitude difference: "<< lat_dif<<std::endl;
    std::cout<<"Longitude difference: "<< lon_dif<<std::endl;


    this->x_correction = R_earth * lat_dif+this->x;
    this->y_correction = R_earth * lon_dif*cos(this->home_lat*M_PI/180)+this->y;

    std::cout<<"x_correction difference: "<< this->x_correction<<std::endl;
    std::cout<<"y_correction difference: "<< this->y_correction<<std::endl;
    
    return true;
}


/*----------------------------------
    Subscription Callbacks
-----------------------------------*/

void PathManagementNode::state_subscription_callback(const glassy_interfaces::msg::State::SharedPtr msg){
    this->current_pose(0) = msg->north;
    this->current_pose(1) = msg->east;

    this->lat = msg->latitude;
    this->lon = msg->longitude;
    this->y = msg->east;
    this->x = msg->north;

}


/*----------------------------------
    Service Callbacks
-----------------------------------*/

void PathManagementNode::activate_deactivate_srv_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    (void) response;
    // create a request (to activate or deactuvat«)
    auto request_pf = std::make_shared<std_srvs::srv::SetBool::Request>();
    if(request->data){
        // if(!this->correct_home_position()){
        if(false){
            std::cout<< "UNABLE TO SET HOME POSITION"<<std::endl;
            request_pf->data=false;
            this->deactivate();
        } else{
            this->activate();
            request_pf->data=true;
            std::cout<< "PATH PLANNING STARTED SUCCESSFULLY"<<std::endl;
        }
    } else{
        this->deactivate();
        request_pf->data=false;
        std::cout<< "PATH PLANNING STOPPED SUCCESSFULLY"<<std::endl;
    }
    this->activate_deactivate_pathfollowing_client->async_send_request(request_pf);
}


void PathManagementNode::set_path_srv_callback(const std::shared_ptr<glassy_interfaces::srv::SetPath::Request> request, std::shared_ptr<glassy_interfaces::srv::SetPath::Response> response){
    (void) response;
    this->setPath(this->path_file_directory + request->path_file);
}



void PathManagementNode::activate(){
    this->is_active=true;
}
void PathManagementNode::deactivate(){
    this->is_active=false;
    this->loop=false;
}





/*----------------------------------
    Initialization Logic
-----------------------------------*/


// for now a simple initialization, parameters may be added in the future
void PathManagementNode::init(){
    std::cout<< " starting this path manager node"<<std::endl;

    
    /* -----------------------------
        ROS2 Service Initialization
    -------------------------------*/


    // subscribe to the joystick topic
    this->state_subscription = this->pathmanagement_node->create_subscription<glassy_interfaces::msg::State>("state_vehicle", 1, std::bind(&PathManagementNode::state_subscription_callback, this, _1));

    // initialize publisher
    this->path_publisher = this->pathmanagement_node->create_publisher<glassy_interfaces::msg::PathReferences>("path_refs", 1);

    // initialize timer, -> dictates when to publish
    this->timer = this->pathmanagement_node->create_wall_timer(100ms, std::bind(&PathManagementNode::ref_publish, this));

    //service setup
    this->activate_deactivate_pathplanning = this->pathmanagement_node->create_service<std_srvs::srv::SetBool>("activate_deactivate_path_planning", std::bind(&PathManagementNode::activate_deactivate_srv_callback, this, _1, _2));
    this->set_path_srv = this->pathmanagement_node->create_service<glassy_interfaces::srv::SetPath>("set_path", std::bind(&PathManagementNode::set_path_srv_callback, this, _1, _2));
    
    // client setup 
    this->activate_deactivate_pathfollowing_client = this->pathmanagement_node->create_client<std_srvs::srv::SetBool>("activate_deactivate_path_following");

}
