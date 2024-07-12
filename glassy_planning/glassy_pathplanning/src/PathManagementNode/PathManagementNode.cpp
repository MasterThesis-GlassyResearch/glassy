#include "PathManagementNode.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

/*----------------------------------
    Constructors
-----------------------------------*/

/**
 * @brief Construct a new Path Management Node:: Path Management Node object
 *
 * @param node
 */
PathManagementNode::PathManagementNode(std::shared_ptr<rclcpp::Node> node) : pathmanagement_node(node)
{
    std::cout << "Creating Path Management Controller Node...\n";
    this->init();
}

/*----------------------------------
    Path Setting Logic
-----------------------------------*/

/**
 * @brief Set the Path object
 *
 * @param file_location
 */
void PathManagementNode::setPath(std::string file_location)
{

    this->requested_surge.clear();
    this->path_segments.clear();
    this->path_index = 0;

    std::ifstream myfile(file_location.c_str());
    std::cout << "my file var created " << std::endl;

    std::string line;
    std::vector<std::string> FileContents;
    try
    {
        if (myfile.is_open())
        {
            while (getline(myfile, line))
            {
                FileContents.push_back(line);
            }
            myfile.close();
            std::cout << "File opened and contents extracted..." << std::endl;
        }
        else
        {
            std::cout << "Unable to open file" << std::endl;
            this->deactivate();
            return;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    //* Now that the file is parsed, lets extract the information
    std::string test_str;
    for (int i = 0; i < (int)FileContents.size(); i++)
    {
        std::cout << "Starting to go through file contents..." << std::endl;
        std::vector<std::string> line_bits;
        std::stringstream ss(FileContents[i]);
        while (ss.good())
        {
            std::getline(ss, test_str, ';');
            line_bits.push_back(test_str);
        }
        if (line_bits[0] == "arc")
        {
            try
            {

                this->path_segments.push_back(std::make_shared<Arc>(
                    Arc(Eigen::Vector2d(std::stof(line_bits[1]) + this->x_correction, std::stof(line_bits[2]) + this->y_correction),
                        Eigen::Vector2d(std::stof(line_bits[3]) + this->x_correction,
                                        std::stof(line_bits[4]) + this->y_correction),
                        std::stof(line_bits[5]) * M_PI / 180)));
                this->requested_surge.push_back(std::stof(line_bits[6]));
            }
            catch (...)
            {
                // Conversion failed,
                RCLCPP_ERROR(this->pathmanagement_node->get_logger(), "Error in setting path");
            }
        }
        else if (line_bits[0] == "line")
        {
            try
            {

                this->path_segments.push_back(std::make_shared<Line>(
                    Line(Eigen::Vector2d(std::stof(line_bits[1]) + this->x_correction, std::stof(line_bits[2]) + this->y_correction),
                         Eigen::Vector2d(std::stof(line_bits[3]) + this->x_correction,
                                         std::stof(line_bits[4]) + this->y_correction))));
                this->requested_surge.push_back(std::stof(line_bits[5]));
            }
            catch (...)
            {
                // Conversion failed,
                RCLCPP_INFO(this->pathmanagement_node->get_logger(), "Error in setting path");
            }
        }
        else if (line_bits[0] == "loop")
        {
            this->loop = true;
        }
    }

    RCLCPP_INFO(this->pathmanagement_node->get_logger(), "FINISHED CREATING PATH ... \n PATH SEGMENTS: %ld", this->path_segments.size());

    this->path_index = 0;
    this->path_segments[this->path_index]->activate();
    this->path_is_set = true;
};

/*----------------------------------
    Path Info publishing
-----------------------------------*/

/**
 * @brief Publish the path information
 *
 */
void PathManagementNode::ref_publish()
{
    if (!path_is_set || !this->is_active)
    {
        return;
    }

    if (!this->path_segments[this->path_index]->is_active)
    {
        if (this->path_index < (int)(this->path_segments.size() - 1))
        {
            this->path_index += 1;
            this->path_segments[this->path_index]->activate();
        }
        else if (this->loop)
        {
            this->path_index = 0;
            this->path_segments[this->path_index]->activate();
        }
        else
        {
            this->pathref_msg.x_ref = 0.0;
            this->pathref_msg.y_ref = 0.0;
            this->pathref_msg.is_active = 0;
            this->pathref_msg.path_vel = 0.0;
            this->path_publisher->publish(this->pathref_msg);

            this->deactivate();
            RCLCPP_INFO(this->pathmanagement_node->get_logger(), "----------- PATH FINISHED -----------\n");
            return;
        }
    }
    Eigen::Vector2d result = this->path_segments[this->path_index]->getClosestPoint(this->pose_ned);

    this->pathref_msg.path_segment_index = this->path_index;
    this->pathref_msg.x_ref = result(0);
    this->pathref_msg.y_ref = result(1);
    this->pathref_msg.is_active = 1;
    this->pathref_msg.curvature = this->path_segments[this->path_index]->getCurvature();
    if (int(this->requested_surge.size()) > 0){
        this->pathref_msg.path_vel = this->requested_surge[this->path_index];
    } else{
        this->pathref_msg.path_vel = 3.0;
    }

    std::cout << "path_seg = " << this->path_index << std::endl;

    this->pathref_msg.tangent_heading = this->path_segments[this->path_index]->getTangHeading(this->pose_ned);

    this->pathref_msg.header.stamp = this->pathmanagement_node->get_clock()->now();

    // publish message
    this->path_publisher->publish(this->pathref_msg);
}

/*----------------------------------
    Necessary Logic
-----------------------------------*/

/**
 * @brief Correct the home position
 *
 * @return success of the operation (bool)
 */
bool PathManagementNode::correct_home_position()
{
    if (!this->lat)
    {
        std::cout << "no state available..." << std::endl;
        return false;
    }

    // define earths radius in meters
    float R_earth = 6378.137 * 1000;

    double lat_dif = M_PI / 180 * (this->home_lat - this->lat);
    double lon_dif = M_PI / 180 * (this->home_lon - this->lon);

    std::cout << "Latitude difference: " << lat_dif << std::endl;
    std::cout << "Longitude difference: " << lon_dif << std::endl;

    this->x_correction = R_earth * lat_dif + pose_ned(0);
    this->y_correction = R_earth * lon_dif * cos(this->home_lat * M_PI / 180) + pose_ned(1);

    std::cout << "x_correction difference: " << this->x_correction << std::endl;
    std::cout << "y_correction difference: " << this->y_correction << std::endl;

    this->pathref_msg.x_home_pos_correction = this->x_correction;
    this->pathref_msg.y_home_pos_correction = this->y_correction;
    return true;
}

/*----------------------------------
    Subscription Callbacks
-----------------------------------*/

/**
 * @brief Callback for the state subscription
 *
 * @param msg
 */
void PathManagementNode::state_subscription_callback(const glassy_msgs::msg::State::SharedPtr msg)
{
    // ned position
    this->pose_ned(0) = msg->p_ned[0];
    this->pose_ned(1) = msg->p_ned[1];

    // get latitude and longitude
    this->lat = msg->lat;
    this->lon = msg->lon;
}

/**
 * @brief Callback for the mission info
 *
 * @param msg
 */
void PathManagementNode::mission_info_subscription_callback(const glassy_msgs::msg::MissionInfo::SharedPtr msg)
{
    if (this->is_active)
    {
        if (this->mission_type != msg->mission_mode)
        {
            this->deactivate();
        }
    }
    else
    {
        if (std::find(MissionTypesPathManager.begin(), MissionTypesPathManager.end(), msg->mission_mode) != MissionTypesPathManager.end())
        {
            this->activate();
        }
    }
    this->mission_type = msg->mission_mode;
}

/**
 * @brief Callback for the path info
 *
 * @param msg
 */
void PathManagementNode::path_info_subscription_callback(const glassy_msgs::msg::PathInfo::SharedPtr msg)
{

    if (this->path_is_set && msg->path_recalculated)
    {
        RCLCPP_INFO(this->pathmanagement_node->get_logger(), "Path NOT Recalculated");
        return;
    }
    this->requested_surge.clear();
    this->path_segments.clear();
    int j = 0;
    for (int i = 0; i < int(msg->path_segments.size()); i++)
    {
        if (msg->path_segments[i] == msg->ARC)
        {
            this->path_segments.push_back(std::make_shared<Arc>(
                Arc(
                    Eigen::Vector2d(msg->path_segment_info[j] + this->x_correction, msg->path_segment_info[j + 1] + this->y_correction),
                    Eigen::Vector2d(msg->path_segment_info[j + 2] + this->x_correction,
                                    msg->path_segment_info[j + 3] + this->y_correction),
                    msg->path_segment_info[j + 4])));                
            j = j + msg->ARC_INFO_SIZE;
        }
        else if (msg->path_segments[i] == msg->STRAIGHT)
        {
            this->path_segments.push_back(std::make_shared<Line>(
                Line(
                    Eigen::Vector2d(msg->path_segment_info[j] + this->x_correction, msg->path_segment_info[j + 1] + this->y_correction),
                    Eigen::Vector2d(msg->path_segment_info[j + 2] + this->x_correction,
                                    msg->path_segment_info[j + 3] + this->y_correction))));
            j = j + msg->STRAIGHT_INFO_SIZE;
        }
    }
    this->path_index = 0;
    this->path_segments[this->path_index]->activate();
    this->path_is_set = true;
}

/*----------------------------------
    Service Callbacks
-----------------------------------*/

/**
 * @brief set the path given a file name
 *
 * @param request
 * @param response
 */
void PathManagementNode::set_path_srv_callback(const std::shared_ptr<glassy_msgs::srv::SetPath::Request> request, std::shared_ptr<glassy_msgs::srv::SetPath::Response> response)
{
    (void)response;
    this->setPath(this->path_file_directory + request->path_file);
}

/*-------------------------------------------
    Activate and deactivate the path planning
--------------------------------------------*/

/**
 * @brief Activate the path planning
 */
void PathManagementNode::activate()
{
    this->is_active = true;
    this->timer->reset();
    RCLCPP_INFO(this->pathmanagement_node->get_logger(), "Path Manager Activated");
    if (!this->is_simulation)
    {
        this->correct_home_position();
    }
}

/**
 * @brief Deactivate the path planning
 */
void PathManagementNode::deactivate()
{
    this->is_active = false;
    this->loop = false;
    this->timer->cancel();
    RCLCPP_INFO(this->pathmanagement_node->get_logger(), "Path Manager Deactivated");
}

/*----------------------------------
    Initialization Logic
-----------------------------------*/

/**
 * @brief Initialize the path management node
 */
void PathManagementNode::init()
{
    std::cout << " starting this path manager node" << std::endl;

    // get parameters (rates/ max surge/ max yawrate)
    // get parameters file locations

    /* -----------------------------
        ROS2 Service Initialization
    -------------------------------*/

    // subscribe to state topic
    this->state_subscription = this->pathmanagement_node->create_subscription<glassy_msgs::msg::State>("/glassy/state", 1, std::bind(&PathManagementNode::state_subscription_callback, this, _1));

    // subscribe to the mission info
    this->mission_info_subscription = this->pathmanagement_node->create_subscription<glassy_msgs::msg::MissionInfo>("/glassy/mission_status", 1, std::bind(&PathManagementNode::mission_info_subscription_callback, this, _1));

    // subscribe to the path info
    this->path_info_subscription = this->pathmanagement_node->create_subscription<glassy_msgs::msg::PathInfo>("/glassy/path_info", 1, std::bind(&PathManagementNode::path_info_subscription_callback, this, _1));

    // initialize publisher
    this->path_publisher = this->pathmanagement_node->create_publisher<glassy_msgs::msg::PathReferences>("/glassy/path_refs", 1);

    // initialize timer, -> dictates when to publish
    this->timer = this->pathmanagement_node->create_wall_timer(100ms, std::bind(&PathManagementNode::ref_publish, this));

    // service setup
    this->set_path_srv = this->pathmanagement_node->create_service<glassy_msgs::srv::SetPath>("/glassy/set_path", std::bind(&PathManagementNode::set_path_srv_callback, this, _1, _2));
}
