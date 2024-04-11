
#include <amr_robot/order_optimizer.hpp>


OrderOptimizer::OrderOptimizer()
: Node("order_optimizer"), order_found_(false)
{   
    // subscribers and publishers
    order_subscription_ = this->create_subscription<custom_msg_amr::msg::Order>(
        "nextOrder", 10, std::bind(&OrderOptimizer::order_callback, this, std::placeholders::_1));

    position_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "currentPosition", 10, std::bind(&OrderOptimizer::position_callback, this, std::placeholders::_1));

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("order_path", 10);

    position_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("currentPosition", 10);

    test_publisher_ = this->create_publisher<custom_msg_amr::msg::TestMsgs>("test_msgs", 10);

    this->declare_parameter<std::string>("directory_path"); // orders directory parameter as input to the node
    this->get_parameter("directory_path", directory_path);
    RCLCPP_INFO(this->get_logger(), "Directory Folder: '%s'", directory_path.c_str());
    order_directory = directory_path + "/orders";
    order_yaml_files_ = getYamlFiles(order_directory);
    products_yaml_file = directory_path + "/configuration/products.yaml";
}

void OrderOptimizer::order_callback(const custom_msg_amr::msg::Order::SharedPtr msg) // callback function when a new order is subscribed
{
    
    order_number = msg->order_id;
    auto message = custom_msg_amr::msg::TestMsgs();
    RCLCPP_INFO(this->get_logger(), "Working on order '%d' (%s)", msg->order_id, msg->description.c_str());

    order_found_.store(false);
    for (const auto& file_path : order_yaml_files_) {
        threads_.emplace_back(std::thread(&OrderOptimizer::OrderSearch, this, file_path)); 
    }

    for (auto &thread : threads_)   // parsing the yaml files by multi-threading ( seperate thread for each yaml file )
    {
        thread.join();
    }

    if (!order_found_) {  // condition if there is a invalid order id or no order specified
        RCLCPP_INFO(this->get_logger(), " Invalid Order number");
        threads_.clear();
        message.orderfound=order_found_;
        test_publisher_->publish(message);
        return;
    }

    message.orderfound=order_found_;
    message.cx=final_cx;
    message.cy=final_cy;
    message.products=productNumbers;
    test_publisher_->publish(message);  // publishing topic for unit tests

    YAML::Node products_config = YAML::LoadFile(products_yaml_file);  // parsing through the products yaml file for to get the parts info
    for (int product_id : productNumbers) {
        for (const auto& item : products_config) {
            int product = item["id"].as<int>();
            if (product == product_id) {
                const auto& parts = item["parts"];
                for (const auto& part_node : parts) {
                    std::string part_name = part_node["part"].as<std::string>();
                    Part part;
                    part.name=part_name;
                    part.cx=part_node["cx"].as<double>();
                    part.cy=part_node["cy"].as<double>();
                    allParts.push_back(std::make_pair(product_id, part));
                }  
            }
        }
    }

    Point origin{initial_x, initial_y};
    
    std::vector<std::pair<int, Part>> pathSequence = shortestPathSequence(origin, allParts); // function call to find the shortest path to destination

    std::cout << "Shortest path sequence:" << std::endl;

    RCLCPP_INFO(this->get_logger(), "Current position of robot is: %f, y: %f", initial_x, initial_y);

    for (const auto& part : pathSequence) {
        RCLCPP_INFO(this->get_logger(), "Fetching part %s for product ‘%d’ at x: %f, y: %f", part.second.name.c_str(), part.first, part.second.cx, part.second.cy);
    }
    RCLCPP_INFO(this->get_logger(), "Delivering to destination x: %f, y: %f", final_cx, final_cy);

    marker_array_.markers.resize(pathSequence.size()+2); 

    // marker array for the robot position, the pick up points and destination

    visualization_msgs::msg::Marker& marker1 = marker_array_.markers[0];  
    marker1.header.frame_id = "map";
    marker1.header.stamp = this->now();
    marker1.ns = "order_markers";
    marker1.id = 0;
    marker1.type = visualization_msgs::msg::Marker::CUBE;
    marker1.action = visualization_msgs::msg::Marker::ADD;
    marker1.pose.position.x = initial_x/100;
    marker1.pose.position.y = initial_y/100;
    marker1.scale.x = 0.5;
    marker1.scale.y = 0.5;
    marker1.scale.z = 0.5;
    marker1.color.r = 1.0;
    marker1.color.a = 1.0;

    for (size_t i = 1; i < pathSequence.size()+1; ++i) {
        const auto& pair = pathSequence[i-1];
        auto &marker = marker_array_.markers[i];
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "order_markers";
        marker.id = pair.first;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = pair.second.cx / 100;
        marker.pose.position.y = pair.second.cy / 100;
        marker.pose.position.z = 0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
    }

    visualization_msgs::msg::Marker& marker2 = marker_array_.markers[pathSequence.size()+1];
    marker2.header.frame_id = "map";
    marker2.header.stamp = this->now();
    marker2.ns = "order_markers";
    marker2.id = 1;
    marker2.type = visualization_msgs::msg::Marker::CYLINDER;
    marker2.action = visualization_msgs::msg::Marker::ADD;
    marker2.pose.position.x = final_cx/100;
    marker2.pose.position.y = final_cy/100;
    marker2.scale.x = 0.5;
    marker2.scale.y = 0.5;
    marker2.scale.z = 0.5;
    marker2.color.g = 1.0;
    marker2.color.a = 1.0;
    
    marker_publisher_->publish(marker_array_);

    auto current_position = geometry_msgs::msg::PoseStamped();
    current_position.pose.position.x = final_cx;
    current_position.pose.position.y = final_cy;
    position_publisher_->publish(current_position);

    marker_array_.markers.clear();
    threads_.clear();
    productNumbers.clear();
    allParts.clear();
    pathSequence.clear();
}

void OrderOptimizer::position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)  // callback for the robot's current position
{
    initial_x=msg->pose.position.x;
    initial_y=msg->pose.position.y;
}

void OrderOptimizer::OrderSearch(const std::string& file_path)  // thread function for parsing orders yaml files
{
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        for (const auto& node : config) {
            
            uint32_t order = node["order"].as<uint32_t>();
            if (order == order_number) {
                {
                std::lock_guard<std::mutex> lock(mutex_);
                final_cx = node["cx"].as<double>();
                final_cy = node["cy"].as<double>();
                const auto& products = node["products"];
                for (const auto& product : products) {
                    productNumbers.push_back(product.as<int>());
                }
                order_found_.store(true);
                
                }
                return;
            }
            if (order_found_.load()) {
                return;
            }
        }
    } catch (const YAML::Exception& e) {
        std::lock_guard<std::mutex> lock(mutex_);
        RCLCPP_ERROR(get_logger(), "Error parsing YAML file %s: %s", file_path.c_str(), e.what());
    } 
}

std::vector<std::string> OrderOptimizer::getYamlFiles(const std::string& order_directory)  // function to get the list of yaml files in a directory
{
    std::vector<std::string> yaml_files;

    try {
        for (const auto& entry : fs::directory_iterator(order_directory)) {
            if (entry.is_regular_file() && entry.path().extension() == ".yaml") {
                yaml_files.push_back(entry.path().string());
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return yaml_files;
}

double OrderOptimizer::distance(const Point& p1, const Point& p2)
{
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

std::vector<std::pair<int, Part>> OrderOptimizer::shortestPathSequence(const Point& origin, const std::vector<std::pair<int, Part>>& pickupPoints)  // function to find shortest path sequence
{
    std::vector<std::pair<int, Part>> pathSequence;

    std::vector<std::pair<int, Part>> remainingPoints = pickupPoints;

    auto cmp = [this, origin](const std::pair<int, Part>& p1, const std::pair<int, Part>& p2) { 
        double dist1 = sqrt((p1.second.cx - origin.x) * (p1.second.cx - origin.x) + (p1.second.cy - origin.y) * (p1.second.cy - origin.y));
        double dist2 = sqrt((p2.second.cx - origin.x) * (p2.second.cx - origin.x) + (p2.second.cy - origin.y) * (p2.second.cy - origin.y));
    return dist1 < dist2; };

    std::sort(remainingPoints.begin(), remainingPoints.end(), cmp);

    for (const auto& point : remainingPoints) {
        pathSequence.push_back(point);
    }
    return pathSequence;
}

void OrderOptimizer::TerminateThreads()
{
    for (auto& thread : threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
}

int main(int argc, char * argv[])   // main function
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrderOptimizer>());
  rclcpp::shutdown();
  return 0;
}