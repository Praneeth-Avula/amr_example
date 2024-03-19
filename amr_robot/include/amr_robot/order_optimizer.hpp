
#include <memory>
#include <filesystem>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <thread>
#include <queue>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"

// messages types
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "custom_msg_amr/msg/order.hpp"
#include "custom_msg_amr/msg/test_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace fs = std::filesystem;

struct Part {
    std::string name;
    double cx;
    double cy;
};

struct Point {
    double x, y;
};

class OrderOptimizer : public rclcpp::Node
{
public:
    OrderOptimizer();

    std::atomic<bool> order_found_;
private:
    void order_callback(const custom_msg_amr::msg::Order::SharedPtr msg);

    void position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void OrderSearch(const std::string& file_path);

    std::vector<std::string> getYamlFiles(const std::string& order_directory);

    double distance(const Point& p1, const Point& p2);

    std::vector<std::pair<int, Part>> shortestPathSequence(const Point& origin, const std::vector<std::pair<int, Part>>& pickupPoints);

    void TerminateThreads();

    uint32_t order_number;
    double initial_x;
    double initial_y;
    double final_cx;
    double final_cy;
    int order_sequence;

    std::vector<int> productNumbers;
    std::vector<std::pair<int, Part>> allParts;
    std::string directory_path;
    std::string order_directory;
    std::vector<std::string> order_yaml_files_;
    std::string products_yaml_file;
    std::vector<std::thread> threads_;
    std::mutex mutex_;

    rclcpp::Subscription<custom_msg_amr::msg::Order>::SharedPtr order_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<custom_msg_amr::msg::TestMsgs>::SharedPtr test_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_publisher_;
    visualization_msgs::msg::MarkerArray marker_array_;
};
