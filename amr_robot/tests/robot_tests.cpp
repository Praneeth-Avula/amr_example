#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <custom_msg_amr/msg/order.hpp>
#include <custom_msg_amr/msg/test_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <thread>


bool orderfound;
std::vector<int> products;

// class node for test msgs subscriber

class TestMsgsSubscriber : public rclcpp::Node
{
public:
    TestMsgsSubscriber() : Node("test_msgs_subscriber")
    {
        test_msgs_subscription_ = this->create_subscription<custom_msg_amr::msg::TestMsgs>(
            "test_msgs", 10, std::bind(&TestMsgsSubscriber::testMsgsCallback, this, std::placeholders::_1));
    }

private:
    void testMsgsCallback(const custom_msg_amr::msg::TestMsgs::SharedPtr msg)
    {
        products.clear();
        orderfound = msg->orderfound;
        products = msg->products;
    }

    rclcpp::Subscription<custom_msg_amr::msg::TestMsgs>::SharedPtr test_msgs_subscription_;
};

// class node for order publisher

class OrderPublisher : public rclcpp::Node
{
public:
    OrderPublisher() : Node("order_publisher")
    {
        order_publisher_ = this->create_publisher<custom_msg_amr::msg::Order>("nextOrder", 10);
    }

    void publishOrder(uint32_t order_id, const std::string& description)
    {
        auto order_msg = std::make_unique<custom_msg_amr::msg::Order>();
        order_msg->order_id = order_id;
        order_msg->description = description;
        order_publisher_->publish(std::move(order_msg));
    }

private:
    rclcpp::Publisher<custom_msg_amr::msg::Order>::SharedPtr order_publisher_;
};

// class node for position publisher

class PositionPublisher : public rclcpp::Node
{
public:
    PositionPublisher() : Node("position_publisher")
    {
        position_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("currentPosition", 10);
    }

    void publishPosition(double x, double y)
    {
        auto position_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        position_msg->pose.position.x = x;
        position_msg->pose.position.y = y;
        position_publisher_->publish(std::move(position_msg));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_publisher_;
};

// Test case for an invalid Order id

TEST(OrderOptimizerTest, InvalidOrderTest) {


    rclcpp::init(0, nullptr);
    auto order_publisher = std::make_shared<OrderPublisher>();
    auto position_publisher = std::make_shared<PositionPublisher>();
    auto test_msgs_subscriber = std::make_shared<TestMsgsSubscriber>();

    order_publisher->publishOrder(1234, "Test order");

    position_publisher->publishPosition(0.0, 0.0);

    std::thread subscriber_thread([&]() {
        rclcpp::spin(test_msgs_subscriber);
    });
    std::this_thread::sleep_for(std::chrono::seconds(10));
    

    ASSERT_FALSE(orderfound);
    rclcpp::shutdown();
    if (subscriber_thread.joinable()) {
        subscriber_thread.join();
    }
}

// test case for an unspecified order id

TEST(OrderOptimizerTest, EmptyOrderTest) {


    rclcpp::init(0, nullptr);
    auto order_publisher = std::make_shared<OrderPublisher>();
    auto position_publisher = std::make_shared<PositionPublisher>();
    auto test_msgs_subscriber = std::make_shared<TestMsgsSubscriber>();

    order_publisher->publishOrder(0, "Test order");

    position_publisher->publishPosition(0.0, 0.0);


    std::thread subscriber_thread([&]() {
        rclcpp::spin(test_msgs_subscriber);
    });
    std::this_thread::sleep_for(std::chrono::seconds(10));
    

    
    ASSERT_FALSE(orderfound);
    rclcpp::shutdown();
    if (subscriber_thread.joinable()) {
        subscriber_thread.join();
    }
}

// test case for an random order id to get the required products

TEST(OrderOptimizerTest, ProductIdCheck) {

    

    rclcpp::init(0, nullptr);

    auto order_publisher = std::make_shared<OrderPublisher>();
    auto position_publisher = std::make_shared<PositionPublisher>();
    auto test_msgs_subscriber = std::make_shared<TestMsgsSubscriber>();

    order_publisher->publishOrder(1124570, "Test order");

    position_publisher->publishPosition(0.0, 0.0);

    std::thread subscriber_thread([&]() {
        rclcpp::spin(test_msgs_subscriber);
    });
    std::this_thread::sleep_for(std::chrono::seconds(10));
    std::vector<int> CorrectProducts = {419, 644, 475, 979};

    ASSERT_EQ(products, CorrectProducts);
    rclcpp::shutdown();
    if (subscriber_thread.joinable()) {
        subscriber_thread.join();
    }
}


