// #include "rclcpp/rclcpp.hpp"
// #include "gazebo_msgs/msg/model_states.hpp"
// #include "visualization_msgs/msg/marker.hpp"

// using std::placeholders::_1;

// class MazeVisualizer : public rclcpp::Node
// {
// public:
//     MazeVisualizer() : Node("maze_visualizer")
//     {
//         // Initialize the publisher
//         marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

//         // Initialize the subscriber
//         subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
//             "model_states", 10,
//             std::bind(&MazeVisualizer::model_states_callback, this, _1)
//         );
//     }

// private:
//     void model_states_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
//     {
//         if (msg->pose.size() == 0)
//         {
//             RCLCPP_WARN(this->get_logger(), "Received empty model states message.");
//             return;
//         }

//         auto marker = visualization_msgs::msg::Marker();
//         marker.header.frame_id = "map";
//         marker.header.stamp = this->get_clock()->now();
//         marker.ns = "maze_visualizer";
//         marker.id = 0;
//         marker.type = visualization_msgs::msg::Marker::CUBE;
//         marker.action = visualization_msgs::msg::Marker::ADD;

//         // Set marker position to the position of the first model in the message
//         marker.pose.position.x = msg->pose[0].position.x;
//         marker.pose.position.y = msg->pose[0].position.y;
//         marker.pose.position.z = msg->pose[0].position.z;
//         marker.pose.orientation.x = msg->pose[0].orientation.x;
//         marker.pose.orientation.y = msg->pose[0].orientation.y;
//         marker.pose.orientation.z = msg->pose[0].orientation.z;
//         marker.pose.orientation.w = msg->pose[0].orientation.w;

//         marker.scale.x = 1.0;
//         marker.scale.y = 1.0;
//         marker.scale.z = 1.0;

//         marker.color.a = 1.0;  // Alpha
//         marker.color.r = 0.0;
//         marker.color.g = 1.0;
//         marker.color.b = 0.0;

//         // Publish the marker
//         marker_pub_->publish(marker);

//         RCLCPP_INFO(this->get_logger(), "Published marker at: %f, %f, %f", 
//                     marker.pose.position.x, 
//                     marker.pose.position.y, 
//                     marker.pose.position.z);
//     }

//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
//     rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
// };

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MazeVisualizer>());
//     rclcpp::shutdown();
//     return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class StaticMarkerPublisher : public rclcpp::Node
{
public:
    StaticMarkerPublisher() : Node("static_marker_publisher")
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "test";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = 1.0;
        marker.pose.position.y = 1.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 2.0;
        marker.scale.y = 2.0;
        marker.scale.z = 2.0;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        publisher_->publish(marker);
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticMarkerPublisher>());
    rclcpp::shutdown();
    return 0;
}

