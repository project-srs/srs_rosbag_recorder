// ros2
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_transport/recorder.hpp>

// std
#include <memory>

// original
// #include "my_rosbag2/my_rosbag2.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    std::shared_ptr<rosbag2_cpp::Writer> writer;
    rosbag2_storage::StorageOptions soptions;
    soptions.storage_id = "sqlite3";
    soptions.uri = "/home/ubuntu/rosbag2";
    rosbag2_transport::RecordOptions roptions;
    roptions.rmw_serialization_format = "cdr";
    roptions.all = true;

    auto node = std::make_shared<rosbag2_transport::Recorder>(writer, soptions, roptions);


    // Use a MultiThreadedExecutor to spin the BagRecorder and Recorder nodes
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    while (rclcpp::ok())
    {
        rclcpp::spin(node);
    }

    rclcpp::shutdown();
    return 0;
}