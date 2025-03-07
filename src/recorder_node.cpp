#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <ctime>
using namespace std::chrono_literals;

struct ModeGetterData {
  std::string data_folder{""};
  std::vector<std::string> topic_list{};
};

class ModeGetter : public rclcpp::Node
{
public:
  ModeGetter() : Node("mode_getter")
  {
    this->declare_parameter<std::string>("data_folder", "rosbag");
    this->declare_parameter<std::vector<std::string>>("topic_list", {"/rosout"});
    mode_sub_ = this->create_subscription<std_msgs::msg::String>("mode", 1, std::bind(&ModeGetter::onModeReceived, this, std::placeholders::_1));
  }

  std::optional<ModeGetterData> getData(void)
  {
    if (data_.has_value()) {
      ModeGetterData mode_data = data_.value();
      data_ = std::nullopt;
      return mode_data;
    }
    return std::nullopt;
  }

private:
  void onModeReceived(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data != last_mode_) {
      RCLCPP_INFO(get_logger(), "logger mode %s->%s", last_mode_.c_str(), msg->data.c_str());

      const std::string base_data_folder = this->get_parameter("data_folder").as_string();
      const std::vector<std::string> topic_list = this->get_parameter("topic_list").as_string_array();

      const std::string data_folder = base_data_folder + "/rosbag_" + getCurrentUtcTime() + "_" + msg->data;
      RCLCPP_INFO(get_logger(), "record %s", data_folder.c_str());

      ModeGetterData data;
      data.data_folder = data_folder;
      data.topic_list = topic_list;
      data_ = data;
    }
    last_mode_ = msg->data; 
  }

  std::string getCurrentUtcTime()
  {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm = *std::gmtime(&now_time_t);
    std::ostringstream oss;
    oss << std::put_time(&utc_tm, "%Y%m%d_%H%M%S");
    return oss.str();
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_{nullptr};
  std::string last_mode_ = "";
  std::optional<ModeGetterData> data_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto mode_getter = std::make_shared<ModeGetter>();
  std::shared_ptr<rosbag2_transport::Recorder> recorder;
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(mode_getter);

  std::shared_ptr<rosbag2_cpp::Writer> writer = std::make_shared<rosbag2_cpp::Writer>();
  std::shared_ptr<KeyboardHandler> keyboard_handler = std::make_shared<KeyboardHandler>(false);
  while(rclcpp::ok()) {
    auto mode_data = mode_getter->getData();
    if (mode_data.has_value()) {
      if (recorder) {
        recorder->stop();
        executor->remove_node(recorder);
      }

      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = mode_data.value().data_folder;
      storage_options.storage_id = "mcap";
      storage_options.max_bagfile_size = 0;
      storage_options.max_bagfile_duration = 0;
      storage_options.max_cache_size = 100;
      storage_options.storage_preset_profile = "";
      storage_options.snapshot_mode = false;

      rosbag2_transport::RecordOptions record_options;
      record_options.topics = mode_data.value().topic_list;
      record_options.rmw_serialization_format = "cdr";
      record_options.topic_polling_interval = std::chrono::milliseconds(1000);

      recorder = std::make_shared<rosbag2_transport::Recorder>(writer, keyboard_handler, storage_options, record_options);
      executor->add_node(recorder);
      recorder->record();
    }
    executor->spin_some();
  }
  if (recorder) {
    recorder->stop();
    executor->remove_node(recorder);
  }
  rclcpp::shutdown();
  return 0;
}