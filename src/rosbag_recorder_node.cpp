#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <ctime>
using namespace std::chrono_literals;

class BagRecorder : public rclcpp::Node
{
public:
  BagRecorder()
      : Node("BagRecorder")
  {
    // Declare parameters with default values
    this->declare_parameter<std::string>("dataFolder", "rosbag");
    this->declare_parameter<int>("fileDuration", 60);
    this->declare_parameter<std::vector<std::string>>("loggedTopics", {"/rosout", "/system_messenger", "/labjack_ain"});

    // Get parameter values
    data_folder_ = this->get_parameter("dataFolder").as_string();
    logged_topics_ = this->get_parameter("loggedTopics").as_string_array();

    reset_bag_timer = this->create_wall_timer(600s, std::bind(&BagRecorder::reset_bag_recording, this));
  }

  void start_recording()
  {

    // Log parameter values for verification
    std::cout << "Data Folder: " << data_folder_ << std::endl;
    std::cout << "Logged Topics: ";
    for (const auto &topic : logged_topics_)
    {
      std::cout << topic << " ";
    }
    std::cout << std::endl;

    // Set storage options
    storage_options_.uri = getDataPath(data_folder_);
    storage_options_.storage_id = "mcap";
    storage_options_.max_bagfile_size = 0;
    storage_options_.max_bagfile_duration = 0;
    storage_options_.max_cache_size = 100;
    storage_options_.storage_preset_profile = "";
    storage_options_.snapshot_mode = false;

    std::cout << "Storage Path: " << storage_options_.uri << std::endl;

    writer_ = std::make_shared<rosbag2_cpp::Writer>();

    auto kh = std::make_shared<KeyboardHandler>(false);

    // Set record options
    rosbag2_transport::RecordOptions record_options;
    record_options.topics = logged_topics_;
    record_options.rmw_serialization_format = "cdr";
    record_options.topic_polling_interval = std::chrono::milliseconds(1000);

    record_options.topic_qos_profile_overrides{};

    // Initialize recorder with unique node name
    recorder_ = std::make_shared<rosbag2_transport::Recorder>(
        writer_, kh, storage_options_, record_options);

    // Start recording
    executor->add_node(recorder_);
    recorder_->record();
  }

  void stop_recording()
  {
    recorder_->stop();
    executor->remove_node(recorder_);
  }

  void reset_bag_recording()
  {
    stop_recording();

    start_recording();
  }

  ~BagRecorder()
  {
    stop_recording();
  }

  std::shared_ptr<rosbag2_transport::Recorder> get_recorder() const
  {
    return recorder_;
  }

  void set_executor(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executorSet)
  {
    executor = executorSet;
  }

private:
  rclcpp::TimerBase::SharedPtr reset_bag_timer;

  std::string data_folder_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  int file_duration_;
  std::vector<std::string> logged_topics_;
  rosbag2_storage::StorageOptions storage_options_;
  rclcpp::NodeOptions node_options_;
  std::shared_ptr<rosbag2_transport::Recorder> recorder_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;

  std::string getCurrentUtcTime()
  {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm = *std::gmtime(&now_time_t);
    std::ostringstream oss;
    oss << std::put_time(&utc_tm, "%Y_%m_%d_%H_%M_%S");
    return oss.str();
  }

  std::string getDataPath(const std::string &baseFolder)
  {
    std::string systemStartTime = getCurrentUtcTime();
    return baseFolder + "/Bag_" + systemStartTime;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto bag_recorder_node = std::make_shared<BagRecorder>();

  // Use a MultiThreadedExecutor to spin the BagRecorder and Recorder nodes
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  executor->add_node(bag_recorder_node);
  bag_recorder_node->set_executor(executor);
  bag_recorder_node->start_recording();
  // Example of how to reset the recorder with a new folder path
  //  std::thread([bag_recorder_node]() {
  //   std::this_thread::sleep_for(std::chrono::seconds(10));
  //   bag_recorder_node->reset_bag_recording();
  // }).detach();

  executor->spin();

  rclcpp::shutdown();
  return 0;
}