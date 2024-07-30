#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "hbmem_pubsub/msg/sample_message1_m.hpp"
#include "hbmem_pubsub/msg/sample_message2_m.hpp"
#include "hbmem_pubsub/msg/sample_message5_m.hpp"
#include "hbmem_pubsub/msg/sample_message10_m.hpp"
#include "hbmem_pubsub/msg/sample_message20_m.hpp"

class MinimalHbmemSubscriber : public rclcpp::Node {
public:
  MinimalHbmemSubscriber() : Node("minimal_hbmem_subscriber") {
    // 创建subscription_hbmem，topic为"sample"
    // 消息回调函数为topic_callback
    subscription_1m_ = this->create_subscription<hbmem_pubsub::msg::SampleMessage1M>(
        "topic_1m", rclcpp::SensorDataQoS(),
        std::bind(&MinimalHbmemSubscriber::topic_callback<hbmem_pubsub::msg::SampleMessage1M>, this, std::placeholders::_1));
    subscription_2m_ = this->create_subscription<hbmem_pubsub::msg::SampleMessage2M>(
        "topic_2m", rclcpp::SensorDataQoS(),
        std::bind(&MinimalHbmemSubscriber::topic_callback<hbmem_pubsub::msg::SampleMessage2M>, this, std::placeholders::_1));
    subscription_5m_ = this->create_subscription<hbmem_pubsub::msg::SampleMessage5M>(
        "topic_5m", rclcpp::SensorDataQoS(),
        std::bind(&MinimalHbmemSubscriber::topic_callback<hbmem_pubsub::msg::SampleMessage5M>, this, std::placeholders::_1));
    subscription_10m_ = this->create_subscription<hbmem_pubsub::msg::SampleMessage10M>(
        "topic_10m", rclcpp::SensorDataQoS(),
        std::bind(&MinimalHbmemSubscriber::topic_callback<hbmem_pubsub::msg::SampleMessage10M>, this, std::placeholders::_1));
    subscription_20m_ = this->create_subscription<hbmem_pubsub::msg::SampleMessage20M>(
        "topic_20m", rclcpp::SensorDataQoS(),
        std::bind(&MinimalHbmemSubscriber::topic_callback<hbmem_pubsub::msg::SampleMessage20M>, this, std::placeholders::_1));
  }

private:
  // 消息回调函数
  template<typename MessageT>
  void topic_callback(const typename MessageT::SharedPtr msg) const{
    // 获取当前时间
    auto time_now = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    auto latency = time_now - msg->time_stamp;

    if ((msg->index - 1) % 5 == 0) {
      printf("\n\n");  // 每5条消息打印一个空行
    }

    RCLCPP_INFO(this->get_logger(), "Received message: %u with latency: %ld us", msg->index, latency);
  }

  // hbmem subscription
  rclcpp::Subscription<hbmem_pubsub::msg::SampleMessage1M>::SharedPtr subscription_1m_;
  rclcpp::Subscription<hbmem_pubsub::msg::SampleMessage2M>::SharedPtr subscription_2m_;
  rclcpp::Subscription<hbmem_pubsub::msg::SampleMessage5M>::SharedPtr subscription_5m_;
  rclcpp::Subscription<hbmem_pubsub::msg::SampleMessage10M>::SharedPtr subscription_10m_;
  rclcpp::Subscription<hbmem_pubsub::msg::SampleMessage20M>::SharedPtr subscription_20m_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalHbmemSubscriber>());
  rclcpp::shutdown();
  return 0;
}


