#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "hbmem_pubsub/msg/sample_message.hpp"

class MultiTopicHbmemSubscriber : public rclcpp::Node {
public:
  MultiTopicHbmemSubscriber() : Node("multi_topic_hbmem_subscriber") {
    std::vector<size_t> data_sizes = {1, 2, 5, 10, 20};
    for (size_t i = 0; i < data_sizes.size(); ++i) {
      std::string topic_name = "topic_" + std::to_string(data_sizes[i]) + "M";

      // 使用 KeepLast(10) 的 QoS 设置
      auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

      // 创建订阅者
      auto subscription = this->create_subscription<hbmem_pubsub::msg::SampleMessage>(
        topic_name, qos, [this, topic_name](const hbmem_pubsub::msg::SampleMessage::SharedPtr msg) {
          this->topic_callback(msg, topic_name);
        });
      subscriptions_.push_back(subscription);
    }
  }

private:
  void topic_callback(const hbmem_pubsub::msg::SampleMessage::SharedPtr msg, const std::string &topic_name) const {
    auto now = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    // 计算延时并打印出来
    int64_t delay_ns = now.nanoseconds() - msg->time_stamp;
    RCLCPP_INFO(this->get_logger(), "Message %d Delay in %s: %.3lf ms", msg->index, topic_name.c_str(), delay_ns / 1e6);

    if (msg->index % 5 == 0) {
      printf("\n\n");
    }
  }

  // 各个topic的订阅者
  std::vector<rclcpp::Subscription<hbmem_pubsub::msg::SampleMessage>::SharedPtr> subscriptions_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiTopicHbmemSubscriber>());
  rclcpp::shutdown();
  return 0;
}
