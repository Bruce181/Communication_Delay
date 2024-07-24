#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hbmem_pubsub/msg/sample_message.hpp"

using namespace std::chrono_literals;

class MultiTopicHbmemPublisher : public rclcpp::Node {
public:
  MultiTopicHbmemPublisher() : Node("multi_topic_hbmem_publisher"), count_(1), set_count_(0) {
    std::vector<size_t> data_sizes = {1, 2, 5, 10, 20};
    for (size_t i = 0; i < data_sizes.size(); ++i) {
      std::string topic_name = "topic_" + std::to_string(data_sizes[i]) + "M";
      size_t data_size = data_sizes[i] * 1024 * 1024;

      // 使用 KeepLast(10) 的 QoS 设置
      auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

      // 创建发布者
      auto publisher = this->create_publisher<hbmem_pubsub::msg::SampleMessage>(topic_name, qos);
      publishers_.push_back(publisher);

      // 创建定时器
      auto timer = this->create_wall_timer(
        500ms, [this, publisher, data_size]() { this->timer_callback(publisher, data_size); });
      timers_.push_back(timer);
    }
  }

private:
  void timer_callback(rclcpp::Publisher<hbmem_pubsub::msg::SampleMessage>::SharedPtr publisher, size_t data_size) {
  
    if ((count_ - 1) % 5 == 0 && (count_ - 1) != 0) {
        printf("\n\n");  // Print an empty line every 5 messages
        set_count_++;
    }  
  
    // 创建消息
    auto message = hbmem_pubsub::msg::SampleMessage();
    auto now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    message.time_stamp = now.nanoseconds();
    message.index = count_;

    // 填充数据
    message.data.fill('x');  // Assuming data is an array of fixed size

    // 发布消息
    publisher->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published message %zu of size %zu MB", count_, data_size / (1024 * 1024));
    count_++;
  }

  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  std::vector<rclcpp::Publisher<hbmem_pubsub::msg::SampleMessage>::SharedPtr> publishers_;
  size_t count_;
  size_t set_count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiTopicHbmemPublisher>());
  rclcpp::shutdown();
  return 0;
}
