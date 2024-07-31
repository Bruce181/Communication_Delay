#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "hbmem_pubsub/msg/sample_message1_m.hpp"
#include "hbmem_pubsub/msg/sample_message2_m.hpp"
#include "hbmem_pubsub/msg/sample_message5_m.hpp"
#include "hbmem_pubsub/msg/sample_message10_m.hpp"
#include "hbmem_pubsub/msg/sample_message20_m.hpp"

using namespace std::chrono_literals;

class MinimalHbmemPublisher : public rclcpp::Node {
public:
  MinimalHbmemPublisher() : Node("minimal_hbmem_publisher"), count_(1) {
    // 创建publisher_hbmem，topic为"topic"
    publisher_1m_ = this->create_publisher<hbmem_pubsub::msg::SampleMessage1M>("topic_1m", rclcpp::SensorDataQoS());
    publisher_2m_ = this->create_publisher<hbmem_pubsub::msg::SampleMessage2M>("topic_2m", rclcpp::SensorDataQoS());
    publisher_5m_ = this->create_publisher<hbmem_pubsub::msg::SampleMessage5M>("topic_5m", rclcpp::SensorDataQoS());
    publisher_10m_ = this->create_publisher<hbmem_pubsub::msg::SampleMessage10M>("topic_10m", rclcpp::SensorDataQoS());
    publisher_20m_ = this->create_publisher<hbmem_pubsub::msg::SampleMessage20M>("topic_20m", rclcpp::SensorDataQoS());

    // 定时器，每隔500毫秒调用一次timer_callback进行消息发送
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalHbmemPublisher::timer_callback, this));
  }

private:
  // 定时器回调函数
  void timer_callback() {
    // 发布不同大小的消息
    publish_message<hbmem_pubsub::msg::SampleMessage1M>(publisher_1m_, 1048576);  // 1MB
    publish_message<hbmem_pubsub::msg::SampleMessage2M>(publisher_2m_, 2097152);  // 2MB
    publish_message<hbmem_pubsub::msg::SampleMessage5M>(publisher_5m_, 5242880);  // 5MB
    publish_message<hbmem_pubsub::msg::SampleMessage10M>(publisher_10m_, 10485760);  // 10MB
    publish_message<hbmem_pubsub::msg::SampleMessage20M>(publisher_20m_, 20971520);  // 20MB
  }

  template<typename MessageT>
  void publish_message(const typename rclcpp::Publisher<MessageT>::SharedPtr& publisher, size_t message_size) {
    // 获取要发送的消息
    auto loanedMsg = publisher->borrow_loaned_message();
    // 判断消息是否可用，可能出现获取消息失败导致消息不可用的情况
    if (loanedMsg.is_valid()) {
      
      // 引用方式获取实际的消息
      auto& msg = loanedMsg.get();
      
      // 填充数据
      std::fill(msg.data.begin(), msg.data.end(), 'x');
     
      if ((count_ - 1) % 5 == 0 && (count_ - 1) != 0) {
          printf("\n\n");  // Print an empty line every 5 messages
      }
      
      // 获取当前时间，单位为us
      auto time_now = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      
      // 对消息的index和time_stamp进行赋值
      msg.index = count_;
      msg.time_stamp = time_now;      

      // 打印发送消息
      RCLCPP_INFO(this->get_logger(), "Sending message: %zu with size: %zuMB", count_, (message_size / 1048576));
      publisher->publish(std::move(loanedMsg));
      count_++;
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to get LoanMessage!");
    }
  }
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  // hbmem publisher
  rclcpp::Publisher<hbmem_pubsub::msg::SampleMessage1M>::SharedPtr publisher_1m_;
  rclcpp::Publisher<hbmem_pubsub::msg::SampleMessage2M>::SharedPtr publisher_2m_;
  rclcpp::Publisher<hbmem_pubsub::msg::SampleMessage5M>::SharedPtr publisher_5m_;
  rclcpp::Publisher<hbmem_pubsub::msg::SampleMessage10M>::SharedPtr publisher_10m_;
  rclcpp::Publisher<hbmem_pubsub::msg::SampleMessage20M>::SharedPtr publisher_20m_;
  // 计数器
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalHbmemPublisher>());
  rclcpp::shutdown();
  return 0;
}




