#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <canlib.h>
#include <deque> // 追加
#include <mutex> // 追加
#include "usb_can_ros_msg/msg/can_frame.hpp"

using namespace std::chrono_literals;

class CanDriverNode : public rclcpp::Node {
public:
    CanDriverNode()
    : Node("can_driver_node") {
        this->declare_parameter("send_interval_ms", 10);
        this->get_parameter("send_interval_ms", send_interval_ms_);

        subscription_ = this->create_subscription<usb_can_ros_msg::msg::CanFrame>(
            "can_rx", 1,
            std::bind(&CanDriverNode::topic_callback, this, std::placeholders::_1)
        );
        setUpChannel(0, ch0_);
        setUpChannel(1, ch1_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(send_interval_ms_), std::bind(&CanDriverNode::timer_callback, this));
    }

    ~CanDriverNode() {
        tearDownChannel(ch0_);
        tearDownChannel(ch1_);
    }

private:
    void setUpChannel(int channel, CanHandle &hnd) {
        hnd = canOpenChannel(channel, canOPEN_ACCEPT_VIRTUAL);
        if (hnd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error opening channel: %d", channel);
            exit(1);
        }

        canSetBusOutputControl(hnd, canDRIVER_NORMAL);
        long freq = canBITRATE_125K; // Frequency (bitrate)
        unsigned int tseg1 = 15;     // Time segment 1
        unsigned int tseg2 = 8;      // Time segment 2
        unsigned int sjw = 3;        // Synchronization jump width
        unsigned int noSamp = 1;     // Number of samples
        unsigned int syncMode = 0;   // Synchronization mode
        
        canSetBusParams(hnd, freq, tseg1, tseg2, sjw, noSamp, syncMode);
        canBusOn(hnd);
    }

    void tearDownChannel(CanHandle hnd) {
        canBusOff(hnd);
        canClose(hnd);
    }

    void topic_callback(const usb_can_ros_msg::msg::CanFrame::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        message_queue_.push_back(*msg);
    }

    void timer_callback() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!message_queue_.empty()) {
            auto msg = message_queue_.front();
            message_queue_.pop_front();

            auto status = canWrite(ch1_, msg.id, msg.data.data(), msg.dlc, msg.flags);
            if (status != canOK) {
                char err_msg[64];
                canGetErrorText(status, err_msg, sizeof(err_msg));
                RCLCPP_ERROR(this->get_logger(), "Error sending frame: %s", err_msg);
                // 再試行のためにメッセージをキューの前に戻す
                message_queue_.push_front(msg);
            }
        }
    }

    rclcpp::Subscription<usb_can_ros_msg::msg::CanFrame>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    CanHandle ch0_;
    CanHandle ch1_;
    int send_interval_ms_;
    std::deque<usb_can_ros_msg::msg::CanFrame> message_queue_; // 追加
    std::mutex mutex_; // 追加
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanDriverNode>());
    rclcpp::shutdown();
    return 0;
}
