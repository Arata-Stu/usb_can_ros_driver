#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <canlib.h>
#include "usb_can_ros_msg/msg/can_frame.hpp"

using namespace std::chrono_literals;

class CanReceiverNode : public rclcpp::Node {
public:
    CanReceiverNode()
    : Node("can_receiver_node") {
        publisher_ = this->create_publisher<usb_can_ros_msg::msg::CanFrame>("can_tx", 10);
        setUpChannel(0, ch0_);
        receive_thread_ = std::thread(&CanReceiverNode::receiveLoop, this);
    }

    ~CanReceiverNode() {
        running_ = false;
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
        tearDownChannel(ch0_);
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

    void receiveLoop() {
        while (running_) {
            long id;
            unsigned char data[8];
            unsigned int dlc;
            unsigned int flags;
            unsigned long timestamp;

            canStatus status = canReadWait(ch0_, &id, data, &dlc, &flags, &timestamp, 1000);
            if (status == canOK) {
                auto message = usb_can_ros_msg::msg::CanFrame();
                message.id = id;
                std::copy(std::begin(data), std::end(data), message.data.begin());
                message.dlc = dlc;
                message.flags = flags;
                publisher_->publish(message);
                // RCLCPP_INFO(this->get_logger(), "Frame received: ID=%ld", id);
            } else if (status != canERR_NOMSG) {
                char err_msg[64];
                canGetErrorText(status, err_msg, sizeof(err_msg));
                RCLCPP_ERROR(this->get_logger(), "Error receiving frame: %s", err_msg);
            }
        }
    }

    rclcpp::Publisher<usb_can_ros_msg::msg::CanFrame>::SharedPtr publisher_;
    CanHandle ch0_;
    std::thread receive_thread_;
    std::atomic<bool> running_{true};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanReceiverNode>());
    rclcpp::shutdown();
    return 0;
}
