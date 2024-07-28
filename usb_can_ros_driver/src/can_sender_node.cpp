#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <canlib.h>
#include "usb_can_ros_msg/msg/can_frame.hpp"

using namespace std::chrono_literals;

class CanDriverNode : public rclcpp::Node {
public:
    CanDriverNode()
    : Node("can_driver_node") {
        subscription_ = this->create_subscription<usb_can_ros_msg::msg::CanFrame>(
            "can_rx", 10,
            std::bind(&CanDriverNode::topic_callback, this, std::placeholders::_1)
        );
        setUpChannel(0, ch0_);
        setUpChannel(1, ch1_);
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
        // Convert the incoming message to a CAN frame
        auto status = canWrite(ch1_, msg->id, msg->data.data(), msg->dlc, msg->flags);
        if (status != canOK) {
            char err_msg[64];
            canGetErrorText(status, err_msg, sizeof(err_msg));
            RCLCPP_ERROR(this->get_logger(), "Error sending frame: %s", err_msg);
        } else {
            // RCLCPP_INFO(this->get_logger(), "Frame sent: ID=%d", msg->id);
        }
    }

    rclcpp::Subscription<usb_can_ros_msg::msg::CanFrame>::SharedPtr subscription_;
    CanHandle ch0_;
    CanHandle ch1_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanDriverNode>());
    rclcpp::shutdown();
    return 0;
}
