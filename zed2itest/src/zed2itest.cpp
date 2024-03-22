#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::placeholders;

class zeddepthsub : public rclcpp::Node {
  public:
    zeddepthsub()
        : Node("zed2itest") {

        /* Note: it is very important to use a QOS profile for the subscriber that is compatible
         * with the QOS profile of the publisher.
         * The ZED component node uses a default QoS profile with reliability set as "RELIABLE"
         * and durability set as "VOLATILE".
         * To be able to receive the subscribed topic the subscriber must use compatible
         * parameters.
         */

        // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings

        rclcpp::QoS depth_qos(10);
        depth_qos.keep_last(10);
        depth_qos.best_effort();
        depth_qos.durability_volatile();

        // Create depth map subscriber
        mDepthSub = create_subscription<sensor_msgs::msg::Image>(
                   "depth", depth_qos,
                   std::bind(&zeddepthsub::depthCallback, this, _1) );
    }

  protected:
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Get a pointer to the depth values casting the data
        // pointer to floating point
        float* depths = (float*)(&msg->data[0]);

        // Image coordinates of the center pixel
        int u = msg->width / 2;
        int v = msg->height / 2;

        // Linear index of the center pixel
        int centerIdx = u + msg->width * v;

        // Output the measure
        RCLCPP_INFO(get_logger(), "Raw: %d m", centerIdx);
        RCLCPP_INFO(get_logger(), "Center distance : %g m", depths[centerIdx]);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mDepthSub;
};

// The main function
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto depth_node = std::make_shared<zeddepthsub>();

    rclcpp::spin(depth_node);
    rclcpp::shutdown();
    return 0;
}

//ros2 run zed2itest zed2itest --ros-args -r depth:=/zed/zed_node/depth/depth_registered

