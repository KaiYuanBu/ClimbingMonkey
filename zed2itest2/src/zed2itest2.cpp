#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::placeholders;

#define RAD2DEG 57.295779513

class MinimalPoseOdomSubscriber : public rclcpp::Node {
public:
    MinimalPoseOdomSubscriber()
        : Node("zed_odom_pose_tutorial") {

        /* Note: it is very important to use a QOS profile for the subscriber that is compatible
         * with the QOS profile of the publisher.
         * The ZED component node uses a default QoS profile with reliability set as "RELIABLE"
         * and durability set as "VOLATILE".
         * To be able to receive the subscribed topic the subscriber must use compatible
         * parameters.
         */

        // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings

        rclcpp::QoS qos(10);
        qos.keep_last(10);
        qos.best_effort();
        qos.durability_volatile();

        // Create pose subscriber
        mPoseSub = create_subscription<geometry_msgs::msg::PoseStamped>(
                    "pose", qos,
                    std::bind(&MinimalPoseOdomSubscriber::poseCallback, this, _1) );

        // Create odom subscriber
        mOdomSub = create_subscription<nav_msgs::msg::Odometry>(
                    "odom", qos,
                    std::bind(&MinimalPoseOdomSubscriber::odomCallback, this, _1) );
    }

protected:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Camera position in map frame
        double tx = msg->pose.position.x;
        double ty = msg->pose.position.y;
        double tz = msg->pose.position.z;

        // Orientation quaternion
        tf2::Quaternion q(
                    msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z,
                    msg->pose.orientation.w);

        // 3x3 Rotation matrix from quaternion
        tf2::Matrix3x3 m(q);

        // Roll Pitch and Yaw from rotation matrix
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Output the measure
        RCLCPP_INFO(get_logger(), "Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f - Timestamp: %u.%u sec ",
                 msg->header.frame_id.c_str(),
                 tx, ty, tz,
                 roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG,
                    msg->header.stamp.sec,msg->header.stamp.nanosec);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Camera position in map frame
        double tx = msg->pose.pose.position.x;
        double ty = msg->pose.pose.position.y;
        double tz = msg->pose.pose.position.z;

        // Orientation quaternion
        tf2::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);

        // 3x3 Rotation matrix from quaternion
        tf2::Matrix3x3 m(q);

        // Roll Pitch and Yaw from rotation matrix
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Output the measure
        RCLCPP_INFO(get_logger(), "Received odometry in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f - Timestamp: %u.%u sec ",
                 msg->header.frame_id.c_str(),
                 tx, ty, tz,
                 roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG,
                    msg->header.stamp.sec,msg->header.stamp.nanosec);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mPoseSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdomSub;
};



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPoseOdomSubscriber>());
    rclcpp::shutdown();
    return 0;
}

//ros2 run zed2itest2 zed2itest2 --ros-args -r odom:=/zed/zed_node/odom -r pose:=/zed/zed_node/pose

