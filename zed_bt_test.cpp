#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace BT;

class HeightDetection: public RosTopicSubNode<sensor_msgs::msg::Image>
{
public:
  HeightDetection(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
    : RosTopicSubNode<sensor_msgs::msg::Image>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    //Need an output for height detected value
    // return {};
    return providedBasicPorts({InputPort<std::string>("topic_name"), 
                                  OutputPort<float>("height_detected")});
  }

  NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::Image>& last_msg) override
  {
    // if(last_msg) // empty if no new message received, since the last tick
    // {
    //   RCLCPP_INFO(logger(), "[%s] new message: %s", name().c_str(), last_msg->data.c_str());
    // }

    // mDepthSub = create_subscription<sensor_msgs::msg::Image>(
    //                "depth", depth_qos,
    //                std::bind(&MinimalDepthSubscriber::depthCallback, this, _1) );

    // protected:
    // void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Get a pointer to the depth values casting the data
        // pointer to floating point
        float new_val = 0;

        for (int counter = 0; counter < 20; counter++)
        {
            float* depths = (float*)(&msg->data[0]);
    
            // Image coordinates of the center pixel
            int u = msg->width / 2;
            int v = msg->height / 2;
    
            // Linear index of the center pixel
            int centerIdx = u + msg->width * v;
            float newcenterIdx = depths[centerIdx];
    
            // Output the measure
            RCLCPP_INFO(get_logger(), "Center distance : %g m", newcenterIdx);
    
            new_val = new_val + newcenterIdx;
        }

        float avrg_val = new_val/20;
        setOutput("height_detected", avrg_val);
        RCLCPP_INFO(get_logger(), "Average center distance of 20 data : %g m", avrg_val);
    // }

    // private:
    //     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mDepthSub;

    return NodeStatus::SUCCESS;
  }
};

  // Simple tree, used to execute once each action.
  static const char* xml_text = R"(
  <root BTCPP_format="4">
    <BehaviorTree>
      <Sequence>
        <Delay delay_msec="3000">
            <HeightDetection topic_name="A", height_detected="{height_1}"/>
        </Delay>
        <Delay delay_msec="3000">
            <HeightDetection topic_name="B", height_detected="{height_2}"/>
        </Delay>
      </Sequence>
    </BehaviorTree>
  </root>
 )";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("zed2i_bt_sub_test");

  BehaviorTreeFactory factory;

  RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "btcpp_string";
  factory.registerNodeType<HeightDetection>("HeightDetection", params);

  auto tree = factory.createTreeFromText(xml_text);

  while(rclcpp::ok())
  {
    tree.tickWhileRunning();
  }

  rclcpp::spin(nh);
  rclcpp::shutdown();
  return 0;
}
