// #include "behaviortree_ros2/bt_topic_sub_node.hpp"
// // #include <std_msgs/msg/string.hpp>
// #include "sensor_msgs/msg/image.hpp"
// #include <iostream>
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/executors.hpp>
// #include <unordered_map>
// #include <behaviortree_cpp/bt_factory.h>
// #include <behaviortree_cpp/behavior_tree.h>
// #include <chrono>

// using namespace BT;
// using namespace std::chrono; // nanoseconds, system_clock, seconds


// class HeightDetection: public RosTopicSubNode<sensor_msgs::msg::Image>
// {
// public:
//   HeightDetection(const std::string& name,
//                 const NodeConfig& conf,
//                 const RosNodeParams& params)
//     : RosTopicSubNode<sensor_msgs::msg::Image>(name, conf, params)
//   {}

//   static PortsList providedPorts()
//   {
//     //Need an output for height detected value
//     // return {};
//     return providedBasicPorts({InputPort<std::string>("topic_name"), 
//                                   // OutputPort<float>("height_detected")
//                                   });
//   }

//   NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::Image>& msg) override
//   {
//     // if(last_msg) // empty if no new message received, since the last tick
//     // {
//     //   RCLCPP_INFO(logger(), "[%s] new message: %s", name().c_str(), last_msg->data.c_str());
//     // }

//     // mDepthSub = create_subscription<sensor_msgs::msg::Image>(
//     //                "depth", depth_qos,
//     //                std::bind(&MinimalDepthSubscriber::depthCallback, this, _1) );

//     // protected:
//     // void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
//         // Get a pointer to the depth values casting the data
//         // pointer to floating point
//       float new_val = 0;

//         for (int counter = 0; counter < 20; counter++)
//         {
//             float* depths = (float*)(&msg->data[0]);
    
//             // Image coordinates of the center pixel
//             int u = msg->width / 2;
//             int v = msg->height / 2;
    
//             // Linear index of the center pixel
//             int centerIdx = u + msg->width * v;
//             float newcenterIdx = depths[centerIdx];
    
//             // Output the measure
//             std::stringstream ss;
//             ss << this->name() << " -> Center distance : %f m " << newcenterIdx;
//             RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
//             // RCLCPP_INFO(get_logger(), "Center distance : %f m", newcenterIdx);
    
//             new_val = new_val + newcenterIdx;
//             sleep(0.5);
//         }
        
//         float avrg_val = new_val/20;
//         setOutput("height_detected", avrg_val);
//         // RCLCPP_INFO(get_logger(), "Average center distance of 20 data : %f m", avrg_val);
//     // }

//     // private:
//     //     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mDepthSub;

//     return NodeStatus::SUCCESS;
//   }
// };

//   // Simple tree, used to execute once each action.
//   static const char* xml_text = R"(
// <root BTCPP_format="4">
//   <BehaviorTree ID="zed2i_test">
//     <Sequence>
//       <HeightDetection topic_name="A"/>
//       <Delay delay_msec="3000">
//         <HeightDetection topic_name="B"/>
//       </Delay>
//     </Sequence>
//   </BehaviorTree>

//   <!-- Description of Node Models (used by Groot) -->
//   <TreeNodesModel>
//     <Condition ID="HeightDetection"
//                editable="true">
//       <input_port name="topic_name"/>
//     </Condition>
//   </TreeNodesModel>

// </root>
//  )";

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto nh = std::make_shared<rclcpp::Node>("zed2i_bt_sub_test");

//   BehaviorTreeFactory factory;

//   RosNodeParams params;
//   params.nh = nh;
//   params.default_port_value = "btcpp_string";
//   factory.registerNodeType<HeightDetection>("HeightDetection", params);

//   auto tree = factory.createTreeFromText(xml_text);

//   while(rclcpp::ok())
//   {
//     tree.tickWhileRunning();
//   }

//   rclcpp::spin(nh);
//   rclcpp::shutdown();
//   return 0;
// }



#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>

using namespace BT;
using namespace std::chrono;

class HeightDetection : public RosTopicSubNode<sensor_msgs::msg::Image>
{
public:
  HeightDetection(const std::string &name,
                  const NodeConfig &conf,
                  const RosNodeParams &params)
      : RosTopicSubNode<sensor_msgs::msg::Image>(name, conf, params)
  {
  }

  static PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<std::string>("topic_name")});
  }

  // NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::Image>& msg) override
  // {
  //   float new_val = 0;

  //   // for (int counter = 0; counter < 20; counter++)
  //   // {
  //     float *depths = (float *)(&msg->data[0]);

  //     int u = msg->width / 2;
  //     int v = msg->height / 2;

  //     int centerIdx = u + msg->width * v;
  //     float newcenterIdx = depths[centerIdx];

  //     std::stringstream ss;
  //     ss << this->name() << " -> Center distance : " << newcenterIdx << " m";
  //     RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

  //     new_val = new_val + newcenterIdx;
  //     sleep(0.5);
  //   // }

  //   // float avrg_val = new_val / 20;
  //   // setOutput("height_detected", avrg_val);
    
  //   return NodeStatus::SUCCESS;
  // }
NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::Image>& msg) override
{
  // float new_val = 0;
  // for (int counter=0; counter < 20; counter ++){
  // if (!msg)
  // {
  //   RCLCPP_ERROR(node_->get_logger(), "Received null pointer for message.");
  //   return NodeStatus::FAILURE;
  // }

  // if (msg->data.empty())
  // {
  //   RCLCPP_ERROR(node_->get_logger(), "Received empty data in the message.");
  //   return NodeStatus::FAILURE;
  // }
  
  // if (msg)
  // {
  // float* depths = (float*)(&msg->data[0]);
  // int u = msg->width / 2;
  // int v = msg->height / 2;
  // int centerIdx = u + msg->width * v;
  // // RCLCPP_INFO(node_->get_logger(), "Loop : %d ", counter);
  // RCLCPP_INFO(node_->get_logger(), "Center distance : %g m", depths[centerIdx]);

    
  // }
  

  // else
  // {
  //   RCLCPP_ERROR(node_->get_logger(), "Invalid index for depths array.");
  //   return NodeStatus::FAILURE;
  // }

  // Use new_val for further processing
  // if (counter == 20)
  // {
    
  // }
  // }

  return NodeStatus::SUCCESS;
}

};

static const char *xml_text = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="zed2i_test">
    <Sequence>
      <HeightDetection topic_name="/zed/zed_node/depth/depth_registered"/>
      <Delay delay_msec="3000">
        <HeightDetection topic_name="/zed/zed_node/depth/depth_registered"/>
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

  // rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(nh);
  // rclcpp::spin(nh);
  
  // tree.tickOnce();
    while (rclcpp::ok())
{
  rclcpp::spin_some(nh);
  tree.tickWhileRunning();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Add a small delay
}

  rclcpp::shutdown();
 
  // rclcpp::shutdown();
  return 0;
}
