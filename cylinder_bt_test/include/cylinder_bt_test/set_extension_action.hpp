#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "dmke_interface/action/set_extension.hpp"

class BtTest2 : public rclcpp::Node
{
    public:
        // using GetPosition = arm_interfaces::srv::GetPosition;
        using SetExtension = dmke_interface::action::SetExtension;
        using GoalHandleSetExtension = rclcpp_action::ServerGoalHandle<SetExtension>;

        BtTest2();

    private:
        // Parameter variables
        // double position_;
        // int ticks_;

        // Publishers, subscribers, service servers, action servers
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> param_callback_handle_;
        // rclcpp::Service<GetPosition>::SharedPtr srv_;
        rclcpp_action::Server<SetExtension>::SharedPtr action_;

        // void get_position_callback(
            // const std::shared_ptr<GetPosition::Request> request,
            // std::shared_ptr<GetPosition::Response> response);
        
        // void on_parameter_changed(const rclcpp::Parameter & p);

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const SetExtension::Goal> goal);
        
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleSetExtension> goal_handle);
        
        void handle_accepted(const std::shared_ptr<GoalHandleSetExtension> goal_handle);

        void execute(const std::shared_ptr<GoalHandleSetExtension> goal_handle);
};
