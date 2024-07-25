#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class GripperController : public rclcpp::Node
{
public:
    GripperController()
    : Node("gripper_controller_node")
    {
        gripper_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/kuka_gripper_controller/joint_trajectory", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GripperController::timer_callback, this));
        joint_names_ = {"left_gripper_finger_joint", "right_gripper_finger_joint"};
        goal_positions_ = {0.04, 0.04}; // Close positions
    }

private:
    void timer_callback()
    {
        auto message = trajectory_msgs::msg::JointTrajectory();
        message.joint_names = joint_names_;
        
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = goal_positions_;
        point.time_from_start.sec = 2;

        message.points.push_back(point);
        gripper_publisher_->publish(message);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> joint_names_;
    std::vector<double> goal_positions_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperController>());
    rclcpp::shutdown();
    return 0;
}
