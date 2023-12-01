#include <rclcpp/rclcpp.hpp>
#include "pick_and_place/robot_arm_node.hpp"
#include <geometry_msgs/msg/point.hpp>


namespace robot_arm_node_ns
{

    robot_arm_node::robot_arm_node(const rclcpp::NodeOptions& options) : moving_plan_()
    {
        node_ = std::make_shared<rclcpp::Node>("robot_arm", options);
        arm_group_name = "panda_arm";
        hand_group_name = "hand";
        hand_frame = "panda_hand";
        planning_group = arm_group_name;

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group);
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        
        //logger = std::make_shared<rclcpp::Logger>("robot_arm");

    }


    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr robot_arm_node::getNodeBaseInterface()
    {
        return node_->get_node_base_interface();
    }

    void robot_arm_node::move2target(geometry_msgs::msg::Pose target_pose)
    {
        move_group_->setMaxAccelerationScalingFactor(0.5);
        move_group_->setMaxVelocityScalingFactor(0.5);

        move_group_->setPoseTarget(target_pose, move_group_->getEndEffectorLink());
        
        if (move_group_->plan(moving_plan_)==moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("robot_arm"), "plan successfull!");
            move_group_->execute(moving_plan_);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("robot_arm"), "plan failed!");
            return;
        };

        return;
    }


    
} // robot_arm_node_ns