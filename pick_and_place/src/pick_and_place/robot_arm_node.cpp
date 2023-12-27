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
        //planning_group = arm_group_name;

        move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_group_name);
        move_group_hand_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, hand_group_name);
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        move_group_arm_->setMaxAccelerationScalingFactor(0.5);
        move_group_arm_->setMaxVelocityScalingFactor(0.5);

        move_group_hand_->setMaxAccelerationScalingFactor(0.02);
        move_group_hand_->setMaxVelocityScalingFactor(0.02);
        
        //logger = std::make_shared<rclcpp::Logger>("robot_arm");

    }

    // getNodeBaseInterface()
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr robot_arm_node::getNodeBaseInterface()
    {
        return node_->get_node_base_interface();
    }

    // move2target()
    void robot_arm_node::move2target(geometry_msgs::msg::Pose target_pose)
    {

        move_group_arm_->setPoseTarget(target_pose, move_group_arm_->getEndEffectorLink());
        
        if (move_group_arm_->plan(moving_plan_)==moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("robot_arm"), "plan successfull!");
            move_group_arm_->execute(moving_plan_);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("robot_arm"), "plan failed!");
            return;
        };

        return;
    }

    // grasp and release aproach and retreat

    void robot_arm_node::grasp()
    {

        // set hand target joint values
        move_group_hand_->setJointValueTarget(std::vector<double>{0.00,0.00});

        // create a plan object to store generated plan
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        if (move_group_hand_->plan(moving_plan_)==moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("robot_hand"), "plan successfull!");
            move_group_hand_->execute(moving_plan_);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("robot_hand"), "plan failed!");
            return;
        };

        // execute the plan
        move_group_hand_->execute(my_plan);

        return;
    }

    void robot_arm_node::release()
    {

        // set hand target joint values
        move_group_hand_->setJointValueTarget(std::vector<double>{0.05,0.05});

        // create a plan object to store generated plan
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        if (move_group_hand_->plan(moving_plan_)==moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("robot_hand"), "plan successfull!");
            move_group_hand_->execute(moving_plan_);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("robot_hand"), "plan failed!");
            return;
        };

        // execute the plan
        move_group_hand_->execute(my_plan);

        return;
    }

    void robot_arm_node::set_constraints()
    {
        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = "panda_link5";
        ocm.header.frame_id = "panda_link0";
        ocm.orientation.w = 1.0;
        ocm.absolute_x_axis_tolerance = 0.1;
        ocm.absolute_y_axis_tolerance = 0.1;
        ocm.absolute_z_axis_tolerance = 0.1;
        ocm.weight = 1.0;

        moveit_msgs::msg::Constraints test_constraints;
        test_constraints.orientation_constraints.push_back(ocm);
        move_group_arm_->setPathConstraints(test_constraints);
    }

    
} // robot_arm_node_ns