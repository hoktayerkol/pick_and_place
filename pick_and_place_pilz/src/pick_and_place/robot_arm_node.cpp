#include <rclcpp/rclcpp.hpp>
#include "pick_and_place_pilz/robot_arm_node.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace robot_arm_node_ns
{

    robot_arm_node::robot_arm_node(const rclcpp::NodeOptions& options) : moving_plan_()
    {
        node_ = std::make_shared<rclcpp::Node>("robot_arm", options);

        // auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        // param_desc.description = "Robot's arm group";
        // node_->declare_parameter("arm_group_name", "arm_group", param_desc);
        // arm_group_name = node_->get_parameter("arm_group_name").as_string();

        // param_desc.description = "Robot's hand group";
        // node_->declare_parameter("hand_group_name", "hand_group", param_desc);
        // hand_group_name = node_->get_parameter("hand_group_name").as_string();

        // param_desc.description = "Robot's hand frame";
        // node_->declare_parameter("hand_frame_name", "hand_frame", param_desc);
        // hand_frame_name = node_->get_parameter("hand_frame_name").as_string();

        // for panda
        // arm_group_name = "panda_arm";
        // hand_group_name = "hand";
        // hand_frame_name = "panda_hand";

        // for ur
        arm_group_name = "arm_group";
        hand_group_name = "hand_group";
        hand_frame_name = "tool0";
        robot_base_link = "base_link";

        move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_group_name);
        move_group_hand_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, hand_group_name);
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        

        visualize_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node_, 
                    robot_base_link, rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_arm_->getRobotModel());
        visualize_->deleteAllMarkers();
        visualize_->loadRemoteControl();


        //move_group_arm_->setMaxAccelerationScalingFactor(0.5);
        //move_group_arm_->setMaxVelocityScalingFactor(0.5);

        //move_group_hand_->setMaxAccelerationScalingFactor(0.02);
        //move_group_hand_->setMaxVelocityScalingFactor(0.02);
        
        //logger = std::make_shared<rclcpp::Logger>("robot_arm");

    }

    // getNodeBaseInterface()
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr robot_arm_node::getNodeBaseInterface()
    {
        return node_->get_node_base_interface();
    }

    // setNamedTarget
    // in the srdf: read, extended. transport for arm and open and close for hand
    // move2target()
    void robot_arm_node::move2target(geometry_msgs::msg::PoseStamped target_pose, std::string mode)
    {
        move_group_arm_->setPlannerId(mode);

        move_group_arm_->setPoseTarget(target_pose, hand_frame_name);
        // move_group_arm_->setPoseTarget(target_pose);
        
        // move_group_arm_->setPoseTarget(target_pose, move_group_arm_->getEndEffectorLink());
        
        if (move_group_arm_->plan(moving_plan_)==moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("robot_arm"), "plan successfull!");
            rviz_visualize("plan succesful!"); //, moving_plan_);
            move_group_arm_->execute(moving_plan_);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("robot_arm"), "move2target plan failed!");
            return;
        };

        //return;
    }

    // grasp and release aproach and retreat

    void robot_arm_node::grasp()
    {

        // set hand target joint values
        // move_group_hand_->setJointValueTarget(std::vector<double>{0.00,0.00});
        move_group_hand_->setNamedTarget("close");
        // create a plan object to store generated plan
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        if (move_group_hand_->plan(moving_plan_)==moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("robot_arm"), "plan successfull!");
            move_group_hand_->execute(moving_plan_);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("robot_arm"), "hand close plan failed!");
            return;
        };

        // execute the plan
        move_group_hand_->execute(my_plan);

        return;
    }

    void robot_arm_node::release()
    {

        // set hand target joint values
        //move_group_hand_->setJointValueTarget(std::vector<double>{0.05,0.05});
        move_group_hand_->setNamedTarget("open");
        // create a plan object to store generated plan
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        if (move_group_hand_->plan(moving_plan_)==moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("robot_arm"), "plan successfull!");
            move_group_hand_->execute(moving_plan_);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("robot_arm"), "hand open plan failed!");
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

    void robot_arm_node::rviz_visualize(std::string text) //, moveit::planning_interface::MoveGroupInterface::Plan plan)
    {
        auto pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);

        visualize_->publishText(pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
        visualize_->trigger();

        visualize_->prompt(text);
        visualize_->trigger();

        auto jmg = move_group_arm_->getRobotModel()->getJointModelGroup(arm_group_name);
        visualize_->publishTrajectoryLine(moving_plan_.trajectory_, jmg);
        visualize_->trigger();

    }
    
    void robot_arm_node::go_to_home()
    {

        // set hand target joint values
        //move_group_hand_->setJointValueTarget(std::vector<double>{0.05,0.05});
        move_group_arm_->setNamedTarget("home");
        // create a plan object to store generated plan
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        if (move_group_arm_->plan(moving_plan_)==moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("robot_arm"), "plan successfull!");
            move_group_arm_->execute(moving_plan_);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("robot_arm"), "got to home plan failed!");
            return;
        };

        // execute the plan
        move_group_hand_->execute(my_plan);

        move_group_arm_->setPlanningPipelineId("pilz_industrial_motion_planner");
        
        return;
    }
} // robot_arm_node_ns