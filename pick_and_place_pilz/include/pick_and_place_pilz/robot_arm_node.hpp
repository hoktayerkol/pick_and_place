#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace robot_arm_node_ns
{

    class robot_arm_node
    {
        private:
            //rclcpp::NodeOptions node_options_;

            std::string planning_group;
            std::string arm_group_name;
            std::string hand_group_name;
            std::string hand_frame_name;
            std::string robot_base_link;
            //moveit::planning_interface::MoveGroupInterface move_group;

            

            // move grouo and planning group
            moveit::planning_interface::MoveGroupInterfacePtr move_group_arm_;
            moveit::planning_interface::MoveGroupInterfacePtr move_group_hand_;
            moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface_;
            
            rclcpp::Node::SharedPtr node_;
            
            moveit::planning_interface::MoveGroupInterface::Plan moving_plan_;
            moveit_visual_tools::MoveItVisualToolsPtr visualize_;



        public:
            robot_arm_node(const rclcpp::NodeOptions& options);
            ~robot_arm_node(){};

            rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

            void move2target(geometry_msgs::msg::PoseStamped target_pose, std::string mode);

            //std::shared_ptr<rclcpp::Logger> logger;

            void grasp();
            void release();

            void set_constraints();
            
            void rviz_visualize(std::string text); //, moveit::planning_interface::MoveGroupInterface::Plan plan);
            
            

    };
}
