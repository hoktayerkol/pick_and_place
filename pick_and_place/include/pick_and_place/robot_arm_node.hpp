#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose.hpp>

namespace robot_arm_node_ns
{

    class robot_arm_node
    {
        private:
            //rclcpp::NodeOptions node_options_;

            std::string planning_group;
            std::string arm_group_name;
            std::string hand_group_name;
            std::string hand_frame;
            //moveit::planning_interface::MoveGroupInterface move_group;

            rclcpp::Node::SharedPtr node_;

            // move grouo and planning group
            moveit::planning_interface::MoveGroupInterfacePtr move_group_;
            moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface_;
            moveit::planning_interface::MoveGroupInterface::Plan moving_plan_;

            


        public:
            robot_arm_node(const rclcpp::NodeOptions& options);
            ~robot_arm_node(){};

            rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

            void move2target(geometry_msgs::msg::Pose target_pose);

            //std::shared_ptr<rclcpp::Logger> logger;
            


    };
}
