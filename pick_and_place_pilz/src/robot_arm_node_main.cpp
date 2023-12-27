// ros2 launch moveit2_tutorials mtc_demo.launch.py 


#include <rclcpp/rclcpp.hpp>

#include <pick_and_place_pilz/robot_arm_node.hpp>
#include <geometry_msgs/msg/pose.hpp>


int main(int argc, char** argv)
{
    // init ros
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    // options.automatically_declare_parameters_from_overrides(true);

    // create nodes
    auto rb = std::make_shared<robot_arm_node_ns::robot_arm_node>(options);

    // create executors
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(rb->getNodeBaseInterface());

    // create thread
    std::thread([&executor](){executor.spin();}).detach(); //join(); detach();

    auto const pre_grasp_pose = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose.orientation.x = 1.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 0.0;
      msg.pose.position.x = 0.6;
      msg.pose.position.y = -0.2;
      msg.pose.position.z = 0.6;
      return msg;
    }();

    auto const grasp_pose = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose.orientation.x = 1.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 0.0;
      msg.pose.position.x = 0.6;
      msg.pose.position.y = -0.2;
      msg.pose.position.z = 0.4;
      return msg;
    }();

    auto const goal_pose = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose.orientation.x = 1.0; //0.7071;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 0; //0.7071;
      msg.pose.position.x = 0.6;
      msg.pose.position.y = 0.0;
      msg.pose.position.z = 0.6;
      return msg;
    }();

    auto const center_pose = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose.orientation.x = 1.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 0.0;
      msg.pose.position.x = 0.6;
      msg.pose.position.y = 0.0;
      msg.pose.position.z = 0.4;
      return msg;
    }();

    rb->go_to_home();
    
    // for pilz, mode = LIN, PTP or CIRC
    while (1){
        rb->move2target(pre_grasp_pose, "LIN");
        rb->move2target(grasp_pose, "LIN");
        rb->grasp();
        rb->move2target(goal_pose, "LIN");
        rb->move2target(center_pose, "LIN");
        rb->release();
    };


    rclcpp::shutdown();
    return 0;

} // main