// ros2 launch moveit2_tutorials mtc_demo.launch.py 


#include <rclcpp/rclcpp.hpp>

#include <pick_and_place/robot_arm_node.hpp>
#include <geometry_msgs/msg/pose.hpp>


int main(int argc, char** argv)
{
    // init ros
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    // create nodes
    auto rb = std::make_shared<robot_arm_node_ns::robot_arm_node>(options);

    // create executors
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(rb->getNodeBaseInterface());

    // creater thread
    std::thread([&executor](){executor.spin();}).detach(); //join(); detach();


    geometry_msgs::msg::Pose target_pose1;
    target_pose1.position.x = 0.32;
    target_pose1.position.y = -0.44;
    target_pose1.position.z = 0.32;

    
    tf2::Quaternion myq;
    myq.setRPY(3.14,0,2*M_PI);
    myq.normalize();

    target_pose1.orientation.w = myq.getW();
    target_pose1.orientation.x = myq.getX();
    target_pose1.orientation.y = myq.getY();
    target_pose1.orientation.z = myq.getZ();

    rb->move2target(target_pose1);

    rclcpp::shutdown();
    return 0;

} // main