/**
 ** application node
 **/
#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <myworkcell_core/PlanCartesianPath.h>


class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh) : ac_("joint_trajectory_action", true)
  {
    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
    cartesian_client_ = nh.serviceClient<myworkcell_core::PlanCartesianPath>("plan_path");
  }

  void start(const std::string& base_frame)
  {
    ROS_INFO("Attempting to localize part");
    // Localize the part
    myworkcell_core::LocalizePart srv;
    srv.request.base_frame = base_frame;
    ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

    if(!vision_client_.call(srv))
    {
      ROS_ERROR("Could not localize part");
      return;
    }
    ROS_INFO_STREAM("part localized: " << srv.response);
    geometry_msgs::Pose move_target = srv.response.pose;
    moveit::planning_interface::MoveGroup move_group("manipulator");
    // Plan for robot to move to part
    move_group.setPoseTarget(move_target);
    move_group.move();
    
    // Plan cartesian path
    myworkcell_core::PlanCartesianPath cartesian_srv;
    cartesian_srv.request.pose = move_target;
    if(!cartesian_client_.call(cartesian_srv))
    {
        ROS_ERROR("Could not plan for path");
        return;
    }
    // Execute descartes-planned path directly (bypass MoveIt)
    ROS_INFO("Got cart path, executing");
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = cartesian_srv.response.trajectory;
    ac_.sendGoal(goal);
    ac_.waitForResult();
    ROS_INFO("Done");
  }

private:
  // planning components
  ros::ServiceClient vision_client_;
  ros::ServiceClient cartesian_client_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "myworkcell_node");
  // async move
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle private_node_handle ("~");


  ROS_INFO("ScanNPlan node has been initialized");

  std::string base_frame;
  private_node_handle.param<std::string>("base_frame", base_frame, "world"); // (key, &value, default)

  ScanNPlan app(nh);
  ros::Duration(.5).sleep();  // wait for the class to initialized.
  app.start(base_frame);

  /*
  moveit::planning_interface::MoveGroup move_group("manipulator");
  ROS_INFO("Move back to AllZeros Pose.");
  move_group.setNamedTarget("AllZeros");
  move_group.move();

  ros::Duration(5).sleep();  // wait for the class to initialized.
  ROS_INFO("Move back to home Pose.");
  move_group.setNamedTarget("home");
  move_group.move();
  */

  //ros::spin();
  ros::waitForShutdown(); // for async move robot arm
  return 0;
}
