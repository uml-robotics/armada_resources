#include "ros/ros.h"
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;

class JointStatePublisher
{
protected:

  ros::NodeHandle nh_;
  ros::Subscriber sub;
  ros::Publisher pub;

  sensor_msgs::JointState joint_state;

  tf::TransformBroadcaster broadcaster;

  geometry_msgs::TransformStamped left_outer_knuckle_transform;
  geometry_msgs::TransformStamped left_inner_finger_transform;
  geometry_msgs::TransformStamped left_inner_knuckle_transform;
  geometry_msgs::TransformStamped right_inner_knuckle_transform;
  geometry_msgs::TransformStamped right_outer_knuckle_transform;
  geometry_msgs::TransformStamped right_inner_finger_transform;

  double joint_pos_deg;
  double joint_pos_rad;

public:

  /**
   * Class Constructor.
   *
   * Constructor for JointStatePublisher class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  JointStatePublisher(ros::NodeHandle nh) :
    nh_(nh)
  {
    pub = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    sub = nh_.subscribe("/Robotiq2FGripperRobotInput", 1000, &JointStatePublisher::gripper_joint_pos_callback, this);
    init_transform_frames();
  }

  /**
   * Store the current actual position of the gripper's finger joint.
   *
   * Callback to retrieve and update actual finger position.
   *
   * @param[in] msg robotiq_2f_gripper_control/Robotiq2FGripper_robot_input Message containing status values of r2f85 gripper.
   */
  void gripper_joint_pos_callback(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input& msg)
  {
    // Joint angles appear to all be the "same" when visualized in rviz and no object deforms the grip to a fitted grasp which is good enough for visualization
    // Joint angles appear to go from ~0-45 degrees
    // Remap gripper position from 0-255 to 0-45 and convert to radian for joint pos
    // convert radian variable joint pose to transform for transform broadcaster and publish transforms of joints for robot_state_publisher to handle
    double pos_val = msg.gPO;
    joint_pos_deg = 45 * (pos_val/255);
    joint_pos_rad = joint_pos_deg * M_PI / 180;

    ros::Time stamp = ros::Time(0);
    // joint_state.name = {"finger_joint", "left_inner_finger_joint","left_inner_knuckle_joint", "right_inner_knuckle_joint", "right_outer_knuckle_joint", "right_inner_finger_joint"};
    // joint_state.position = {joint_pos, -joint_pos, joint_pos, joint_pos, joint_pos, -joint_pos};
    joint_state.name = {"finger_joint"};
    joint_state.position = {joint_pos_rad};
    joint_state.header.stamp = stamp;

    pub.publish(joint_state);
  }

  void init_transform_frames()
  {
    // this joint is just called "finger"
    left_outer_knuckle_transform.header.frame_id = "base_link";
    left_outer_knuckle_transform.child_frame_id = "left_outer_knuckle";

    left_inner_finger_transform.header.frame_id = "base_link";
    left_inner_finger_transform.child_frame_id = "left_inner_finger";

    left_inner_knuckle_transform.header.frame_id = "base_link";
    left_inner_knuckle_transform.child_frame_id = "left_inner_knuckle";

    right_inner_knuckle_transform.header.frame_id = "base_link";
    right_inner_knuckle_transform.child_frame_id = "right_inner_knuckle";

    right_outer_knuckle_transform.header.frame_id = "base_link";
    right_outer_knuckle_transform.child_frame_id = "right_outer_knuckle";

    right_inner_finger_transform.header.frame_id = "base_link";
    right_inner_finger_transform.child_frame_id = "right_inner_finger";
  }

  void publish_transforms()
  {
    try {
      tf::TransformListener listener;
      listener.waitForTransform("base_link", "robotiq_arg2f_base_link", ros::Time::now(), ros::Duration(1.0) );
      left_outer_knuckle_transform.transform.rotation = tf::createQuaternionMsgFromYaw(joint_pos_deg);
      left_outer_knuckle_transform.header.stamp = ros::Time(0);;
      broadcaster.sendTransform(left_outer_knuckle_transform);
    } catch (tf::TransformException err) {
      ROS_ERROR("%s", err.what());
    }

    try {
      tf::TransformListener listener;
      listener.waitForTransform("base_link", "left_outer_finger", ros::Time::now(), ros::Duration(1.0) );
      left_inner_finger_transform.transform.rotation = tf::createQuaternionMsgFromYaw(joint_pos_deg);
      left_inner_finger_transform.header.stamp = ros::Time(0);;
      broadcaster.sendTransform(left_inner_finger_transform);
    } catch (tf::TransformException err) {
      ROS_ERROR("%s", err.what());
    }

    try {
      tf::TransformListener listener;
      listener.waitForTransform("base_link", "robotiq_arg2f_base_link", ros::Time::now(), ros::Duration(1.0) );
      left_inner_knuckle_transform.transform.rotation = tf::createQuaternionMsgFromYaw(joint_pos_deg);
      left_inner_knuckle_transform.header.stamp = ros::Time(0);;
      broadcaster.sendTransform(left_inner_knuckle_transform);
    } catch (tf::TransformException err) {
      ROS_ERROR("%s", err.what());
    }

    try {
      tf::TransformListener listener;
      listener.waitForTransform("base_link", "robotiq_arg2f_base_link", ros::Time::now(), ros::Duration(1.0) );
      right_inner_knuckle_transform.transform.rotation = tf::createQuaternionMsgFromYaw(joint_pos_deg);
      right_inner_knuckle_transform.header.stamp = ros::Time(0);;
      broadcaster.sendTransform(right_inner_knuckle_transform);
    } catch (tf::TransformException err) {
      ROS_ERROR("%s", err.what());
    }

    try {
      tf::TransformListener listener;
      listener.waitForTransform("base_link", "robotiq_arg2f_base_link", ros::Time::now(), ros::Duration(1.0) );
      right_outer_knuckle_transform.transform.rotation = tf::createQuaternionMsgFromYaw(joint_pos_deg);
      right_outer_knuckle_transform.header.stamp = ros::Time(0);;
      broadcaster.sendTransform(right_outer_knuckle_transform);
    } catch (tf::TransformException err) {
      ROS_ERROR("%s", err.what());
    }

    try {
      tf::TransformListener listener;
      listener.waitForTransform("base_link", "right_outer_finger", ros::Time::now(), ros::Duration(1.0) );
      right_inner_finger_transform.transform.rotation = tf::createQuaternionMsgFromYaw(joint_pos_deg);
      right_inner_finger_transform.header.stamp = ros::Time(0);;
      broadcaster.sendTransform(right_inner_finger_transform);
    } catch (tf::TransformException err) {
      ROS_ERROR("%s", err.what());
    }
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "r2f85_joint_state_publisher");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  JointStatePublisher jointStatePublisher = JointStatePublisher(nh);
  ROS_WARN("r2f85_joint_state_publisher node Ready.");

  while(ros::ok()){
    // spin
    jointStatePublisher.publish_transforms();
    loop_rate.sleep();
  }

  return 0;
}
