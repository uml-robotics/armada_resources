#include "ros/ros.h"
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

using namespace std;

class JointStatePublisher
{
protected:

  ros::NodeHandle nh_;
  ros::Subscriber sub;
  ros::Publisher pub;

  sensor_msgs::JointState joint_state;

  tf::TransformBroadcaster broadcaster;

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
  }

  /**
   * Store the current actual position of the gripper's finger joint.
   *
   * Callback to retrieve and update actual finger position. Finger joint angles are a product of the position value/255 across a 45 degree span.
   *
   * @param[in] msg robotiq_2f_gripper_control/Robotiq2FGripper_robot_input Message containing status values of r2f85 gripper.
   */
  void gripper_joint_pos_callback(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input& msg)
  {
    double pos_val = msg.gPO;
    joint_pos_deg = 45 * (pos_val/255);
    joint_pos_rad = joint_pos_deg * M_PI / 180;

    ros::Time stamp = ros::Time::now();
    joint_state.name = {"finger_joint"};
    joint_state.position = {joint_pos_rad};
    joint_state.header.stamp = stamp;

    pub.publish(joint_state);
  }

  void publish_transforms()
  {
    std::vector<tf::StampedTransform> finger_tf_vector;
    tf::StampedTransform stamped_finger_tf;
    tf::Quaternion finger_joint_tf;

    std::vector<string> finger_links = {"left_outer_knuckle", "right_outer_knuckle", "left_inner_finger", "right_inner_finger"};
    std::vector<int> joint_direction = {1, -1, 1, -1};

    for (long i = 0; i < finger_links.size(); i++)
    {
      finger_joint_tf.setEuler(0,0,joint_pos_deg * joint_direction[i]);
      finger_joint_tf.normalize();

      try {
        tf::TransformListener listener;
        listener.waitForTransform("robotiq_arg2f_base_link", "base_link", ros::Time(0), ros::Duration(3.0) );
        listener.lookupTransform("robotiq_arg2f_base_link", "base_link", ros::Time(0), stamped_finger_tf);
      } catch (tf::TransformException err) {
        ROS_ERROR("%s", err.what());
      }

      tf::Transform finger_tf_(finger_joint_tf, tf::Vector3(0, 0, 0));
      tf::Transform finger_tf = stamped_finger_tf * finger_tf_;

      stamped_finger_tf.setData(finger_tf);
      stamped_finger_tf.child_frame_id_ = finger_links[i];
      stamped_finger_tf.frame_id_ = "robotiq_arg2f_base_link";
      stamped_finger_tf.stamp_ = ros::Time::now();

      finger_tf_vector.clear();
      finger_tf_vector.push_back(stamped_finger_tf);
      broadcaster.sendTransform(finger_tf_vector);
    }
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "r2f85_joint_state_publisher");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  JointStatePublisher jointStatePublisher = JointStatePublisher(nh);
  ROS_WARN("r2f85_joint_state_publisher node Ready.");

  while(ros::ok()){
    jointStatePublisher.publish_transforms();
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
