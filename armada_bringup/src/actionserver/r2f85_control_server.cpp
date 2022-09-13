#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommand.h>
#include <control_msgs/GripperCommandAction.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>


class R2f85GripperControlAction
{
protected:

  ros::NodeHandle nh_;
  ros::Publisher gripper_pub;
  ros::Subscriber gripper_sub;
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output gripper_cmd;
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_input gripper_state;
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> R2f85GripperCommandServer_;
  control_msgs::GripperCommandFeedback r2f85_gripper_feedback_;
  control_msgs::GripperCommandResult r2f85_gripper_result_;
  double gPO;
  double gripper_pos_val;

public:

  /**
   * Class Constructor.
   *
   * Constructor for R2f85GripperControlAction class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  R2f85GripperControlAction(ros::NodeHandle nh) :
    nh_(nh),
    R2f85GripperCommandServer_(nh, "r2f85_gripper_command", boost::bind(&R2f85GripperControlAction::r2f85GripperCommand, this, _1), false)
  {
    gripper_pub = nh_.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput", 10);
    gripper_sub = nh_.subscribe("/Robotiq2FGripperRobotInput", 10, &R2f85GripperControlAction::gripperStateCallback, this);
    R2f85GripperCommandServer_.start();
  }

  /**
   * Plan a cartesian path.
   *
   * Plan a cartesian path along one or more set points using the MoveIt interface.
   *
   * @param goal GripperCommand action goal: control_msgs/GripperCommand, a gripper command target position.
   */
  void r2f85GripperCommand(const control_msgs::GripperCommandGoalConstPtr &goal)
  {
    if (!gripper_state.gACT && !gripper_state.gGTO)
    {
      gripper_cmd.rACT = 1;
      gripper_cmd.rGTO = 1;
      gripper_cmd.rSP = 255;
      gripper_cmd.rFR = 150;
      gripper_pub.publish(gripper_cmd);
      while (!gripper_state.gGTO && !gripper_state.gFLT && ros::ok())
      {
        // wait until gripper activation sequence is complete
      }
    }
    // map goal position into range 0->255
    double target_position = goal->command.position * 255;
    // send command to go to position
    gripper_cmd.rPR = int(target_position);
    gripper_pub.publish(gripper_cmd);
    while (gripper_state.gOBJ != 0 && !gripper_state.gFLT && ros::ok())
    {
      // wait until gripper starts moving
    }
    // wait until gripper gets to position, stalls, or faults
    while (gripper_state.gOBJ == 0 && !gripper_state.gFLT && ros::ok())
    {
      // get current gripper position (##/255) and convert it to a decimal val (0-1)
      gPO = gripper_state.gPO;
      gripper_pos_val = gPO/255;
      r2f85_gripper_feedback_.position = gripper_pos_val;
      R2f85GripperCommandServer_.publishFeedback(r2f85_gripper_feedback_);
      ros::Duration(0.1).sleep();
    }
    // get current gripper position (##/255) and convert it to a decimal val (0-1)
    gPO = gripper_state.gPO;
    gripper_pos_val = gPO/255;
    r2f85_gripper_result_.position = gripper_pos_val;
    switch (gripper_state.gOBJ)
    {
    case 1: // gripper faulted
      r2f85_gripper_result_.stalled = 0;
      r2f85_gripper_result_.reached_goal = 0;
      R2f85GripperCommandServer_.setAborted(r2f85_gripper_result_);
      R2f85GripperCommandServer_.setSucceeded(r2f85_gripper_result_);
      break;
    case 2: // stalled
      r2f85_gripper_result_.stalled = 1;
      r2f85_gripper_result_.reached_goal = 1;
      R2f85GripperCommandServer_.setSucceeded(r2f85_gripper_result_);
      break;
    case 3: // reached
      r2f85_gripper_result_.stalled = 0;
      r2f85_gripper_result_.reached_goal = 1;
      R2f85GripperCommandServer_.setSucceeded(r2f85_gripper_result_);
      break;
    default:
      // something went wrong
      r2f85_gripper_result_.stalled = 0;
      r2f85_gripper_result_.reached_goal = 0;
      R2f85GripperCommandServer_.setSucceeded(r2f85_gripper_result_);
      break;
    }
  }

  /**
   * Store the current state of the gripper.
   *
   * A callback function to store the current full state of the gripper to the R2f85GripperControlAction object for use in sending commands and returning feedback.
   *
   * @param msg Robotiq2FGripper input message: robotiq_2f_gripper_control/Robotiq2FGripper_robot_input, a robotiq gripper status message.
   * @return Success of motion plan execution.
   */
  void gripperStateCallback(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input::ConstPtr& msg)
  {
    this->gripper_state = *msg;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "r2f85_gripper_command_action_server");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  R2f85GripperControlAction cartesian_planning_cpp_server(nh);
  ROS_WARN("r2f85_gripper_command_action_server Ready.");

  while(ros::ok()){
    // spin
  }

  return 0;
}
