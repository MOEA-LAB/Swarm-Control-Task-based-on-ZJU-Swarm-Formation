#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <plan_manage/planner_manager.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <traj_utils/Assignment.h>
#include <traj_utils/DataDisp.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/planning_visualization.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>
using std::vector;

namespace ego_planner
{

class EGOReplanFSM
{
 private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE
  {
    INIT,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP,
    SEQUENTIAL_START
  };
  enum TARGET_TYPE
  {
    MANUAL_TARGET = 1,
    PRESET_TARGET,
    SWARM_MANUAL_TARGET
  };

  /* planning utils */
  EGOPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;
  traj_utils::DataDisp data_disp_;

  /* parameters */
  int target_type_;  // 1 mannual select, 2 hard code
  double no_replan_thresh_, replan_thresh_;
  double waypoints_[50][3];
  int waypoint_num_;
  int goal_num_;
  double goalpoints_[50][3];
  double planning_horizen_, planning_horizen_time_;
  double emergency_time_;
  bool flag_realworld_experiment_;
  bool enable_fail_safe_;
  int last_end_id_;
  double replan_trajectory_time_;

  // global goal setting for swarm
  Eigen::Vector3d swarm_central_pos_;
  double swarm_relative_pts_y_[50][3];
  double swarm_relative_pts_u_[50][3];
  double swarm_relative_pts_s1_[50][3];
  double swarm_relative_pts_s2_[50][3];

  int times = 0;
  geometry_msgs::PoseStamped last_goal_msg;

  double swarm_scale_;

  /* planning data */
  bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_,
      have_local_traj_;
  FSM_EXEC_STATE exec_state_;
  int continously_called_times_{0};

  Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;  // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d end_pt_, end_vel_;                                        // goal state
  Eigen::Vector3d local_target_pt_, local_target_vel_;  // local target state
  int current_wp_;

  bool flag_escape_emergency_;
  bool flag_relan_astar_;

  GlobalTrajData frontend_traj_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_;
  ros::Subscriber waypoint_sub_, odom_sub_, swarm_trajs_sub_, broadcast_bspline_sub_,
      trigger_sub_, assignment_sub_;
  ros::Publisher replan_pub_, new_pub_, poly_traj_pub_, data_disp_pub_, swarm_trajs_pub_,
      broadcast_bspline_pub_;
  ros::Publisher broadcast_ploytraj_pub_;
  ros::Publisher reached_pub_, start_pub_;
  ros::Subscriber central_goal;
  ros::Subscriber broadcast_ploytraj_sub_;
  // result file and file name
  string result_fn_;
  fstream result_file_;
  /* helper functions */
  bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj,
                         bool use_formation);        // front-end and back-end method
  bool callEmergencyStop(Eigen::Vector3d stop_pos);  // front-end and back-end method
  bool planFromGlobalTraj(const int trial_times = 1);
  bool planFromLocalTraj(bool flag_use_poly_init, bool use_formation);

  /* return value: std::pair< Times of the same state be continuously called, current
   * continuously called state > */
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
  void printFSMExecState();

  void planGlobalTrajbyGivenWps();
  // void getLocalTarget();

  /* ROS functions */
  void execFSMCallback(const ros::TimerEvent &e);
  void checkCollisionCallback(const ros::TimerEvent &e);
  void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
  void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void RecvBroadcastPolyTrajCallback(const traj_utils::PolyTrajConstPtr &msg);
  void polyTraj2ROSMsg(traj_utils::PolyTraj &msg);
  void formationWaypointCallback(const geometry_msgs::PoseStampedPtr &msg);
  bool frontEndPathSearching();
  bool checkCollision();

 public:
  EGOReplanFSM(/* args */) {}
  ~EGOReplanFSM();

  void init(ros::NodeHandle &nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace ego_planner

#endif