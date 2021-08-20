#pragma once

namespace joint_trajectory_controller
{
template <class SegmentImpl, class HardwareInterface>
GitaiJointTrajectoryController<SegmentImpl, HardwareInterface>::GitaiJointTrajectoryController()
{
  if (this->verbose_)
  {
    ROS_WARN_STREAM("Gitai JointTrajectoryController. ");
  }
}

template <class SegmentImpl, class HardwareInterface>
bool GitaiJointTrajectoryController<SegmentImpl, HardwareInterface>::updateTrajectoryCommand(
    const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh, std::string* error_string)
{
  ROS_INFO_STREAM("Gitai updataTrajectoryCommand");
  typedef GitaiInitJointTrajectoryOptions<Trajectory> Options;
  Options options;
  options.error_string = error_string;
  std::string error_string_tmp;

  // Preconditions
  if (!this->isRunning())
  {
    error_string_tmp = "Can't accept new commands. Controller is not running.";
    ROS_ERROR_STREAM_NAMED(this->name_, error_string_tmp);
    options.setErrorString(error_string_tmp);
    return false;
  }

  if (!msg)
  {
    error_string_tmp = "Received null-pointer trajectory message, skipping.";
    ROS_WARN_STREAM_NAMED(this->name_, error_string_tmp);
    options.setErrorString(error_string_tmp);
    return false;
  }

  // Time data
  TimeData* time_data = this->time_data_.readFromRT();  // TODO: Grrr, we need a lock-free data structure here!

  // Time of the next update
  const ros::Time next_update_time = time_data->time + time_data->period;

  // Uptime of the next update
  ros::Time next_update_uptime = time_data->uptime + time_data->period;

  // Hold current position if trajectory is empty
  if (msg->points.empty())
  {
    this->setHoldPosition(time_data->uptime, gh);
    ROS_DEBUG_NAMED(this->name_, "Empty trajectory command, stopping.");
    return true;
  }

  // Trajectory initialization options
  TrajectoryPtr curr_traj_ptr;
  this->curr_trajectory_box_.get(curr_traj_ptr);

  options.other_time_base = &next_update_uptime;
  options.current_trajectory = curr_traj_ptr.get();
  options.joint_names = &this->joint_names_;
  options.vel_limits = &this->vel_limits_;
  options.angle_wraparound = &this->angle_wraparound_;
  options.rt_goal_handle = gh;
  options.default_tolerances = &this->default_tolerances_;
  options.allow_partial_joints_goal = this->allow_partial_joints_goal_;

  // Update currently executing trajectory
  try
  {
    TrajectoryPtr traj_ptr(new Trajectory);
    *traj_ptr = gitaiInitJointTrajectory<Trajectory>(*msg, next_update_time, options);
    if (!traj_ptr->empty())
    {
      this->curr_trajectory_box_.set(traj_ptr);
    }
    else
    {
      return false;
    }
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM_NAMED(this->name_, ex.what());
    options.setErrorString(ex.what());
    return false;
  }
  catch (...)
  {
    error_string_tmp = "Unexpected exception caught when initializing trajectory from ROS message data.";
    ROS_ERROR_STREAM_NAMED(this->name_, error_string_tmp);
    options.setErrorString(error_string_tmp);
    return false;
  }

  return true;
}

}  // namespace joint_trajectory_controller
