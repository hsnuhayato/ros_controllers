#pragma once

#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <joint_trajectory_controller/gitai_init_joint_trajectory.h>

namespace joint_trajectory_controller
{
template <class SegmentImpl, class HardwareInterface>
class GitaiJointTrajectoryController : public JointTrajectoryController<SegmentImpl, HardwareInterface>
{
public:
  GitaiJointTrajectoryController();

protected:
  using typename JointTrajectoryController<SegmentImpl, HardwareInterface>::Trajectory;
  using typename JointTrajectoryController<SegmentImpl, HardwareInterface>::TrajectoryPtr;
  using typename JointTrajectoryController<SegmentImpl, HardwareInterface>::JointTrajectoryConstPtr;
  using typename JointTrajectoryController<SegmentImpl, HardwareInterface>::RealtimeGoalHandlePtr;
  using typename JointTrajectoryController<SegmentImpl, HardwareInterface>::TimeData;

  virtual bool updateTrajectoryCommand(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh,
                                       std::string* error_string = nullptr) override;
};
}  // namespace joint_trajectory_controller

#include <joint_trajectory_controller/gitai_joint_trajectory_controller_impl.h>
