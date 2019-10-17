/**
 * @file freespace_srv_example.cpp
 * @brief Freespace Server implementation
 *
 * @author Chris Lewis
 * @date October 15, 1019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/freespace_srv_example.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/file_write_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;

namespace tesseract_ros_examples
{
  const std::string MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract_rviz";
  const std::string GET_ENVIRONMENT_CHANGES_SERVICE = "get_tesseract_changes_rviz";

  bool FreespaceSrvExample::setup()
  {
    // Set Log Level
    util::gLogLevel = util::LevelInfo;
    
    // Initialize the environment
    ResourceLocatorFn locator = tesseract_rosutils::locateResource;
    if (!tesseract_->init(urdf_xml_string_, srdf_xml_string_, locator)){
      return false;
    }

    // Create plotting tool
    plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>(tesseract_->getEnvironment());
    
    if (rviz_)
      {
	// These are used to keep visualization updated
	modify_env_rviz_ = nh_.serviceClient<tesseract_msgs::ModifyEnvironment>(MODIFY_ENVIRONMENT_SERVICE, false);
	get_env_changes_rviz_ =
	  nh_.serviceClient<tesseract_msgs::GetEnvironmentChanges>(GET_ENVIRONMENT_CHANGES_SERVICE, false);
	
	// Check RViz to make sure nothing has changed
	if (!checkRviz())
	  return false;
      }
    return true;
  }
  bool FreespaceSrvExample::run()
  {
    // Get and Set the initial state of the robot
    std::unordered_map<std::string, double> joint_states;
    joint_states["iiwa_joint_1"] = 0.0;
    joint_states["iiwa_joint_2"] = 0.0;
    joint_states["iiwa_joint_3"] = 0.0;
    joint_states["iiwa_joint_4"] = -1.57;
    joint_states["iiwa_joint_5"] = 0.0;
    joint_states["iiwa_joint_6"] = 0.0;
    joint_states["iiwa_joint_7"] = 0.0;
    tesseract_->getEnvironment()->setState(joint_states);

    if (rviz_)
      {
	// Now update rviz environment
	if (!sendRvizChanges(0))
	  return false;
      }

    ////////////
    /// PICK ///
    ////////////

    if (rviz_)
      {
	ROS_ERROR("Press enter to continue");
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }

    // Choose the manipulator and end effector link
    std::string manip = "Manipulator";
    std::string end_effector = "iiwa_link_ee";

    // Define the final pose
    // TODO change this to a joint pose
    Eigen::Isometry3d final_pose;
    Eigen::Quaterniond orientation(0.0, 0.0, 1.0, 0.0);
    final_pose.linear() = orientation.matrix();
    final_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.77153);  // Offset for the table

    // Define the approach pose
    Eigen::Isometry3d approach_pose = final_pose;
    approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.15);

    // Create the problem construction info
    trajopt::ProblemConstructionInfo pci(tesseract_);

    pci.basic_info.n_steps = steps_ * 2;
    pci.basic_info.manip = manip;
    pci.basic_info.dt_lower_lim = 2;    // 1/most time
    pci.basic_info.dt_upper_lim = 100;  // 1/least time
    pci.basic_info.start_fixed = true;
    pci.basic_info.use_time = false;

    // Create Kinematic Object
    pci.kin = pci.getManipulator(pci.basic_info.manip);

    pci.init_info.type = trajopt::InitInfo::STATIONARY;
    pci.init_info.dt = 0.5;

    // Add a collision cost
    if (true)
      {
	std::shared_ptr<trajopt::CollisionTermInfo> collision(new trajopt::CollisionTermInfo);
	collision->name = "collision";
	collision->term_type = trajopt::TT_COST;
	collision->continuous = true;
	collision->first_step = 1;
	collision->last_step = pci.basic_info.n_steps - 1;
	collision->gap = 1;
	collision->info = trajopt::createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 40);
	pci.cost_infos.push_back(collision);
      }

    // Add a velocity cost without time to penalize paths that are longer
    if (true)
      {
	std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);
	jv->targets = std::vector<double>(7, 0.0);
	jv->coeffs = std::vector<double>(7, 5.0);
	jv->term_type = trajopt::TT_COST;
	jv->first_step = 0;
	jv->last_step = pci.basic_info.n_steps - 1;
	jv->name = "joint_velocity_cost";
	pci.cost_infos.push_back(jv);
      }

    // Add a velocity cnt with time to insure that robot dynamics are obeyed
    if (false)
      {
	std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);

	// Taken from iiwa documentation (radians/s) and scaled by 0.8
	std::vector<double> vel_lower_lim{ 1.71 * -0.8, 1.71 * -0.8, 1.75 * -0.8, 2.27 * -0.8,
	    2.44 * -0.8, 3.14 * -0.8, 3.14 * -0.8 };
	std::vector<double> vel_upper_lim{ 1.71 * 0.8, 1.71 * 0.8, 1.75 * 0.8, 2.27 * 0.8,
	    2.44 * 0.8, 3.14 * 0.8, 3.14 * 0.8 };

	jv->targets = std::vector<double>(7, 0.0);
	jv->coeffs = std::vector<double>(7, 50.0);
	jv->lower_tols = vel_lower_lim;
	jv->upper_tols = vel_upper_lim;
	jv->term_type = (trajopt::TT_CNT | trajopt::TT_USE_TIME);
	jv->first_step = 0;
	jv->last_step = pci.basic_info.n_steps - 1;
	jv->name = "joint_velocity_cnt";
	pci.cnt_infos.push_back(jv);
      }

    // Add cartesian pose cnt at the desired final pose
    if (false)
      {
	Eigen::Quaterniond rotation(final_pose.linear());
	std::shared_ptr<trajopt::CartPoseTermInfo> pose_constraint = std::shared_ptr<trajopt::CartPoseTermInfo>(new trajopt::CartPoseTermInfo);
	pose_constraint->term_type = trajopt::TT_CNT;
	pose_constraint->link = end_effector;
	pose_constraint->timestep = 2 * steps_ - 1;
	pose_constraint->xyz = final_pose.translation();

	pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
	pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
	pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
	pose_constraint->name = "pose_" + std::to_string(2 * steps_ - 1);
	pci.cnt_infos.push_back(pose_constraint);
      }

    // Add a cost on the total time to complete the move
    if (false)
      {
	std::shared_ptr<trajopt::TotalTimeTermInfo> time_cost(new trajopt::TotalTimeTermInfo);
	time_cost->name = "time_cost";
	time_cost->coeff = 5.0;
	time_cost->limit = 0.0;
	time_cost->term_type = trajopt::TT_COST;
	pci.cost_infos.push_back(time_cost);
      }

    // Create the pick problem
    trajopt::TrajOptProb::Ptr move_problem = ConstructProblem(pci);

    // Set the optimization parameters (Most are being left as defaults)
    tesseract_motion_planners::TrajOptPlannerConfig config(move_problem);
    config.params.max_iter = 100;

    // Create Plot Callback
    if (plotting_)
      {
	config.callbacks.push_back(PlotCallback(*move_problem, plotter_));
      }

    // Create file write callback discarding any of the file's current contents
    std::shared_ptr<std::ofstream> stream_ptr(new std::ofstream);
    if (write_to_file_)
      {
	std::string path = ros::package::getPath("tesseract_ros_examples") + "/file_output_pick.csv";
	stream_ptr->open(path, std::ofstream::out | std::ofstream::trunc);
	config.callbacks.push_back(trajopt::WriteCallback(stream_ptr, move_problem));
      }

    // Create the planner and the responses that will store the results
    tesseract_motion_planners::TrajOptMotionPlanner planner;
    tesseract_motion_planners::PlannerResponse planning_response;
    tesseract_motion_planners::PlannerResponse planning_response_place;

    // Set Planner Configuration
    planner.setConfiguration(std::make_shared<tesseract_motion_planners::TrajOptPlannerConfig>(config));

    // Solve problem. Results are stored in the response
    planner.solve(planning_response);

    if (write_to_file_)
      stream_ptr->close();

    // Plot the resulting trajectory
    if (plotting_)
      {
	long num_joints = static_cast<long>(move_problem->GetKin()->getJointNames().size());
	auto joint_names = move_problem->GetKin()->getJointNames();
	auto columns = planning_response.joint_trajectory.trajectory.leftCols(num_joints);
	plotter_->plotTrajectory(joint_names, columns);
      }

    ROS_INFO("Done");
    return true;
  }
}  // namespace tesseract_ros_examples
