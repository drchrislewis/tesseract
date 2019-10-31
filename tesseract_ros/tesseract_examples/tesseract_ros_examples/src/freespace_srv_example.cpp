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
  FreespaceSrvExample::FreespaceSrvExample(std::string jv_srv_name, int steps)
    :joint_value_server_(nh_, jv_srv_name, boost::bind(&FreespaceSrvExample::jointValueCallBack, this, _1), false),
      steps_(steps)
  {
    ros::NodeHandle pnh("~");
    
    // get all the necessary parameters
    if(!nh_.getParam("robot_description", urdf_xml_string_))
      {
	ROS_ERROR("FreespaceSrvExample must have robot_description defined on parameter server");
      }
    if(!nh_.getParam("robot_description_semantic", srdf_xml_string_))
      {
	ROS_ERROR("FreespaceSrvExample must have robot_description_semantic defined on parameter server");
      }

    // get private parameters
    if(!pnh.getParam("manipulator", manipulator_))
      {
	ROS_ERROR("FreespaceSrvExample must have manipulator defined on parameter server");
      }
    if(!pnh.getParam("end_effector", end_effector_))
      {
	ROS_ERROR("FreespaceSrvExample must have end_effector defined on parameter server");
      }
  }

  bool FreespaceSrvExample::setup()
  {
    // Initialize tesseract environment
    ResourceLocatorFn locator = tesseract_rosutils::locateResource;
    if (!tesseract_->init(urdf_xml_string_, srdf_xml_string_, locator)){
      ROS_ERROR("Failed to initialize tesseract envioronment");
      return false;
    }
    return true;
  }
  
  void FreespaceSrvExample::jointValueCallBack(const control_msgs::JointTrajectoryGoalConstPtr& goal)
  {
    /*
    // Get and Set the initial state of the robot
    std::unordered_map<std::string, double> joint_states;
    tesseract_->getEnvironment()->getState(joint_states);
    
    if(joint_states.size() != goal->trajectory.points.size())
      {
	ROS_ERROR("FreespaceSrvExample::jointValueCallback() goal.points and joint_states size difference %d %d",
		  joint_states.size(),
		  goal->trajectory.points.size());
	return;
      }
    long unsigned int N_joints = (long unsigned int) joint_states.size();
    
    // set desired joint states using the action's goal assuming there is only one point goal
    desired_joint_states_.clear();
    for(long unsigned int i=0; i< N_joints; i++){
      std::string jname = goal->trajectory.joint_names[i];
      double jvalue = goal->trajectory.points[0].positions[i];
      desired_joint_states_.insert({jname, jvalue});
    }
    // Create the problem construction info
    trajopt::ProblemConstructionInfo pci(tesseract_);
    pci.basic_info.n_steps = steps_ * 2;
    pci.basic_info.manip = manipulator_;
    pci.basic_info.dt_lower_lim = 2;    // 1/most time
    pci.basic_info.dt_upper_lim = 100;  // 1/least time
    pci.basic_info.start_fixed = true;
    pci.basic_info.use_time = false;
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

    // Add joint pose cnt at the desired final joint pose
    if (true)
      {
	// why not use trajectory_cost.hpp: JointPoseEqCost?
	//   JointPosEqCost(const VarArray& traj,
	//                 const Eigen::VectorXd& coeffs,
	//                 const Eigen::VectorXd& targets,
	//                 int& first_step,
	//                 int& last_step);
	// why not use a trajectory_cost.hpp: JointPosEqConstraint?
	//	  JointPosEqConstraint(const VarArray& traj,
	//                       const Eigen::VectorXd& coeffs,
	//                       const Eigen::VectorXd& targetss,
	//                       int& first_step,
	//                       int& last_step);
	// base class TermInfo supports the following interface:
	// getSupportedTypes()
	// fromJson(ProblemConstructionInfo& pci, const Json::Value& v) = 0;
	// hatch(TrajOptProb& prob) = 0;
	// TermInfo::Ptr fromName(const std::string& type);
	// it also has some wierd things

	std::shared_ptr<trajopt::JointPosTermInfo> pose_constraint = std::shared_ptr<trajopt::JointPosTermInfo>(new trajopt::JointPosTermInfo);
	// set items specific to JointPosTermInfo
	pose_constraint->coeffs     = std::vector<double>(7, 50.0); // DblVec coef to scale cost per joint
	pose_constraint->targets.clear(); // DblVec target positions
	for(long unsigned int i=0; i< N_joints; i++)
	  {
	    pose_constraint->targets.push_back(desired_joint_states_[goal->trajectory.joint_names[i]]);
	  }
	pose_constraint->upper_tols = std::vector<double>(N_joints, 0.0); // DblVec upper limit on each joint
	pose_constraint->lower_tols = std::vector<double>(N_joints, 0.0); // DblVec lower limit on each joint
	pose_constraint->first_step = 0; // int first time step in which this constraint is applied
	pose_constraint->last_step  = -1; // int last time step in which this constraint is applied
	// set base class TermInfo items
	pose_constraint->name = "joint_pose_";
	pose_constraint->term_type = trajopt::TT_CNT; // can be one of TT_COST, TT_CNT, TT_USE_TIME

	pci.cnt_infos.push_back(pose_constraint);
      }

    // Add a cost on the total time to complete the move
    if (true)
      {
	std::shared_ptr<trajopt::TotalTimeTermInfo> time_cost(new trajopt::TotalTimeTermInfo);
	time_cost->name = "time_cost";
	time_cost->coeff = 5.0;
	time_cost->limit = 0.0;
	time_cost->term_type = trajopt::TT_COST;
	pci.cost_infos.push_back(time_cost);
      }

    // Create the move problem
    trajopt::TrajOptProb::Ptr move_problem = ConstructProblem(pci);

    // Set the optimization parameters (Most are being left as defaults)
    tesseract_motion_planners::TrajOptPlannerConfig config(move_problem);
    config.params.max_iter = 100;

    // Create the planner and the responses that will store the results
    tesseract_motion_planners::TrajOptMotionPlanner planner;
    tesseract_motion_planners::PlannerResponse planning_response;
    tesseract_motion_planners::PlannerResponse planning_response_place;

    // Set Planner Configuration
    planner.setConfiguration(std::make_shared<tesseract_motion_planners::TrajOptPlannerConfig>(config));

    // Solve problem. Results are stored in the response
    planner.solve(planning_response);
    */
  }



}  // namespace tesseract_ros_examples
