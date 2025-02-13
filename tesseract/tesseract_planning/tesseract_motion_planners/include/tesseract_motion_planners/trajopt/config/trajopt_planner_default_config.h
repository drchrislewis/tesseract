/**
 * @file trajopt_planner_default_config.h
 * @brief A TrajOpt planner configuration class with default values suitable for most applications
 *
 * @author Michael Ripperger
 * @date September 16, 2019
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_PLANNER_DEFAULT_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_PLANNER_DEFAULT_CONFIG_H

#include <tesseract_motion_planners/trajopt/config/trajopt_planner_config.h>

namespace tesseract_motion_planners
{
/**
 * @brief Default configuration to setup TrajOpt planner.
 *
 * While there are many parameters that can be set, the majority of them have defaults that will be suitable for many
 * problems. These are always required: tesseract_, maninpulator_, link_, tcp_
 *
 */
struct TrajOptPlannerDefaultConfig : public TrajOptPlannerConfig
{
  TrajOptPlannerDefaultConfig(const tesseract::Tesseract::ConstPtr& tesseract_,
                              const std::string& manipulator_,
                              const std::string& link_,
                              const tesseract_common::VectorIsometry3d& tcp_);

  TrajOptPlannerDefaultConfig(const tesseract::Tesseract::ConstPtr& tesseract_,
                              const std::string& manipulator_,
                              const std::string& link_,
                              const Eigen::Isometry3d& tcp_);

  /** @brief Function for creating a ProblemConstructionInfo from the planner configuration */
  virtual std::shared_ptr<trajopt::ProblemConstructionInfo> generatePCI() const;

  /** @brief Generates the TrajOpt problem and saves the result internally */
  virtual bool generate() override;

  /** @brief Tesseract object. ***REQUIRED*** */
  tesseract::Tesseract::ConstPtr tesseract;
  /** @brief Manipulator used for pathplanning ***REQUIRED*** */
  std::string manipulator;
  /** @brief This is the tip link in the kinematics object used for the cartesian positions ***REQUIRED*** */
  std::string link;
  /**
   * @brief Vector of TCP transforms. This should contain either one transform to be applied to all waypoints
   * or a separate transform for each waypoint
   */
  tesseract_common::VectorIsometry3d tcp;
  /** @brief Vector of target waypoints given as constraints to the optimization */

  /** @brief The target tool waypoints */
  std::vector<Waypoint::Ptr> target_waypoints;

  /** @brief Selects the type of initialization used for raster path. If GIVEN_TRAJ, then the seed_trajectory_ must be
   * set */
  trajopt::InitInfo::Type init_type = trajopt::InitInfo::STATIONARY;
  /** @brief The trajectory used as the optimization seed when init_type_ is set to GIVEN_TRAJ */
  trajopt::TrajArray seed_trajectory;
  /** @brief This joint waypoint represents the desired configuration (Optional). This is not guaranteed, but a small
   * equality cost is set to the joint position for all points to pull the optimization in that direction.
   *
   * An example use case is setting it to be at the center of the joint limits. This tends to pull the robot away from
   * singularities */
  JointWaypoint::ConstPtr configuration = nullptr;

  /** @brief If true, collision checking will be enabled. Default: true*/
  bool collision_check = true;
  /** @brief If true, use continuous collision checking */
  bool collision_continuous = true;
  /** @brief Max distance over which collisions are checked */
  double collision_safety_margin = 0.025;
  /** @brief If true, a joint velocity cost with a target of 0 will be applied for all timesteps Default: true*/
  bool smooth_velocities = true;
  /** @brief If true, a joint acceleration cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_accelerations = true;
  /** @brief If true, a joint jerk cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_jerks = true;
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_PLANNER_DEFAULT_CONFIG_H
