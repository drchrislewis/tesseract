/**
 * @file continuous_motion_validator.cpp
 * @brief Tesseract OMPL planner continuous collision check between two states
 *
 * @author Jonathan Meyer
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <thread>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>

namespace tesseract_motion_planners
{
ContinuousMotionValidator::ContinuousMotionValidator(ompl::base::SpaceInformationPtr space_info,
                                                     tesseract_environment::Environment::ConstPtr env,
                                                     tesseract_kinematics::ForwardKinematics::ConstPtr kin)
  : MotionValidator(space_info), env_(std::move(env)), kin_(std::move(kin))
{
  joints_ = kin_->getJointNames();

  // kinematics objects does not know of every link affected by its motion so must compute adjacency map
  // to determine all active links.
  tesseract_environment::AdjacencyMap adj_map(
      env_->getSceneGraph(), kin_->getActiveLinkNames(), env_->getCurrentState()->transforms);
  links_ = adj_map.getActiveLinkNames();

  continuous_contact_manager_ = env_->getContinuousContactManager();
  continuous_contact_manager_->setActiveCollisionObjects(links_);
  continuous_contact_manager_->setContactDistanceThreshold(0);

  discrete_contact_manager_ = env_->getDiscreteContactManager();
  discrete_contact_manager_->setActiveCollisionObjects(links_);
  discrete_contact_manager_->setContactDistanceThreshold(0);
}

bool ContinuousMotionValidator::checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const
{
  std::pair<ompl::base::State*, double> dummy = { nullptr, 0.0 };
  return checkMotion(s1, s2, dummy);
}

bool ContinuousMotionValidator::checkMotion(const ompl::base::State* s1,
                                            const ompl::base::State* s2,
                                            std::pair<ompl::base::State*, double>& lastValid) const
{
  const ompl::base::StateSpace& state_space = *si_->getStateSpace();

  unsigned n_steps = state_space.validSegmentCount(s1, s2);

  ompl::base::State* start_interp = si_->allocState();
  ompl::base::State* end_interp = si_->allocState();

  bool is_valid = true;
  unsigned i = 1;
  for (i = 1; i <= n_steps; ++i)
  {
    state_space.interpolate(s1, s2, static_cast<double>(i - 1) / n_steps, start_interp);
    state_space.interpolate(s1, s2, static_cast<double>(i) / n_steps, end_interp);

    // Currently we must perform both continuous and discrete because continuous does not perform
    // self collision checking.
    if (!si_->isValid(end_interp) || !continuousCollisionCheck(start_interp, end_interp) ||
        !discreteCollisionCheck(end_interp))
    {
      is_valid = false;
      break;
    }
  }

  if (!is_valid)
  {
    lastValid.second = static_cast<double>(i - 1) / n_steps;
    if (lastValid.first != nullptr)
      state_space.interpolate(s1, s2, lastValid.second, lastValid.first);
  }

  si_->freeState(start_interp);
  si_->freeState(end_interp);
  return is_valid;
}

bool ContinuousMotionValidator::continuousCollisionCheck(const ompl::base::State* s1, const ompl::base::State* s2) const
{
  const ompl::base::RealVectorStateSpace::StateType* start = s1->as<ompl::base::RealVectorStateSpace::StateType>();
  const ompl::base::RealVectorStateSpace::StateType* finish = s2->as<ompl::base::RealVectorStateSpace::StateType>();

  // It was time using chronos time elapsed and it was faster to cache the contact manager
  unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
  tesseract_collision::ContinuousContactManager::Ptr cm;
  mutex_.lock();
  auto it = continuous_contact_managers_.find(hash);
  if (it == continuous_contact_managers_.end())
  {
    cm = continuous_contact_manager_->clone();
    continuous_contact_managers_[hash] = cm;
  }
  else
  {
    cm = it->second;
  }
  mutex_.unlock();

  const auto dof = si_->getStateDimension();
  Eigen::Map<Eigen::VectorXd> start_joints(start->values, dof);
  Eigen::Map<Eigen::VectorXd> finish_joints(finish->values, dof);

  tesseract_environment::EnvState::Ptr state0 = env_->getState(joints_, start_joints);
  tesseract_environment::EnvState::Ptr state1 = env_->getState(joints_, finish_joints);

  for (const auto& link_name : links_)
    cm->setCollisionObjectsTransform(link_name, state0->transforms[link_name], state1->transforms[link_name]);

  tesseract_collision::ContactResultMap contact_map;
  cm->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

  return contact_map.empty();
}

bool ContinuousMotionValidator::discreteCollisionCheck(const ompl::base::State* s2) const
{
  const ompl::base::RealVectorStateSpace::StateType* finish = s2->as<ompl::base::RealVectorStateSpace::StateType>();

  // It was time using chronos time elapsed and it was faster to cache the contact manager
  unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
  tesseract_collision::DiscreteContactManager::Ptr cm;
  mutex_.lock();
  auto it = discrete_contact_managers_.find(hash);
  if (it == discrete_contact_managers_.end())
  {
    cm = discrete_contact_manager_->clone();
    discrete_contact_managers_[hash] = cm;
  }
  else
  {
    cm = it->second;
  }
  mutex_.unlock();

  const auto dof = si_->getStateDimension();
  Eigen::Map<Eigen::VectorXd> finish_joints(finish->values, dof);

  tesseract_environment::EnvState::Ptr state1 = env_->getState(joints_, finish_joints);

  for (const auto& link_name : links_)
    cm->setCollisionObjectsTransform(link_name, state1->transforms[link_name]);

  tesseract_collision::ContactResultMap contact_map;
  cm->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

  return contact_map.empty();
}

}  // namespace tesseract_motion_planners
