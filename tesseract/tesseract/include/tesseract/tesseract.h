/**
 * @file tesseract.h
 * @brief This is a container class for all tesseract packages. Provides
 * methods to simplify construction of commonly used features.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_TESSERACT_H
#define TESSERACT_TESSERACT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_environment/core/environment.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <tesseract_scene_graph/utils.h>
#include <boost/filesystem/path.hpp>
#include <tesseract/forward_kinematics_manager.h>
#include <tesseract/inverse_kinematics_manager.h>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract
{
/**
 * @brief The Tesseract class
 *
 * This is a container class which hold objects needed for motion planning.
 * It also provides several construction methods for loading from urdf, srdf
 *
 */
class Tesseract
{
public:
  using Ptr = std::shared_ptr<Tesseract>;
  using ConstPtr = std::shared_ptr<const Tesseract>;

  Tesseract();
  virtual ~Tesseract() = default;

  bool isInitialized() const;

  bool init(tesseract_scene_graph::SceneGraph::Ptr scene_graph);
  bool init(tesseract_scene_graph::SceneGraph::Ptr scene_graph, tesseract_scene_graph::SRDFModel::ConstPtr srdf_model);
  bool init(const std::string& urdf_string, tesseract_scene_graph::ResourceLocatorFn locator);
  bool init(const std::string& urdf_string,
            const std::string& srdf_string,
            tesseract_scene_graph::ResourceLocatorFn locator);
  bool init(const boost::filesystem::path& urdf_path, tesseract_scene_graph::ResourceLocatorFn locator);
  bool init(const boost::filesystem::path& urdf_path,
            const boost::filesystem::path& srdf_path,
            tesseract_scene_graph::ResourceLocatorFn locator);

  const tesseract_scene_graph::SRDFModel::ConstPtr& getSRDFModel() const;

  const tesseract_environment::Environment::Ptr& getEnvironment();
  const tesseract_environment::Environment::ConstPtr& getEnvironmentConst() const;

  const ForwardKinematicsManager::Ptr& getFwdKinematicsManager();
  const ForwardKinematicsManager::ConstPtr& getFwdKinematicsManagerConst() const;

  const InverseKinematicsManager::Ptr& getInvKinematicsManager();
  const InverseKinematicsManager::ConstPtr& getInvKinematicsManagerConst() const;

private:
  bool initialized_;
  tesseract_environment::Environment::Ptr environment_;
  tesseract_environment::Environment::ConstPtr environment_const_;
  tesseract_scene_graph::SRDFModel::ConstPtr srdf_model_;
  ForwardKinematicsManager::Ptr fwd_kin_manager_;
  ForwardKinematicsManager::ConstPtr fwd_kin_manager_const_;
  InverseKinematicsManager::Ptr inv_kin_manager_;
  InverseKinematicsManager::ConstPtr inv_kin_manager_const_;

  bool registerDefaultContactManagers();
  bool registerDefaultInvKinSolvers();
  bool registerDefaultFwdKinSolvers();

  void clear();
};

}  // namespace tesseract
#endif  // TESSERACT_TESSERACT_H
