/**
 * @file freespace_srv_example.h
 * @brief An example of a robot picking up a box and placing it on a shelf.
 *
 * @author Chris Lewis
 * @date October 15, 2019
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
#ifndef TESSERACT_ROS_EXAMPLES_FREESPACE_SRV_EXAMPLE_H
#define TESSERACT_ROS_EXAMPLES_FREESPACE_SRV_EXAMPLE_H


#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <ros/console.h>
#include <ros/service_client.h>
#include <memory>
#include <string>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/macros.h>
#include <tesseract/tesseract.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/conversions.h>
#include <tesseract_common/types.h>
#include <tesseract_ros_examples/example.h>
#include <tesseract_rosutils/plotting.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/JointTrajectoryAction.h>

namespace tesseract_ros_examples
{
/**
 * @brief An example of a robot picking up a box and placing it on a shelf leveraging
 * tesseract and trajopt to generate the motion trajectory.
 */
class FreespaceSrvExample
{
 public:
  typedef actionlib::SimpleActionServer<control_msgs::JointTrajectoryAction> JointValuesServer;
  
  FreespaceSrvExample(std::string jv_srv_name, int steps);
  ~FreespaceSrvExample() = default;
  tesseract::Tesseract::Ptr tesseract_;     /**< @brief Tesseract Manager Class */

  void jointValueCallBack(const control_msgs::JointTrajectoryGoalConstPtr& goal);

  bool setup();
  
 private:
  ros::NodeHandle nh_;
  JointValuesServer joint_value_server_;
  int steps_;
  std::string urdf_xml_string_;
  std::string srdf_xml_string_;
  std::string manipulator_;
  std::string end_effector_;
  std::unordered_map<std::string, double> desired_joint_states_;
};

}  // namespace tesseract_ros_examples

#endif  // TESSERACT_ROS_EXAMPLES_FREESPACE_SRV_EXAMPLE_H
