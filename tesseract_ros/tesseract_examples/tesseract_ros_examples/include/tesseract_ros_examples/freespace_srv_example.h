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
#include <string>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

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
class FreespaceSrvExample : public Example
{
 public:
  typedef actionlib::SimpleActionServer<control_msgs::JointTrajectoryAction> JointValuesServer;
  
 FreespaceSrvExample(ros::NodeHandle nh, bool plotting, bool rviz, int steps, bool write_to_file, std::string srv_name)
   : Example(plotting, rviz), nh_(nh), steps_(steps), write_to_file_(write_to_file),
    joint_value_server_(nh_, srv_name, boost::bind(&FreespaceSrvExample::jointValueCallBack, this, _1), false)
      {
	if(!nh_.getParam("robot_description", urdf_xml_string_))
	  {
	    ROS_ERROR("FreespaceSrvExample must have robot_description defined on parameter server");
	  }
	if(!nh_.getParam("robot_description_semantic", srdf_xml_string_))
	  {
	    ROS_ERROR("FreespaceSrvExample must have robot_description_semantic defined on parameter server");
	  }
      }
  ~FreespaceSrvExample() = default;
  bool run() override;
  bool setup();
  void jointValueCallBack(const control_msgs::JointTrajectoryGoalConstPtr& goal)
  {
    ROS_ERROR("position, %lf", goal->trajectory.points[0]);
  }

 private:
  ros::NodeHandle nh_;
  int steps_;
  bool write_to_file_;
  JointValuesServer joint_value_server_;
  std::string urdf_xml_string_;
  std::string srdf_xml_string_;
  tesseract_rosutils::ROSPlottingPtr plotter_;

};

}  // namespace tesseract_ros_examples

#endif  // TESSERACT_ROS_EXAMPLES_FREESPACE_SRV_EXAMPLE_H
