/**
 * @file freespace_srv_example_node.cpp
 * @brief Freespace service example node
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

#include <tesseract_ros_examples/freespace_srv_example.h>

using namespace tesseract_ros_examples;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "freespace_srv_example_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool plotting = true;
  bool rviz = true;
  int steps = 5;
  bool write_to_file = false;
  std::string planning_server_name("jv_planner");
  
  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param("steps", steps, steps);
  pnh.param("write_to_file", write_to_file, write_to_file);
  pnh.param("server_name", planning_server_name, planning_server_name);

  FreespaceSrvExample example(nh, plotting, rviz, steps, write_to_file, planning_server_name);

  if(!example.setup())
    {
      ROS_ERROR("unable to setup FreespaceSrvExample");
      return 0;
    }

  ros::spin();
  return 0;
}
