/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Ryodo Tanaka.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Ryodo Tanaka
 *********************************************************************/
#include <simple_global_planner/simple_global_planner.hpp>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(simple_global_planner::SimpleGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace simple_global_planner {

  SimpleGlobalPlanner::SimpleGlobalPlanner()
    : costmap_ros_(NULL), initialized_(false){}

  SimpleGlobalPlanner::SimpleGlobalPlanner(string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  
  void SimpleGlobalPlanner::initialize(string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("resolution", resolution_, 50);
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }


  bool SimpleGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                                     const geometry_msgs::PoseStamped& goal,
                                     vector<geometry_msgs::PoseStamped>& plan){

    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    
    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
                costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      plan.clear();
      return false;
    }

    plan.clear();
    costmap_ = costmap_ros_->getCostmap();

    //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    double start_yaw = tf2::getYaw(start.pose.orientation);

    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double goal_yaw = tf2::getYaw(goal.pose.orientation);
    
    double diff_x = goal_x - start_x;
    double diff_y = goal_y - start_y;
    double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

    double target_x = goal_x;
    double target_y = goal_y;
    double target_yaw = goal_yaw;

    geometry_msgs::PoseStamped new_goal = goal;

    for(int i=0; i<=resolution_; i++){
      target_x = start_x + diff_x * (i/static_cast<double>(resolution_));
      target_y = start_y + diff_y * (i/static_cast<double>(resolution_));
      target_yaw = start_yaw + diff_yaw * (i/static_cast<double>(resolution_));

      tf2::Quaternion target_quat;
      target_quat.setRPY(0,0,target_yaw);

      new_goal.pose.position.x = target_x;
      new_goal.pose.position.y = target_y;
      new_goal.pose.orientation.x = target_quat.x();
      new_goal.pose.orientation.y = target_quat.y();
      new_goal.pose.orientation.z = target_quat.z();
      new_goal.pose.orientation.w = target_quat.w();

      plan.push_back(new_goal);
    }
    
    return true;
  }

};
