/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_twist_joy/teleop_twist_joy.h"
#include <std_msgs/Int8.h>
#include <map>
#include <string>
#include <cstdlib> // for system()
#include <unitree_legged_msgs/HighState.h>


namespace teleop_twist_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);
  void HighStateCallback(const unitree_legged_msgs::HighState::ConstPtr& msg);

  ros::Subscriber joy_sub;
  ros::Subscriber high_state;
  ros::Publisher cmd_vel_pub;
  ros::Publisher mode_12;
  ros::Publisher pub_gaittype;
  std_msgs::Int8 mode12;
  std_msgs::Int8 gaittype;
  int enable_button;
  int enable_turbo_button;
  float bodyHeight;
  int mode;
  std::map<std::string, int> axis_linear_x_map;
  std::map< std::string, std::map<std::string, double> > scale_linear_x_map;
  
  std::map<std::string, int> axis_linear_y_map;
  std::map< std::string, std::map<std::string, double> > scale_linear_y_map;

  std::map<std::string, int> axis_angular_z_map;
  std::map< std::string, std::map<std::string, double> > scale_angular_z_map;

  std::map<std::string, int> axis_angular_y_map;
  std::map< std::string, std::map<std::string, double> > scale_angular_y_map;

  bool sent_disable_msg;
};

/**
 * Constructs TeleopTwistJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;

  pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  pimpl_->mode_12 = nh->advertise<std_msgs::Int8>("mode_12", 1, true);
  pimpl_->pub_gaittype = nh->advertise<std_msgs::Int8>("gaittype", 1, true);
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);
  pimpl_->high_state = nh->subscribe<unitree_legged_msgs::HighState>("high_state", 1, &TeleopTwistJoy::Impl::HighStateCallback,pimpl_);

  nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
  nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

  if (nh_param->getParam("axis_linear_x", pimpl_->axis_linear_x_map))
  {
    nh_param->getParam("scale_linear_x", pimpl_->scale_linear_x_map["normal"]);
    nh_param->getParam("scale_linear_x_turbo", pimpl_->scale_linear_x_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_linear_x", pimpl_->axis_linear_x_map["x"], 1);
    nh_param->param<double>("scale_linear_x", pimpl_->scale_linear_x_map["normal"]["x"], 0.5);
    nh_param->param<double>("scale_linear_x_turbo", pimpl_->scale_linear_x_map["turbo"]["x"], 1.0);
  }

  if (nh_param->getParam("axis_linear_y", pimpl_->axis_linear_y_map))
  {
    nh_param->getParam("scale_linear_y", pimpl_->scale_linear_y_map["normal"]);
    nh_param->getParam("scale_linear_y_turbo", pimpl_->scale_linear_y_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_linear_y", pimpl_->axis_linear_y_map["y"], 0);
    nh_param->param<double>("scale_linear_y", pimpl_->scale_linear_y_map["normal"]["y"], 0.5);
    nh_param->param<double>("scale_linear_y_turbo", pimpl_->scale_linear_y_map["turbo"]["y"], 1.0);
  }

  if (nh_param->getParam("axis_angular_z", pimpl_->axis_angular_z_map))
  {
    nh_param->getParam("scale_angular_z", pimpl_->scale_angular_z_map["normal"]);
    nh_param->getParam("scale_angular_z_turbo", pimpl_->scale_angular_z_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_angular_z", pimpl_->axis_angular_z_map["yaw"], 3);
    nh_param->param<double>("scale_angular_z", pimpl_->scale_angular_z_map["normal"]["yaw"], 0.5);
    nh_param->param<double>("scale_angular_z_turbo",
        pimpl_->scale_angular_z_map["turbo"]["yaw"], pimpl_->scale_angular_z_map["normal"]["yaw"]);
  }

  if (nh_param->getParam("axis_angular_y", pimpl_->axis_angular_y_map))
  {
    nh_param->getParam("scale_angular_y", pimpl_->scale_angular_y_map["normal"]);
    nh_param->getParam("scale_angular_y_turbo", pimpl_->scale_angular_y_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_angular_y", pimpl_->axis_angular_y_map["pitch"], 4);
    nh_param->param<double>("scale_angular_y", pimpl_->scale_angular_y_map["normal"]["pitch"], 0.5);
    nh_param->param<double>("scale_angular_y_turbo",
        pimpl_->scale_angular_y_map["turbo"]["pitch"], pimpl_->scale_angular_y_map["normal"]["pitch"]);
  }

  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop enable button %i.", pimpl_->enable_button);
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
      "Turbo on button %i.", pimpl_->enable_turbo_button);

  for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_x_map.begin();
      it != pimpl_->axis_linear_x_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear_x axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_linear_x_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for linear_x axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_x_map["turbo"][it->first]);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_y_map.begin();
      it != pimpl_->axis_linear_y_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear_y axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_linear_y_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for linear_y axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_y_map["turbo"][it->first]);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_angular_z_map.begin();
      it != pimpl_->axis_angular_z_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular_z axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_angular_z_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for angular_z axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_z_map["turbo"][it->first]);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_angular_y_map.begin();
      it != pimpl_->axis_angular_y_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular_y axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_angular_y_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for angular_y axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_y_map["turbo"][it->first]);
  }

  pimpl_->sent_disable_msg = false;
}

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map,
              const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  geometry_msgs::Twist cmd_vel_msg;

  cmd_vel_msg.linear.x = getVal(joy_msg, axis_linear_x_map, scale_linear_x_map[which_map], "x");
  cmd_vel_msg.linear.y = getVal(joy_msg, axis_linear_y_map, scale_linear_y_map[which_map], "y");
  cmd_vel_msg.linear.z = getVal(joy_msg, axis_linear_x_map, scale_linear_x_map[which_map], "z");
  cmd_vel_msg.angular.z = getVal(joy_msg, axis_angular_z_map, scale_angular_z_map[which_map], "yaw");
  cmd_vel_msg.angular.y = getVal(joy_msg, axis_angular_y_map, scale_angular_y_map[which_map], "pitch");
  cmd_vel_msg.angular.x = getVal(joy_msg, axis_angular_z_map, scale_angular_z_map[which_map], "roll");

  cmd_vel_pub.publish(cmd_vel_msg);
  sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::HighStateCallback(const unitree_legged_msgs::HighState::ConstPtr& msg)
{
  bodyHeight = msg->bodyHeight;
  mode = msg->mode;
}


void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if (enable_turbo_button >= 0 &&
      joy_msg->buttons.size() > enable_turbo_button &&
      joy_msg->buttons[enable_turbo_button])
  {
    sendCmdVelMsg(joy_msg, "turbo");
  }
  else if (joy_msg->buttons.size() > enable_button &&
           joy_msg->buttons[enable_button])
  {
    sendCmdVelMsg(joy_msg, "normal");
  }
  else if (joy_msg->axes.size() > 2 && joy_msg->axes[2] == -1)
  {
    if (joy_msg->buttons.size() > 0 && joy_msg->buttons[0])
    {
      //stand up and down
      if (bodyHeight > 0.15)
      {
        system("rosrun unitree_legged_real stand_down");
      }
      else if (mode == 5)
      {
        system("rosrun unitree_legged_real stand_up"); 
      }
      else if (mode == 7)
      {
        system("rosrun unitree_legged_real stand_down");
      }
    }
    else if (joy_msg->buttons.size() > 1 && joy_msg->buttons[1])
    {
      //damping
      if (mode == 5)
      {
        system("rosrun unitree_legged_real damping"); 
      }
    }
    else if (joy_msg->buttons.size() > 2 && joy_msg->buttons[2])
    {
      //recovery stand
      system("rosrun unitree_legged_real recovery_stand"); 
    }
  }
  else if (joy_msg->axes.size() > 5 && joy_msg->axes[5] == -1)
  {
    if (joy_msg->buttons.size() > 0 && joy_msg->buttons[0])
    {
      //praying
      if (bodyHeight > 0.25 && mode != 5 && mode!=7)
      {
        system("rosrun unitree_legged_real praying");
      }
    }
    else if (joy_msg->buttons.size() > 1 && joy_msg->buttons[1])
    {
      //dance1
      if (bodyHeight > 0.25 && mode != 5 && mode !=7)
      {
        system("rosrun unitree_legged_real dance1"); 
      }
    }
    else if (joy_msg->buttons.size() > 2 && joy_msg->buttons[2])
    {
      //dance2
      if (bodyHeight > 0.25 && mode != 5 && mode !=7)
      {
        system("rosrun unitree_legged_real dance2"); 
      }
    }
    else if (joy_msg->buttons.size() > 3 && joy_msg->buttons[3])
    {
      //dance2
      if (bodyHeight > 0.25 && mode != 5 && mode !=7)
      {
        system("rosrun unitree_legged_real jump_and_yaw90"); 
      }
    }    
  }   
  else if (joy_msg->buttons.size() > 6 &&
           joy_msg->buttons[6])
  {
    mode12.data = 1;
    mode_12.publish(mode12);
  }
  else if (joy_msg->buttons.size() > 7 &&
           joy_msg->buttons[7])
  {
    mode12.data = 2;
    mode_12.publish(mode12);
  }
  else if (joy_msg->buttons.size() > 0 && joy_msg->buttons[0] )
  {
    gaittype.data = 1;
    pub_gaittype.publish(gaittype);
  }
  else if (joy_msg->buttons.size() > 2 && joy_msg->buttons[2] ) 
  {
    gaittype.data = 2;
    pub_gaittype.publish(gaittype);
  }
  else if (joy_msg->buttons.size() > 3 && joy_msg->buttons[3] )
  {
    gaittype.data = 3;
    pub_gaittype.publish(gaittype);
  }
  else if (joy_msg->buttons.size() > 1 && joy_msg->buttons[1] )
  {
    gaittype.data = 4;
    pub_gaittype.publish(gaittype);
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      // Initializes with zeros by default.
      geometry_msgs::Twist cmd_vel_msg;
      cmd_vel_pub.publish(cmd_vel_msg);
      sent_disable_msg = true;
    }
  }
}

}  // namespace teleop_twist_joy
