/**
Software License Agreement (BSD)

\file      joy_pwm_node.cpp
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

#include "ros/ros.h"
#include "jackal_msgs/Drive.h"
#include "sensor_msgs/Joy.h"
#include <ros/console.h>
#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <string>
#include "std_msgs/Int32.h"
#include <rosserial_server/serial_session.h>
#include "boost/algorithm/clamp.hpp"
#include "realtime_tools/realtime_publisher.h"
#include <stdlib.h> 

namespace jackal_teleop
{

class SimpleJoy
{
public:
  explicit SimpleJoy(ros::NodeHandle* nh);
  void controlThread();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  

  ros::NodeHandle* nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher control_pub_;
  ros::Publisher do_control_pub_;
  realtime_tools::RealtimePublisher<jackal_msgs::Drive> drive_pub_;

  int deadman_button_;
  int axis_linear_;
  int axis_angular_;
  float scale_linear_;
  float scale_angular_;
  int axis_throttle_;
  int axis_turn_;
  int energy_based_btn;
  int do_lemniscate_btn;
  int do_circle_btn;
  bool started_energy_based;
  float last_left;
  float last_right;

  bool sent_deadman_msg_;
  bool controller_alive;
  float max_change; // maximum change in angular velocity of each wheel per cycle (rad/s per cycle)
  float max_velocity; // maximum velocity per wheel (rad/s)
  float min_vel; // small velocity to set instead of zero to prevent braking.
  int mode; // 0 = basic pwm, 2 = vel
  sensor_msgs::Joy::ConstPtr controller;

};

SimpleJoy::SimpleJoy(ros::NodeHandle* nh) : nh_(nh)
{
  ros::param::param("/bluetooth_teleop/l1", deadman_button_, 0);
  ros::param::param("/bluetooth_teleop/ly", axis_linear_, 1);
  ros::param::param("/bluetooth_teleop/rx", axis_angular_, 0);

  ros::param::param("/bluetooth_teleop/circle", energy_based_btn, 1);
  ros::param::param("/bluetooth_teleop/square", do_lemniscate_btn, 2);
  ros::param::param("/bluetooth_teleop/triangle", do_circle_btn, 3);

  ros::param::param("/joystick/scale_linear", scale_linear_, 0.5f);
  ros::param::param("/joystick/scale_angular", scale_angular_, 0.5f);
  ros::param::param("/control/max_change", max_change, 0.5f);
  ros::param::param("/control/max_velocity", max_velocity, 20.0f);
  ros::param::param("/control/min_vel", min_vel, 0.1f);
  ros::param::param("~control_mode", mode, 0);

  ROS_INFO("Starting Joystick control with control_mode: %d", mode);
  ROS_INFO("MODES:\n\t0: basic velocity control\n\t1: publish velocity setpoint only (/vel_setpoint topic)\n\t2: velocity control + energy based control");

  if(mode == 0){ 
    // publish commands directly to jackal
    drive_pub_.init(*nh_, "cmd_drive", 1);
  }else{
    // publish to intermediate topic for other nodes to use
    drive_pub_.init(*nh_, "vel_setpoint", 1);
  }
  
  do_control_pub_ = nh_->advertise<std_msgs::Int32>("/do_control", 1, true);
  joy_sub_ = nh_->subscribe<sensor_msgs::Joy>("/bluetooth_teleop/joy", 1, &SimpleJoy::joyCallback, this);
  controller_alive = false;
  started_energy_based = false;
  last_left = 0;
  last_right = 0;
  
}

void SimpleJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  controller = joy_msg;
  if(!controller_alive){
    controller_alive = true;
  }else{
    controlThread();
  }
  
}

  

  void SimpleJoy::controlThread(/*ros::Rate rate*/){

    if (controller_alive && drive_pub_.trylock()){
        
      if (!controller->buttons[deadman_button_]){ // safety button, if not held down, stop the robot
        drive_pub_.msg_.mode = jackal_msgs::Drive::MODE_NONE;
        drive_pub_.msg_.drivers[jackal_msgs::Drive::LEFT] = 0;
        drive_pub_.msg_.drivers[jackal_msgs::Drive::RIGHT] = 0;
        drive_pub_.unlockAndPublish();
        return;
      }

      // do velocity control + energy_based if running
      std_msgs::Int32 do_control_msg;
      do_control_msg.data = 0;
      
      if(controller->buttons[energy_based_btn] && mode == 2){
        do_control_msg.data = 1;
        started_energy_based = true;
      }else if(controller->buttons[do_circle_btn] && mode ==2){
        do_control_msg.data = 2;
        started_energy_based = true;
      }else if(controller->buttons[do_lemniscate_btn] && mode ==2){
        do_control_msg.data = 3;
        started_energy_based = true;
      }else{
        drive_pub_.msg_.mode = jackal_msgs::Drive::MODE_VELOCITY;
        started_energy_based = false;
        do_control_msg.data = 0;
        float throttle = (controller->axes[axis_linear_]) * scale_linear_;
        float turn = controller->axes[axis_angular_] * scale_angular_;
        
        float left_vel = boost::algorithm::clamp(max_velocity*(throttle-turn), -max_velocity, max_velocity);
        float right_vel = boost::algorithm::clamp(max_velocity*(throttle+turn), -max_velocity, max_velocity);
        
        left_vel = boost::algorithm::clamp(left_vel, last_left-max_change, last_left+max_change);
        right_vel = boost::algorithm::clamp(right_vel, last_right-max_change, last_right+max_change);
        
        //prevent setting velocities to zero, this enters "brake mode" on the jackal controller, 
        // set to min val with matching sign
        if(abs(left_vel) < min_vel){
          left_vel = std::copysign(min_vel, left_vel);
        }

        if(abs(right_vel) < min_vel){
          right_vel = std::copysign(min_vel, right_vel); 
        }

        last_left = left_vel;
        last_right = right_vel;
    
        drive_pub_.msg_.drivers[jackal_msgs::Drive::LEFT] = left_vel;
        drive_pub_.msg_.drivers[jackal_msgs::Drive::RIGHT] = right_vel;
      }

      do_control_pub_.publish(do_control_msg); // true for energy based control, false otherwise

      if(started_energy_based){
        // if we are doing energy based control, release the lock but do not publish a drive command
        drive_pub_.unlock(); 
      }else{
        drive_pub_.unlockAndPublish();
      }
    }  
  }
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "jackal_teleop_joy_pwm");
  
  std::string port;
  ros::param::param<std::string>("~port", port, "/dev/jackal");
  boost::asio::io_service io_service;
  new rosserial_server::SerialSession(io_service, port, 115200);
  boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

  ros::NodeHandle nh;
  jackal_teleop::SimpleJoy simple_joy(&nh);

  // boost::thread(boost::bind(&jackal_teleop::SimpleJoy::controlThread, &simple_joy, ros::Rate(50)));
  ros::spin();
}


