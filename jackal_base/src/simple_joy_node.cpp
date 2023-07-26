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
#include "jackal_msgs/Feedback.h"
#include "jackal_msgs/PWMControl.h"
#include "sensor_msgs/Joy.h"
#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <string>
#include <rosserial_server/serial_session.h>
#include "boost/algorithm/clamp.hpp"
#include "realtime_tools/realtime_publisher.h"

namespace jackal_teleop
{

class SimpleJoy
{
public:
  explicit SimpleJoy(ros::NodeHandle* nh);
  void controlThread(ros::Rate rate);

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void feedbackCallback(const jackal_msgs::Feedback::ConstPtr& msg);
  

  ros::NodeHandle* nh_;
  ros::Subscriber joy_sub_;
  ros::Subscriber feedback_sub_;
  ros::Publisher control_pub_;
  realtime_tools::RealtimePublisher<jackal_msgs::Drive> drive_pub_;

  int deadman_button_;
  int axis_linear_;
  int axis_angular_;
  float scale_linear_;
  float scale_angular_;
  int axis_throttle_;
  int axis_turn_;

  bool sent_deadman_msg_;
  bool controller_alive;
  bool feedback_alive;
  int reverse_button_;
  long feedback_msg_cnt;
  bool torque_control_;
  bool invert;
  bool pressed_invert;
  sensor_msgs::Joy::ConstPtr controller;
  jackal_msgs::Feedback::ConstPtr feedback;
  // boost::mutex joy_msg_mutex_;

};

SimpleJoy::SimpleJoy(ros::NodeHandle* nh) : nh_(nh)
{
  joy_sub_ = nh_->subscribe<sensor_msgs::Joy>("/bluetooth_teleop/joy", 1, &SimpleJoy::joyCallback, this);
  feedback_sub_ = nh_->subscribe<jackal_msgs::Feedback>("/feedback", 1, &SimpleJoy::feedbackCallback, this);
  control_pub_ = nh_->advertise<jackal_msgs::PWMControl>("/control_msg", 1, true);
  // drive_pub_ = nh_->advertise<jackal_msgs::Drive>("cmd_drive", 1, true);
  
  
  ros::param::param("/bluetooth_teleop/l1", deadman_button_, 0);
  ros::param::param("/bluetooth_teleop/ly", axis_linear_, 1);
  ros::param::param("/bluetooth_teleop/rx", axis_angular_, 0);
  ros::param::param("/bluetooth_teleop/r2_analog", axis_throttle_, 5);
  ros::param::param("/bluetooth_teleop/r1", reverse_button_, 5);
  ros::param::param("/bluetooth_teleop/lx", axis_turn_, 0);
  ros::param::param("~scale_linear", scale_linear_, 0.5f);
  ros::param::param("~scale_angular", scale_angular_, 0.5f);
  ros::param::param("~do_torque_control", torque_control_, false);
  if(!torque_control_){
    drive_pub_.init(*nh_, "cmd_drive", 1);
  }else{
    drive_pub_.init(*nh_, "torque_setpoint", 1);
  }
  controller_alive = false;
  feedback_alive = false;
  feedback_msg_cnt = 0;
  invert=false;
  pressed_invert = false;
  // joy_msg_mutex_.lock();
  
}

void SimpleJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  controller = joy_msg;
  if(!controller_alive){
    controller_alive = true;
    // joy_msg_mutex_.unlock();
  }
  // boost::mutex::scoped_lock lock(joy_msg_mutex_);
  
}


void SimpleJoy::feedbackCallback(const jackal_msgs::Feedback::ConstPtr& msg){
  if(!feedback_alive){
    feedback_alive = true;
  }
  feedback = msg;
  feedback_msg_cnt++;
}


  // namespace jackal_teleop
  

  void SimpleJoy::controlThread(ros::Rate rate){
    jackal_msgs::PWMControl pwm_control_msg;
    while(1){
      
      if (controller_alive && feedback_alive && drive_pub_.trylock()){
        pwm_control_msg.left_current = feedback->drivers[0].current;
        pwm_control_msg.right_current = feedback->drivers[1].current;

        pwm_control_msg.left_measured_velocity = feedback->drivers[0].measured_velocity;
        pwm_control_msg.right_measured_velocity = feedback->drivers[1].measured_velocity;

        pwm_control_msg.left_measured_travel = feedback->drivers[0].measured_travel;
        pwm_control_msg.right_measured_travel = feedback->drivers[1].measured_travel;
        
        pwm_control_msg.left_motor_temperature = feedback->drivers[0].motor_temperature;
        pwm_control_msg.right_motor_temperature = feedback->drivers[1].motor_temperature;
        pwm_control_msg.commanded_mode = feedback->commanded_mode;
        pwm_control_msg.feedback_header = feedback->header;
        pwm_control_msg.feedback_msg_cnt = feedback_msg_cnt;
        
        if (controller->buttons[deadman_button_]){
          drive_pub_.msg_.mode = jackal_msgs::Drive::MODE_PWM;
          if(!torque_control_){
            float linear = controller->axes[axis_linear_] * scale_linear_;
            float angular = controller->axes[axis_angular_] * scale_angular_;
            float left_pwm = boost::algorithm::clamp(linear - angular, -1.0, 1.0);
            float right_pwm = boost::algorithm::clamp(linear + angular, -1.0, 1.0);

            drive_pub_.msg_.drivers[jackal_msgs::Drive::LEFT] = left_pwm;
            drive_pub_.msg_.drivers[jackal_msgs::Drive::RIGHT] = right_pwm;
            pwm_control_msg.left_pwm = left_pwm;
            pwm_control_msg.right_pwm = right_pwm;
          }else{
            
            if(controller->buttons[reverse_button_] && !pressed_invert){
              invert = !invert;
              pressed_invert = true;
            }else if (!controller->buttons[reverse_button_] && pressed_invert){
              pressed_invert = false;
            }
            float throttle = -(controller->axes[axis_throttle_]-1);
            if(invert){
              throttle *= -1;
            }
            float turn = controller->axes[axis_turn_]*2;
            float left_torque = boost::algorithm::clamp(throttle-turn, -1.0, 1.0);
            float right_torque = boost::algorithm::clamp(throttle+turn, -1.0, 1.0);

            drive_pub_.msg_.drivers[jackal_msgs::Drive::LEFT] = left_torque;
            drive_pub_.msg_.drivers[jackal_msgs::Drive::RIGHT] = right_torque;
            pwm_control_msg.left_pwm = left_torque;
            pwm_control_msg.right_pwm = right_torque;
          }

        }
        else{
          // When deadman button is released, immediately send a single no-motion command
          // in order to stop the robot.
          // if (!sent_deadman_msg_)
          // {
            drive_pub_.msg_.mode = jackal_msgs::Drive::MODE_NONE;
            // sent_deadman_msg_ = true;
          // }
        }
        control_pub_.publish(pwm_control_msg);
        drive_pub_.unlockAndPublish();
      }

    rate.sleep();
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

  boost::thread(boost::bind(&jackal_teleop::SimpleJoy::controlThread, &simple_joy, ros::Rate(50)));
  ros::spin();
}
