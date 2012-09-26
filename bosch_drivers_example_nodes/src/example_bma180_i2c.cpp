/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Robert Bosch LLC.
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
 *   * Neither the name of the Robert Bosch nor the names of its
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
 *********************************************************************/

//\Author Joshua Vasquez and Philip Roan, Robert Bosch LLC

// example_bma180_i2c.cpp

#include <ros/ros.h>
#include <ros/time.h>

#include <bma180.hpp>
#include <arduino_interface.hpp> 


// NODE PARAMETERS:
static const int PUBLISH_RATE = 50;  

using namespace bosch_drivers_common;

int main( int argc, char** argv )
{
  // Create an Arduino Bosch Hardware interface:
  ArduinoInterface Arduino( "/dev/ttyACM1" );
 
  // Instantiate a sensor, and pass pointers to it's hardware interface and its parameters:
  BMA180 AccelSensor( &Arduino ); 

  //AccelSensor.setSlaveAddressBit(1);// SDO connected to VDD.  (GND)
 
  //AccelSensor.setAccelRange(BMA180Parameters::RANGE_4);
  //AccelSensor.setBandwidth(BMA180Parameters::BW_150);
  //AccelSensor.setPreCalOffsets(false); 

  // Initialize the Driver (and hardware) with sensor-specific parameters:
  bool init_success = AccelSensor.initialize(); 

  if( init_success == false )
  {
    ROS_ERROR("Accel Sensor initialization failed.");
    return 1;
  }

  bool run_continuous_ = true;

  // Initialize time, since we're not actually creating a node:
  ros::Time::init();
  ros::Rate wait_interval( PUBLISH_RATE ); 
  
  double pitch, roll;
 
  while( run_continuous_)
  {
    AccelSensor.takeMeasurement();
    pitch = AccelSensor.getStaticPitch();
    roll = AccelSensor.getStaticRoll();
    pitch *= 180.0 / M_PI;
    roll *= 180.0 / M_PI;

    ROS_INFO("Pitch : %f ", pitch);
    ROS_INFO("roll  : %f ", roll);
    
    wait_interval.sleep();
  }
 
  return 0;
}
