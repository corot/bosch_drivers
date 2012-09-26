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

#include <ros/ros.h>
#include <ros/time.h>

#include <bma180.hpp>
#include <sub20_interface.hpp> 

using namespace bosch_drivers_common;

int main( int argc, char** argv )
{
  // Create a Bosch Hardware interface
  Sub20Interface Sub20( "0208" );

  // Instantiate a sensor, and pass in the address of its hw interface:
  BMA180 AccelSensor( &Sub20 ); 
  AccelSensor.setProtocol( I2C ); // CSB connected to VDD
  AccelSensor.setFrequency( 400000 );
 
  AccelSensor.setSlaveAddressBit( 0 ); // SDO connected to VSS
 
  AccelSensor.setAccelRange( BMA180Parameters::RANGE_16 );
  AccelSensor.setBandwidth( BMA180Parameters::BW_150 );
  AccelSensor.setPreCalOffsets( true ); // enables precalibrated offsets. Disabled by default.

  // Initialize the Driver (and hardware) with sensor-specific parameters:
  if( AccelSensor.initialize() == false )
  {
    ROS_ERROR("Accel Sensor initialization failed.");
    return 1;
  }

  sleep( 3 ); // to read the register values.

  bool run_continuous_ = true;

  // Initialize ROS's time, since we're not actually creating a node:
  ros::Time::init();
  // create a loop rate:
  ros::Rate wait_interval( 20 ); 
   
  while( run_continuous_)
  {
    AccelSensor.takeMeasurement();
  
    ROS_INFO("Accel_X: %f", AccelSensor.AccelX_);
    ROS_INFO("Accel_Y: %f", AccelSensor.AccelY_);
    ROS_INFO("Accel_Z: %f", AccelSensor.AccelZ_);
    
    wait_interval.sleep();
    
    /*
     * NOTE: if you wish to change the accel_range dynamically (NOT RECOMMENDED ON DATASHEET),
     * you must do two function calls:
     * 1. AccelSensor.setAccelRange( newRange)
     * 2. AccelSensor.changeAccelRange()
     * so that both the class and the sensor interpret the data correctly.
     */
  }
 
  return 0;
}
