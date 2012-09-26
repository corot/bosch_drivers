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

#include <bmc050.hpp>
#include <arduino_interface.hpp> 

using namespace bosch_drivers_common;

int main( int argc, char** argv )
{
  // Create an Arduino Bosch Hardware interface
  ArduinoInterface Arduino( "/dev/ttyACM1" );
 
  // Instantiate a sensor, and pass pointers to it's hardware interface and its parameters:
  BMC050 DualSensor( &Arduino ); 
 
  DualSensor.setProtocol( I2C ); 
  DualSensor.setFrequency( 400000 );
 
  // the hardware configuration determines the compass and accelerometer i2c addresses:
  DualSensor.setCSB2( 0 );  
  DualSensor.setSDO( 0 );  

  // tweak accelerometer settings:
  DualSensor.setAccelRange( BMC050Parameters::RANGE_8 ); // adjusts for the sensitivity AND changes the range in the initialization process.
  DualSensor.setAccelBandwidth( BMC050Parameters::BW_63HZ );
  DualSensor.setFilter( true ); // get filtered data upon initializing
  DualSensor.setCompassRate( BMC050Parameters::ODR_25HZ );
 
  // Initialize the Driver (and hardware) with sensor-specific parameters:
  bool init_success = DualSensor.initialize(); 

  if( init_success == false )
  {
    ROS_ERROR("Sensor initialization failed.");
    return 1;
  }

  bool run_continuous_ = true;

  // Initialize time, since we're not actually creating a node:
  ros::Time::init();
  ros::Rate wait_interval( 50 ); 
  
  double MagX, MagY, MagZ, AccX, AccY, AccZ, Temp;
  uint16_t RHall;
 
  while( run_continuous_ )
  {
    if( DualSensor.takeMeasurement() == false )
      break;
  
    MagX = DualSensor.CompassX_;
    MagY = DualSensor.CompassY_;
    MagZ = DualSensor.CompassZ_;
    Temp = DualSensor.Temperature_;
    AccX = DualSensor.AccelX_;
    AccY = DualSensor.AccelY_;
    AccZ = DualSensor.AccelZ_;
    RHall = DualSensor.RHall_;
    
    //ROS_INFO("RHall: %d",RHall);
    ROS_INFO("CompassX: %f", MagX);
    ROS_INFO("CompassY: %f", MagY);
    ROS_INFO("CompassZ: %f", MagZ);
    //ROS_INFO("Accel_Temp: %f", Temp);
    ROS_INFO("AccelerometerX: %f", AccX);  
    ROS_INFO("AccelerometerY: %f",AccY); 
    ROS_INFO("AccelerometerZ: %f\r\n\r\n", AccZ);
  
    wait_interval.sleep();
  } 
  ROS_ERROR("BMC050 node exiting.");

  return 0;
}
