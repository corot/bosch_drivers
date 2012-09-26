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

// ROS headers
#include <ros/ros.h>
#include <ros/time.h>

#include <bmp085.hpp> 
#include <sub20_interface.hpp>
 
// ROS message
#include <bmp085_driver/bmp085_measurement.h>  


/**
 * This class is an example node that publishes the data from several
 * BMP085s.  In this case, the BMP085s are all connected on the same
 * hardware interface, a sub20.  However, multiple hardware interfaces
 * can be instantiated.  In this way, the user can create a node that 
 * publishes all BMP085 data regardless of the hardware interface that
 * it is connected to.
 **/
int main( int argc, char **argv )
{
  //ROS initialization and publisher/subscriber setup
  ros::init( argc, argv, "BMP085_node" );
  ros::NodeHandle nh;

  std::string hw_id;
  int publish_rate_Hz;


  // Get parameters from .launch file or parameter server, or take defaults
  nh.param<int>( "/bmp085_node/publish_rate_Hz", publish_rate_Hz, 10 );
  nh.param<std::string>( "/bmp085_node/hardware_id", hw_id, "0208" );

  Sub20Interface sub20( hw_id ); 
  BMP085 PressureSensor( &sub20 ); 

  // Apply any sensor settings that deviate from the default settings:
  //PressureSensor.setSamplingMode( BMP085Parameters::ULTRA_HIGH_RESOLUTION );

  // Initialize the Pressure Sensor:
  if( PressureSensor.initialize() == false )
  {
    ROS_ERROR( "BMP085 initialization failed." );
    return 1;
  }

  // Set up ROS message and publisher
  bmp085_driver::bmp085_measurement msg;
  ros::Publisher bmp085_pub = nh.advertise<bmp085_driver::bmp085_measurement>( "BMP085_data", 10 ); // buffer up to 10 messages of data.  
  
  //set loop rate for measurement polling
  ros::Rate loop_rate_Hz( publish_rate_Hz ); 
      
  // Run sensor node:
  while( nh.ok() )
  {
    // stuff message:
    msg.header.stamp = ros::Time::now();

    // take measurement; halt node if false.
    if( PressureSensor.takeMeasurement() == false )
    {
      ROS_ERROR( "BMP085 device failed to takeMeasurement()." );
      return 1;
    }
  
    msg.Temperature = PressureSensor.getTemperature();        
    msg.Pressure = PressureSensor.getPressure();              
    msg.Altitude = PressureSensor.getAltitude();
    bmp085_pub.publish( msg ); 
  
    ros::spinOnce();
    loop_rate_Hz.sleep();
  }
  
  ROS_WARN( "Closing BMP085 driver." );
  return 0;
}
