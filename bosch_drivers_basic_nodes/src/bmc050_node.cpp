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

#include <bmc050.hpp>
#include <sub20_interface.hpp> 

// ROS message
#include <bmc050_driver/bmc050_measurement.h>

int main( int argc, char **argv )
{
  
  //ROS initialization and publisher/subscriber setup
  ros::init( argc, argv, "bmc050_node" );
  ros::NodeHandle nh;

  std::string hw_id;
  int number_of_bmc050_sensors;
  int publish_rate_Hz;

  // Get parameters from .launch file or parameter server, or take defaults
  nh.param<int>( "/bmc050_node/number_of_bmc050_sensors", number_of_bmc050_sensors, 1 );
  nh.param<int>( "/bmc050_node/publish_rate_Hz", publish_rate_Hz, 50 );
  nh.param<std::string>( "/bmc050_node/hardware_id", hw_id, "0208" );

  Sub20Interface sub20( hw_id );
  std::vector<BMC050*> BMC050_Chain;
  
  // Intantiate the array of BMC050s
  for( int i = 0; i < number_of_bmc050_sensors; i++ )
  {
    // instantiate a sensor and pass in a pointer to it's hw interface:
    BMC050* eCompass = new BMC050( &sub20 ); 
  
    eCompass->setProtocol( SPI ); 
    eCompass->setFrequency( 125000 );
    eCompass->setCompassPin( i*2 ); // SS0 on the sub20
    eCompass->setAccelPin( i*2 + 1 ); // SS1 on the sub20
    // NOTE: in SPI mode, the BMC050 pins PS1 and PS2 connected to GND.

    // tweak accelerometer settings:
    //eCompass->setAccelRange( BMC050Parameters::RANGE_4 ); // adjusts for the sensitivity AND changes the range in the initialization process.
    //eCompass->setAccelBandwidth( BMC050Parameters::BW_63HZ );
    //eCompass->setFilter( true ); // get filtered data upon initializing
    // tweak compass settings:
    //eCompass->setCompassRate( BMC050Parameters::ODR_30HZ );
    //eCompass->setNumRepetitionsXY( 31 );
    //eCompass->setNumRepetitionsZ( 31 );

    // Initialize that BMC050_Chain member:
    if( eCompass->initialize() == false )
    {
      ROS_ERROR( "BMC050 device: %d initialization failed.", i );
      return 1;
    }
    //eCompass->simpleCalibrationAccel();

    // push the bosch_imu instance onto the stack:
    BMC050_Chain.push_back( eCompass );
  }
  
  // instantiate a message:
  bmc050_driver::bmc050_measurement msg;
  ros::Publisher bmc050_pub = nh.advertise<bmc050_driver::bmc050_measurement>("BMC050_data", 20 );

  //set loop rate for measurement polling
  ros::Rate loop_rate_Hz( publish_rate_Hz );
    
  // Run sensor node:
  while( nh.ok() )
  {
    // remove old data on all vectors, so data doesn't accumulate:
    msg.AccelerationX.clear();
    msg.AccelerationY.clear();
    msg.AccelerationZ.clear();
    msg.Temperature.clear();
    msg.MagneticFieldIntensityX.clear();
    msg.MagneticFieldIntensityY.clear();
    msg.MagneticFieldIntensityZ.clear();
    msg.HallResistance.clear();
  
    for( int i = 0; i < number_of_bmc050_sensors; i++ )
    { 
      // take measurement
      if( BMC050_Chain[i]->takeMeasurement() == false )
      {
        ROS_ERROR( "BMC050 device %d failed. Node failed.", i );
        return 1;
      }

      msg.AccelerationX.push_back( BMC050_Chain[i]->AccelX_ );
      msg.AccelerationY.push_back( BMC050_Chain[i]->AccelY_ );
      msg.AccelerationZ.push_back( BMC050_Chain[i]->AccelZ_ );
      msg.Temperature.push_back( BMC050_Chain[i]->Temperature_ );
      msg.MagneticFieldIntensityX.push_back( BMC050_Chain[i]->CompassX_ );
      msg.MagneticFieldIntensityY.push_back( BMC050_Chain[i]->CompassY_ );
      msg.MagneticFieldIntensityZ.push_back( BMC050_Chain[i]->CompassZ_ );
      msg.HallResistance.push_back( BMC050_Chain[i]->RHall_ );
    }
    msg.header.stamp = ros::Time::now();

    bmc050_pub.publish( msg ); 

    ros::spinOnce();
    loop_rate_Hz.sleep();
  }

  ROS_WARN( "Closing BMC050 driver." );
  return 0;
}
