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

// ROS header for debugging output
#include <ros/console.h>

#include "bmp085_driver/bmp085.hpp"
#include "bmp085_driver/bmp085_parameters.hpp"


/**********************************************************************/
// Constructor
/**********************************************************************/
BMP085::BMP085( bosch_hardware_interface* hw ) :
  sensor_driver( hw ),
  BMP085Parameters()
{
  // set default pressure at sea level:
  pressure_at_sea_level_ = 101.325; // in [kPa]
}


/**********************************************************************/
// Destructor
/**********************************************************************/
BMP085::~BMP085() 
{
}


/**********************************************************************/
// Initialization
/**********************************************************************/
bool BMP085::initialize()
{  
  // Only I2C is supported for BMP085.  
  if( this->getProtocol() != I2C )
  {
    ROS_ERROR("BMP085::initialize(): You cannot read the BMP085 with this protocol");
    return false;
  }

  // Initialize the hardware interface with the selected parameters.
  if( hardware_->initialize() == false )
  {
    ROS_ERROR("BMP085::initialize(): Could not initialize a hardware interface!");
    return false;
  }
  
  // set oversampling setting:
  oss = this->getSamplingMode(); 
  ROS_INFO("Pressure Sensor Sampling Mode (oss): %d", oss);

  // Get all Calibration Constants.  
  // Luckily, they're all consecutively stored in memory, so we can just
  // read them all at once into one array using a multi-byte read.   
  uint8_t FullCalData[22];
   
  // Read 22 bytes of Calibration-constant data
 
  if( hardware_->read( this->getDeviceAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), (uint8_t)ADDRESS_AC1_MSB, FullCalData, 22 ) < 0 )
  {
    ROS_ERROR("BMP085::initialize(): Invalid Calibration data");
    return false;
  }
 
  // Compute calibration constants:
  AC1 = ((uint16_t)FullCalData[0]  <<8) | (uint16_t)FullCalData[1];
  AC2 = ((uint16_t)FullCalData[2]  <<8) | (uint16_t)FullCalData[3];
  AC3 = ((uint16_t)FullCalData[4]  <<8) | (uint16_t)FullCalData[5];
  AC4 = ((uint16_t)FullCalData[6]  <<8) | (uint16_t)FullCalData[7];
  AC5 = ((uint16_t)FullCalData[8]  <<8) | (uint16_t)FullCalData[9];
  AC6 = ((uint16_t)FullCalData[10] <<8) | (uint16_t)FullCalData[11];
 
  B1 =  ((uint16_t)FullCalData[12] <<8) | (uint16_t)FullCalData[13];
  B2 =  ((uint16_t)FullCalData[14] <<8) | (uint16_t)FullCalData[15];

  MB =  ((uint16_t)FullCalData[16] <<8) | (uint16_t)FullCalData[17];
  MC =  ((uint16_t)FullCalData[18] <<8) | (uint16_t)FullCalData[19];
  MD =  ((uint16_t)FullCalData[20] <<8) | (uint16_t)FullCalData[21];

  //ROS_INFO("%d",AC1); // for debugging
  return true;
}


/**********************************************************************/
// measure both Temperature and Pressure
/**********************************************************************/
bool BMP085::takeMeasurement()
{
  // pressure measurement depends on temperature, so get temperature first!
  //   Specifically, the B5 constant in the pressure calculation comes
  // from the temp calculation.
  if( this->getTemperatureData() == false )
    return false;
  if( this->getPressureData() == false )
    return false;

  return true;
}


/**********************************************************************/
/**********************************************************************/
uint8_t BMP085::getDeviceAddress()
{
  // I2C is the only way the sensor can be read.  
  // Ensure I2C has been set:
  switch( this->getProtocol() )
  {
  case I2C:
    return DEVICE_ADDRESS;
    break;
  default:
    ROS_ERROR("BMP085::getDeviceAddress(): Protocol not set in parameters. Please set a protocol, and recompile.");
    return DEVICE_ADDRESS;
  } 
}


/**********************************************************************/
/**********************************************************************/
bool BMP085::getTemperatureData( void )  
{
  /* Note: lots of casting! The Temperature is a uint16_t. 
   *   However, we need to access it's 8-bit MSB chunk 
   *   and 8-bit LSB chunk separately.  
   *   Then, we combine them into a uint16_t.
   */
  uint8_t MSB, LSB;
  
  if( writeToReg( INPUT_REG_ADDRESS, ADDRESS_TEMP_REQUEST ) == false )
  {
    ROS_ERROR("BMP085::getTemperatureData(): INVALID DATA");
    return false;
  }
 
  usleep( 4500 ); // sleep for 4.5 [ms] while BMP085 processes temperature.
  
  if( hardware_->read( this->getDeviceAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), MEAS_OUTPUT_MSB, &MSB, 1 ) < 0 )
  {
    ROS_ERROR("BMP085::getTemperatureData(): Invalid data"); 
    return false;
  }
  if( hardware_->read( this->getDeviceAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), MEAS_OUTPUT_LSB, &LSB, 1 ) < 0 )
  {
    ROS_ERROR("BMP085::getTemperatureData(): Invalid data"); 
    return false;
  }
 
  UT = ( (uint16_t)MSB << 8 ) + LSB;

  long X1, X2; 
  X1 = ( (UT - AC6) * AC5) >> 15; // Note: X>>15 == X/(pow(2,15))
  X2 = (MC << 11) / (X1 + MD); // Note: X<<11 == X<<(pow(2,11))
  // B5 is part of the class.
  B5 = X1 + X2;
  T = (B5 + 8) >> 4;

  temperature_ = (double)T * 0.1; 
 
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMP085::getPressureData()
{
  /* Note: The Pressure has up to 19 bits of data.
   * We need to combine three 8-bit numbers and shift them accordingly.
   */  

  uint8_t PressureRequest = PRESSURE_OSRS_0 + (oss<<6);
  
  if( writeToReg( INPUT_REG_ADDRESS, PressureRequest ) == false )
  {
    ROS_ERROR("BMP085::getPressureData():INVALID DATA");
    return false;
  }
 
  // Wait for the chip's pressure measurement to finish processing 
  // before accessing the data.  Wait time depends on the sampling mode.
  switch( oss )
  {
  case 0:
    usleep( 4500 ); 
    break;
  case 1:
    usleep( 7500 );
    break;
  case 2:
    usleep( 13500 );
    break;
  case 3:
    usleep( 25500 );
    break;
  default:
    usleep( 25500 );
    break;
  }

  unsigned char data[3]; 
  
  if( hardware_->read( this->getDeviceAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), MEAS_OUTPUT_MSB, data, 3 ) < 0 )
  {
    ROS_ERROR("BMP085::getPressureData():Invalid data");
    return false;
  }

  UP = (long)( ( ( (ulong)data[0]<<16) | ((ulong)data[1]<<8) | (ulong)data[2] ) >> (8-oss));
  // ROS_INFO("UP: %dl",UP);  // sanity check.
  long B3, B6, X1, X2, X3;  // coefficients derived from existing coefficients. 
  // Note: we calculated B5 from the temperature!
  ulong B4, B7;
  
  B6 = B5 - 4000;
  X1 = ( (B2 * ((B6 * B6)>>12)) >> 11 );
  X2 = ( ((long)AC2* B6) >> 11);
  X3 = X1 + X2;
  B3 = ( ( ( ((long)AC1*4) + X3)<< oss) + 2) >> 2;
  
  X1 = (((long)AC3*B6) >> 13);
  X2 = ( B1*((B6*B6) >> 12) ) >> 16;
  X3 = ( (X1 + X2) + 2 ) >> 2;
  B4 = ((long)AC4* (ulong)(X3 + 32768)) >> 15;
  
  B7 =  (ulong)(UP - B3)*(50000>>oss);
  if( B7 < 0x80000000 )
  {
    p = (B7 << 1)/B4;
  }
  else
  {
    p = (B7/B4) << 1;
  }
  X1 = (p >> 8)*(p >> 8);
  X1 = (X1*3038) >> 16;
  X2 = ((-7357)*p)>>16;
  p = p + ( ( X1 + X2 + 3791 ) >> 4);

  pressure_ = p / 1000.0; // in [kPa]
  //ROS_INFO("pressure: %f",pressure_);
  return true;
}

/**********************************************************************/
/**********************************************************************/
double BMP085::calcAltitude( double static_pressure )
{
  // fancy math: barometric equation
  altitude_ = 44330 * (1 - pow( (static_pressure / pressure_at_sea_level_), (1/5.255) ));
  return altitude_;
}


/**********************************************************************/
// returns the altitude based on the most recent pressure measurement
/**********************************************************************/
double BMP085::getAltitude()
{
  return calcAltitude( pressure_ );
}


/**********************************************************************/
/**********************************************************************/
double BMP085::getTemperature()
{
  return temperature_;
}


/**********************************************************************/
/**********************************************************************/
double BMP085::getPressure()
{
  return pressure_;
}


/**********************************************************************/
/**********************************************************************/
void BMP085::setPressureAtSeaLevel( double pressure )
{
  pressure_at_sea_level_ = pressure;
}


/**********************************************************************/
// write a byte to a register
/**********************************************************************/
bool BMP085::writeToReg( uint8_t reg, uint8_t value )
{
  if( hardware_->write( this->getDeviceAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), (uint8_t)reg, &value, 1 ) < 0 )
  {
    ROS_ERROR("could not write value to register."); 
    return false;
  }
  return true;
}
