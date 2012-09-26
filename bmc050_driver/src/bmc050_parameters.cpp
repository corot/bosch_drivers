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

//\Author Joshua Vasquez and Philip Roan, Robert  Bosch LLC

#include "bmc050_driver/bmc050_parameters.hpp"

BMC050Parameters::BMC050Parameters() :
  accel_sensitivity_( 0.00391 ), // units: [g/LSB] // corresponds to RANGE_2
  repsXY_( 0 ),
  repsZ_( 0 )
{
  setFrequency( 400000 );
  this->setProtocol( I2C );
 
  // clear flags:
  flags_ = 0;
  this->setSpiMode( SPI_MODE_0 );
  this->setByteOrder( MSB_FIRST );
  this->setAccelPin( 0 );
  this->setCompassPin( 1 );
 
  // default Accel settings:
  this->setAccelRange( RANGE_2 );
  this->setAccelBandwidth( BW_1000HZ );
  this->setFilter( true ); // default: filter accel data.

  // default Compass settings:
  this->setCompassRate( ODR_10HZ ); // default: output compass rate is 10 Hz.
 
  // these parameters can be adjusted if other settings are desired:
  this->setCSB2( 0 ); 
  this->setSDO( 0 );
}


BMC050Parameters::~BMC050Parameters() // Destructor
{
}


double BMC050Parameters::getAccelSensitivity()
{
  return accel_sensitivity_;
}


uint8_t BMC050Parameters::getAccelAddress()
{
  // depends on the protocol:
  switch( protocol_ )
  {
  case I2C:
    // depends on BOTH SDO and CSB2 pins
    if ( (CSB2 == 0) && (SDO == 0) )
      return 0x18;
    else if ( (CSB2 == 0) && (SDO == 1) )
      return 0x19;
    else if ( (CSB2 == 1) && (SDO == 0) )
      return 0x18;
    else
      return 0x19;
  case SPI:
    flags_ = ((0x0F)&flags_) | (accel_pin_ << 4);           // spi flags must be adjusted every time we wish to talk to a different sensor on the BMC050
    return accel_pin_;
  default:
    ROS_ERROR("BMC050Parameters::getAccelAddress(): invalid protocol.");
    return 0;
  }
}


uint8_t BMC050Parameters::getCompassAddress()
{  
// depends on the protocol:
  switch( protocol_ )
  {
  case I2C:
    // depends on BOTH SDO and CSB2 pins
    if ( (CSB2 == 0) && (SDO == 0) )
      return 0x10;
    else if ( (CSB2 == 0) && (SDO == 1) )
      return 0x11;
    else if ( (CSB2 == 1) && (SDO == 0) )
      return 0x12;
    else
      return 0x13;
  case SPI:
    flags_ = ((0x0F)&flags_) | (compass_pin_ << 4); // spi flags must be adjusted every time we wish to talk to a different sensor on the BMC050
    return compass_pin_;
  default:
    ROS_ERROR("BMC050Parameters::getCompassAddress(): invalid protocol.");
    return 0;
  }
}


bool BMC050Parameters::setProtocol( interface_protocol protocol )
{
  switch( protocol )
  {
  case I2C:
    protocol_ = protocol;
    break;
  case SPI:
    protocol_ = protocol;
    break;
  default:
    ROS_ERROR("bma180_parameters:Unsupported protocol.");
    return false;
  }
  return true;
}


bool BMC050Parameters::setFrequency( int frequency )
{
  frequency_ = frequency;
  return true;
}


int BMC050Parameters::getFrequency()
{
  return frequency_;
}


interface_protocol BMC050Parameters::getProtocol()
{
  return protocol_;
}


bool BMC050Parameters::setPin( uint8_t pin )
{
  // adjust pin to desired value:
  pin_ = pin;
  // adjust the current flags to include the pin value:
  flags_ = ((0x0F)&flags_) | (pin_ << 4); 

  return true;
}


bool BMC050Parameters::setAccelPin( uint8_t pin )
{
  if( pin < 16 ) // pin value must fit in 4 bits of uint8_t flags
  {
    accel_pin_ = pin;
    return true;
  }
  else
    return false;
}


bool BMC050Parameters::setCompassPin( uint8_t pin )
{
  if( pin < 16 ) // pin value must fit in 4 bits of uint8_t flags
  {
    compass_pin_ = pin;
    return true;
  }
  else
    return false;
}


int BMC050Parameters::getPin() // irrelevant      
{
  return pin_;
}

int* BMC050Parameters::getFlags()
{
  return &flags_;
}


bool BMC050Parameters::setSpiMode( uint8_t mode )
{
  switch( mode )
  {
  case SPI_MODE_3:
    break;
  case SPI_MODE_0:
    break;
  case SPI_MODE_1: {}
  case SPI_MODE_2: {}
  default:
    ROS_ERROR("bma180_parameters: SPI_MODE can only be one of four choices");
    return false;
   
  }
  // adjust the flags
  flags_ = ((0xFC & flags_) | (mode) );
  return true;
}


bool BMC050Parameters::setByteOrder( uint8_t value )
{
  if( value > 1 )
  {
    ROS_ERROR("bma180_parameters: Byte order must be either LSBFIRST or MSBFIRST");
    return false;
  }
  // adjust the flags
  flags_ = ((0xFB & flags_) | (value << 2));
  return true;
}


void BMC050Parameters::setCSB2( bool val )
{
  CSB2 = val;
}


void BMC050Parameters::setSDO( bool val )
{
  SDO = val;
}  


void BMC050Parameters::setAccelRange( accel_range new_range )
{
  accel_range_ = new_range;
}


BMC050Parameters::accel_range BMC050Parameters::getAccelRange()
{
  return accel_range_;
}


void BMC050Parameters::setFilter( bool request )
{
  accel_is_filtered_ = request;
}


void BMC050Parameters::setAccelBandwidth( bandwidth bw )
{ 
  bw_reg_ = bw;
}


void BMC050Parameters::setCompassRate( compass_rate rate )
{
  compass_rate_ = rate;
}


void BMC050Parameters::setNumRepetitionsXY( uint16_t num_reps )
{
  uint16_t actual_reps = num_reps;
 
  // check if even:
  if( num_reps %2 == 0 )
  {
    ROS_WARN("BMC050Parameters::setNumRepetitionsXY(): Only odd numbers are valid. Value converted to an odd number..");
    // make odd:
    actual_reps = num_reps + 1;
  }
 
  // only odd numbers from 1 through 511 are valid
  if( num_reps > 511 )
  {
    actual_reps = 511;
    ROS_WARN("BMC050Parameters::setNumRepetitionsXY(): Repetititions must be less than 512");
  }
  else 
  {
    if( num_reps == 0 )
    {
      actual_reps = 1;
      ROS_WARN("BMC050Parameters::setNumRepetitionsXY(): Repetitions must be greater than zero.");
    }
  } 
  // construct hex code:
  actual_reps = (uint8_t)( (actual_reps - 1) / 2); // see datasheet page 78
 
  // store to class variable:
  repsXY_ = actual_reps;
 
  ROS_INFO("BMC050Parameters::setNumRepetitionsXY(): input: %d  actual: %d", num_reps, actual_reps);
}


void BMC050Parameters::setNumRepetitionsZ( uint16_t num_reps )
{
  uint16_t actual_reps = num_reps;
 
  // only integers from 1 through 256 are valid:
  if( num_reps > 256 )
  {
    actual_reps = 256;
    ROS_WARN("BMC050Parameters::setNumRepetitionsXY(): Repetititions must be less than 256");
  }
  else 
  {
    if( num_reps == 0 )
    {
      actual_reps = 1;
      ROS_WARN("BMC050Parameters::setNumRepetitionsXY(): Repetitions must be greater than zero.");
    }
  } 
  // construct hex code and store to class variable:
  repsZ_ = (uint8_t)(actual_reps - 1);                 // see datasheet pg 78
 
  ROS_INFO("BMC050Parameters::setNumRepetitionsZ(): input: %d  actual: %d", num_reps, actual_reps);
}
