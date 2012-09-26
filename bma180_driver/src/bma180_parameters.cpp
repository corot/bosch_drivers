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

#include "bma180_driver/bma180_parameters.hpp"


/**********************************************************************/
/**********************************************************************/
BMA180Parameters::BMA180Parameters() // Constructor
{
  // set defaults:
  /**
   * \warning flags_ MUST be cleared BEFORE changing any SPI settings!
   */
  flags_ = 0;       
  this->setFrequency( 400000 );   
  this->setProtocol( I2C );
  this->setAccelRange( RANGE_2 ); // default: ±2 [g]
  this->setBandwidth( BW_150 ); // default
  this->setSlaveAddressBit( 0 ); // default: SDO connected to VSS
 
  this->setPin( 10 ); // default: SPI Chip select pin.
  this->setSpiMode( SPI_MODE_3 ); // default: flags adjusted. These settings don't need to be tweaked to read the bma180 via SPI.
  this->setByteOrder( MSB_FIRST ); // default: flags adjusted.
}

/**********************************************************************/
/**********************************************************************/
BMA180Parameters::~BMA180Parameters() // Destructor
{
}


/**********************************************************************/
/**********************************************************************/
bool BMA180Parameters::setProtocol( interface_protocol protocol )
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
    ROS_ERROR( "bma180_parameters:Unsupported protocol." );
    return false;
  }
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMA180Parameters::setFrequency( int frequency )
{
  frequency_ = frequency;
  return true;
}


/**********************************************************************/
/**********************************************************************/
interface_protocol BMA180Parameters::getProtocol()
{
  return protocol_;
}


/**********************************************************************/
/**********************************************************************/
int BMA180Parameters::getFrequency()
{
  return frequency_;
}


/**********************************************************************/
/**********************************************************************/
bool BMA180Parameters::setPin( uint8_t pin )
{
  // adjust pin to desired value:
  pin_ = pin;
  // adjust the current flags to include the pin value:
  flags_ = ( (0x0F)&flags_ ) | (pin_ << 4); // is this correct??

  return true;
}


/**********************************************************************/
/**********************************************************************/
int BMA180Parameters::getPin()
{
  return pin_;
}


/**********************************************************************/
/**********************************************************************/
int* BMA180Parameters::getFlags()
{
  return &flags_;
}


/**********************************************************************/
/**********************************************************************/
bool BMA180Parameters::setSlaveAddressBit( bool choice )
{
  slave_address_bit_ = choice;
  return true;
}


/**********************************************************************/
/**********************************************************************/
uint8_t BMA180Parameters::getSlaveAddressBit()
{
  return slave_address_bit_;
}


/**********************************************************************/
/**********************************************************************/
bool BMA180Parameters::setByteOrder( uint8_t value )
{
  // adjust the flags
  flags_ = ( (0xFB & flags_) | (value << 2) );
  if( value > 1 )
  {
    ROS_ERROR("bma180_parameters: Byte order must be either LSB_FIRST or MSB_FIRST");
    return false;
  }
  return true;
 
}


/**********************************************************************/
/**********************************************************************/
bool BMA180Parameters::setSpiMode( uint8_t mode )
{
  // adjust the flags
  flags_ = ( (0xFC & flags_) | (mode) ); // 111111xx, where xx is the mode.
 
  switch( mode )
  {
  case SPI_MODE_3: {}
    return true;
  case SPI_MODE_0: {}
  case SPI_MODE_1: {}
  case SPI_MODE_2: {}
  default:
    ROS_ERROR( "bma180_parameters: bma180 can only be read in SPI_MODE_3" );
    return false;
  }
}


/**********************************************************************/
/**********************************************************************/
bool BMA180Parameters::setAccelRange( accel_range new_range )
{
  // reassign the class's new range:
  accel_range_ = new_range;
 
  // adjust the sensitivity accordingly:
  switch( new_range )
  {
  case RANGE_1:
    sensitivity_ = 0.00013;
    break;
  case RANGE_1_5:
    sensitivity_ = 0.00019;
    break;
  case RANGE_2:
    sensitivity_ = 0.00025;
    break;
  case RANGE_3:
    sensitivity_ = 0.00038;
    break;
  case RANGE_4:
    sensitivity_ = 0.00050;
    break;
  case RANGE_8:
    sensitivity_ = 0.00099;
    break;
  case RANGE_16:
    sensitivity_ = 0.00198;
    break;
  default: // shouldn't happen because input argument is only an accel_range data type.
    ROS_ERROR( "bma180_parameters: invalid range setting." );
    return false;
  }
  return true;
 
  // NOTE: bma180::changeAccelRange() must be called after this method so that the sensor's range is adjusted as well!
}


/**********************************************************************/
/**********************************************************************/
void BMA180Parameters::setBandwidth( bandwidth bw )
{ 
  bandwidth_ = bw;
}


/**********************************************************************/
/**********************************************************************/
double BMA180Parameters::getSensitivity()
{
  return sensitivity_;
}


/**********************************************************************/
/**********************************************************************/
void BMA180Parameters::setPreCalOffsets( bool choice )
{
  offsetsEnabled_ = choice;
}
