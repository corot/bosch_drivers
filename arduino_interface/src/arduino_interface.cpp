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

#include "arduino_interface/arduino_interface.hpp"

/**********************************************************************/
// Constructor
/**********************************************************************/
ArduinoInterface::ArduinoInterface( std::string port_name ) :
  port_name_( port_name ),
  baud_rate_( DEFAULT_BAUD_RATE ),
  connection_failure_( true ), // no connection established yet.
  timeout_( 0.5 ), // delay in seconds before we declare Serial communication lost.
  is_initialized_( false ),
  data_packet_( 0 )
{
  // Create a serial port and assign it the class name: 
  serial_port_ = new uniserial();
}


/**********************************************************************/
// Destructor
/**********************************************************************/
ArduinoInterface::~ArduinoInterface()
{
  delete serial_port_;
}


/**********************************************************************/
// Initialize:   
/**********************************************************************/
bool ArduinoInterface::initialize()
{
  // Prevent Double-initialization!
  if( is_initialized_ == true )
  {
    ROS_INFO( "ArduinoInterface::initialize(): hardware already initialized." );
    return true;
  }
 
  // uniserial init args: [8-bit data], [no parity], [1 stop bit], [class baud rate]. 
  bool init_success = serial_port_->initialize( port_name_, 8 , 0, 1, baud_rate_ );
 
  // At least a 3-second delay while the Arduino resets. Note: We can remove this 3 second startup if we cut the solder bridge on the board's RESET EN trace. 
  sleep( 3 );
 
  // Error if connection failure:
  if( init_success != true )
  {
    ROS_ERROR( "Arduino Serial connection not initialized properly.  Make sure serial_port and baud_rate match on both ends!" );
    connection_failure_ = true;
    return false;
  }
 
  // successful Serial connection!
  connection_failure_ = false; 
  is_initialized_ = true;
  ROS_INFO( "ArduinoInterface::initialize(): hardware initialized." );
  return true;
}


/**********************************************************************/
// Read
/**********************************************************************/
ssize_t ArduinoInterface::read( int device_address, interface_protocol protocol, int frequency, int* flags, uint8_t reg_address, uint8_t* data, size_t num_bytes )
{
  int error_code = 0;
  
  // Check connection:
  if( connection_failure_ == true )
  {
    return -1;
  }
  
  // Create data packet for reading: (depends on protocol)
  data_packet_ = 0;
 
  switch( protocol )
  {
  case I2C:
  {
    error_code = arduinoI2cRead( device_address, frequency, reg_address, data, num_bytes );               
    break;
  }
  case SPI: 
  {
    error_code = arduinoSpiRead( frequency, flags, reg_address, data, num_bytes );
    break;
  } 
  default:
    ROS_ERROR("Arduino does not support reading through this protocol.");
    return -1;
  }
  return error_code;
}


/**********************************************************************/
// Write
/**********************************************************************/
ssize_t ArduinoInterface::write( int device_address, interface_protocol protocol, int frequency, int* flags, uint8_t reg_address, uint8_t* data, size_t num_bytes )
{ 
  int error_code = 0;

  // Check connection:
  if( connection_failure_ == true )
  {
    return -1;
  }
  
  // Create data packet for writing: (depends on protocol)
  data_packet_ = 1;
 
  switch( protocol )
  {
  case I2C:
    error_code = arduinoI2cWrite( device_address, frequency, reg_address, data, num_bytes );
    break;
  case SPI:
  {
    error_code = arduinoSpiWrite( frequency, flags, reg_address, data, num_bytes );
    break;
  }
  default:
    ROS_ERROR( "Arduino does not support writing through this protocol." );
    error_code = -1;
  }
 
  return error_code; // bytes written, or error. 
}


/**********************************************************************/
// supportedProtocol
/**********************************************************************/
bool ArduinoInterface::supportedProtocol( interface_protocol protocol )
{
  switch( protocol )
  {
  case SPI: 
    return true;
  case I2C: 
    return true;
  case GPIO: {}
  case RS232: {}
  case RS485: {}
  case ETHERNET: {}
  case ETHERCAT: {}
  default:
    return false;
  }
}


/**********************************************************************/
std::string ArduinoInterface::getID()
/**********************************************************************/
{
  return port_name_;
}


/**********************************************************************/
/**********************************************************************/
bool ArduinoInterface::waitOnBytes( int num_bytes )
{
  // Error checking variables 
  unsigned int sleep_interval_us = 25;
  unsigned int loop_counter = 0;

  while( serial_port_->Available() < num_bytes ) // wait for Arduino's response.
  {
    double time_lapsed = (double)(loop_counter * sleep_interval_us ) / 1e6; // [s]
    if( time_lapsed > timeout_ )
    {
      ROS_ERROR( "Serial communication timed out waiting for: %d byte(s).", num_bytes );
      connection_failure_ = true;
      return false;
    }
    loop_counter++;
    usleep( sleep_interval_us );
  }
  return true;   
}


/**********************************************************************/
/**********************************************************************/
ssize_t ArduinoInterface::arduinoI2cWrite( uint8_t device_address, int frequency, uint8_t reg_address, uint8_t* data, size_t num_bytes )
{
  // Encode i2c frequency data into data_packet_:
  switch( frequency )
  {
  case 100000:
    // bytes 5,4,3 should already be zero. no action needed.
    break;
    // code: 000 000 01
  case 400000:
    // set the appropriate bits high
    data_packet_ |= (1 << 2);
    // code: 000 001 01
    break;
  default:
    ROS_ERROR("Arduino cannot read at this frequency.");
    return -1; // error code. 
  }
   
  uint8_t i2c_write_prompt[4] = { data_packet_, (uint8_t)device_address, reg_address, num_bytes };
  serial_port_->Write_Bytes( 4, i2c_write_prompt );
   
  //Wait for verification:
  waitOnBytes( 1 );
   
  int verification = serial_port_->Read();
   
  // Flush the buffer before receiving the data:
  serial_port_->Flush();
  if( verification != VERIFY )
  {
    ROS_INFO("No response to i2c write prompt. Instead, 0x%x", verification);
    return -1; // error code.
  }

  serial_port_->Write_Bytes( num_bytes, data );

  // wait for Arduino's Verification code:
  waitOnBytes( 1 );
   
  // Verify:
  verification = serial_port_->Read();
  if( verification != SUCCESS )
  {
    ROS_ERROR("arduinoI2cWrite error.  No verification.");
    return -1;
  }
   
  return num_bytes;
}


/**********************************************************************/
/**********************************************************************/
ssize_t ArduinoInterface::arduinoSpiWrite( int frequency, int* flags, uint8_t reg_address, uint8_t* data, size_t num_bytes )
{
  // construct data_packet_ flag:
  data_packet_ |= (SPI << 5) | (frequency << 2); // EX: spi, div16, write : 001 001 01
  
  // construct array to send to Arduino:
  uint8_t write_packet[num_bytes + 4];
  // load it with setup parameters and data:
  write_packet[0] = data_packet_;
  write_packet[1] = *flags;
  write_packet[2] = reg_address;
  write_packet[3] = num_bytes;

  for( uint8_t i = 0; i < num_bytes; i++ )
  {
    write_packet[i+4] = data[i];
  }
   
  // send the data:
  serial_port_->Write_Bytes( (num_bytes + 4), write_packet );
   
  usleep( 5000 );    
  //Wait for verification:
  waitOnBytes( 1 );
   
  uint8_t verification = serial_port_->Read();
   
  //serial_port_->Flush();
  if( verification != VERIFY )
  {
    ROS_INFO("No response to spi write prompt. Instead, 0x%x", verification);
    return -1; // error code.
  }
   
  return num_bytes;
}


/**********************************************************************/
/**********************************************************************/
ssize_t ArduinoInterface::arduinoSpiRead( int frequency, int* flags, uint8_t reg_address, uint8_t* data, size_t num_bytes )
{
  data_packet_ = 0;
  data_packet_ |= (SPI << 5) | (frequency << 2); // spi, div16, read : 001 001 00
 
  uint8_t spi_read_prompt[4] = { data_packet_, * flags, reg_address, num_bytes };
                
  serial_port_->Write_Bytes( 4, spi_read_prompt );
   
  // wait for the Arduino to verify these commands:
  //TIMEOUT CHECK ROUTINE:
  // BEGIN: HACK   
  //bool error = waitOnBytes(1);
  //if (error == false)
  //{
  //ROS_ERROR(" Read broke: Arduino did not verify Serial SPI data to be transmitted.");
  //return -1;
  //}  
  while( serial_port_->Available() < 1 )
    ; // do nothing until verification arrives.
  // END: HACK
   
  // Verify:
  int verification = serial_port_->Read();

  if( verification != VERIFY )
  {
    ROS_INFO( "No response to SPI read prompt. Instead:  0x%x", verification );
    return -1; // error code.
  }
  
  // Timeout Check Routine: 
  bool error = waitOnBytes( num_bytes );
  if( error == false )
  {
    ROS_ERROR( "Read broke: Arduino did not return SPI data." );
    return -1; // error code.
  }
  //while( serial_port_->Available() < num_bytes);
  // Read num_bytes bytes off the serial line:
  if( serial_port_->Read_Bytes( num_bytes, data ) == false )
    return -1; // error code.
  else
    return num_bytes;
}



/**********************************************************************/
ssize_t ArduinoInterface::arduinoI2cRead(uint8_t device_address, int frequency, uint8_t reg_address, uint8_t* data, size_t num_bytes )
{
  data_packet_ = 0;  // reset data packet. 
  switch( frequency )
  {
  case 100000:
    // bytes 5,4,3 should all be zero. no action needed.
    // code: 000 000 00
    data_packet_ = 0;
    break;
  case 400000:
    // set the appropriate bits high
    data_packet_ |= (1 << 2);
    // code: 000 001 00, which is 0x04
    break;
  default:
    ROS_ERROR("ArduinoInterface::arduinoI2cRead(): Arduino cannot read at this frequency.");
    return -1; // error code.
  }
   
  uint8_t i2c_read_prompt[4] = { data_packet_, (uint8_t)device_address, reg_address, num_bytes };
                
  serial_port_->Write_Bytes( 4, i2c_read_prompt );  
   
  // BEGIN: HACK
  // wait for the Arduino to verify these commands:
  //Timeout Check Routine:
  //error = waitOnBytes(1);  // HACK: comment this out and ...Now, it works??
  //if (error == false)
  //{
  //  ROS_ERROR("Read broke.");
  //  return -1;
  // }
  while( serial_port_->Available() < 1 )
    ; // HACK: put this line in instead.
  // END: HACK

  // Verify:
  int verification = serial_port_->Read();

  if( verification != VERIFY )
  {
    ROS_ERROR("ArduinoInterface::arduinoI2cRead(): No response to I2C read prompt. Instead:  0x%x", verification);
    return -1; // error code.
  }
   
  // Wait for data to arrive from sensor:
  //Timeout Check Routine:   
  bool error = waitOnBytes( num_bytes );
  if( error == false )
  {
    ROS_ERROR("ArduinoInterface::arduinoI2cRead(): Read broke: did not receive data back.");
    return -1; // error code.
  } 
   
  // Read num_bytes bytes off the serial line:
  if( serial_port_->Read_Bytes( num_bytes, data) == false )
    return -1; // error code.
  else
    return num_bytes;
}
