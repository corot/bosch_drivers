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

/*
 * This class contains a set of parameters that are specific to the 
 * BMC050 sensor.  The class inherits from a generic parent class of 
 * parameters.  I have listed the inherited parameters as comments
 * below.
 * The sensor parameters are stored as members of the class and 
 * accessible to other classes through methods such as 
 * "getProtocol()."
 */

#ifndef BMC050_PARAMETERS_H_
#define BMC050_PARAMETERS_H_

// ROS debugging output
#include <ros/console.h>

#include <bosch_drivers_parameters.hpp>
#include <bosch_drivers_hardware_interface.hpp>

using namespace bosch_drivers_common;

class BMC050Parameters: public Parameters
{
public:
  friend class BMC050;

  enum accel_range
  {
    RANGE_2  = 0x03, // ±2.0 g   3.91  [mg/LSB]
    RANGE_4  = 0x05, // ±4 g    7.81 [mg/LSB]
    RANGE_8  = 0x08, // ±8 g    15.62 [mg/LSB]
    RANGE_16 = 0x0C  // ±16 g    31.25 [mg/LSB]
  };

  enum bandwidth
  {
    BW_8HZ    = 0x08, // 7.81  [Hz]
    BW_16HZ   = 0x09, // 15.63 [Hz]
    BW_31HZ   = 0x0A, // 31.25 [Hz]
    BW_63HZ   = 0x0B, // 62.5 [Hz]
    BW_125HZ  = 0x0C, // 125  [Hz]
    BW_250HZ  = 0x0D, // 250  [Hz]
    BW_500HZ  = 0x0E, // 500  [Hz]
    BW_1000HZ = 0x0F  // 1000 [Hz]
  };


  // Compass Output Data Rate: 
  // write to 0x4C
  enum compass_rate 
  {
    ODR_10HZ = 0x00, // 000b  default   
    ODR_2HZ  = 0x01, // 001b
    ODR_6HZ  = 0x02, // 010b
    ODR_8HZ  = 0x03, // 011b
    ODR_15HZ = 0x04, // 100b
    ODR_20HZ = 0x05, // 101b
    ODR_25HZ = 0x06, // 110b
    ODR_30HZ = 0x07  // 111b
  };
 
  /**
   * \brief constructor
   */
  BMC050Parameters();

  /**
   * \brief destructor
   */
  ~BMC050Parameters();
  
  /**
   * \brief  Change BMC050 protocol parameter to a different protocol.
   *
   * The protocol depends on the physical hardware configuration 
   * of the sensor.  
   */
  bool setProtocol( interface_protocol protocol );
  
  
 /**
  * \brief  change interface protocol frequency to a different 
  *      frequency.
  *
  * User should not need to change this value.  The frequency
  * set upon instantiation is functional.
  */
  bool setFrequency( int frequency );

 /**
  * \brief  This method must be implemented since
  *      it is inheritted from a virtual method in the parameters 
  *      class.
  *
  * Since the BMC050 is actually two sensors, it has two 
  *      separate chip-select pins. 
  * \return  \a pin_, which has no effect on this class
  */   
  bool setPin( uint8_t pin ); // always false... Needed because of inheritance

  /**
   * \brief  Alert software driver of the physical pin of the hardware 
   *      interface that the accelerometer's chip-select is 
   *      connected to.
   *   
   *      This is an SPI-relevant parameter only.
   */   
  bool setAccelPin( uint8_t pin );
  
  /**
   * \brief  alert software driver of the physical pin of the hardware 
   *      interface that the compass's chip-select is connected to.  
   *      This is an SPI-relevant parameter only.
   */ 
  bool setCompassPin( uint8_t pin );

  /// Accelerometer-specific
  /**
   * \param accel_range new_range an enumerated datatype specifying one of 
   *     the sensor's selectable ranges.  (Sensor initializes with 
   *     default range if no range is specified.)
   * \note this user-choice is then applied to the sensor upon 
   *     initialization.
   */
  void setAccelRange( accel_range new_range );

  /**
   * \brief choose whether or not to use filtered(default) or unfiltered
   *     data from the accelerometer.
   *
   * \param bool input true indicates filtered data; false indicates 
   *     unfiltered data.
   * \note this user preference is applied upon sensor initialization.
   */
  void setFilter(bool input);        

  /**
   * \brief select the bandwidth that the sensor will respond to.  See the
   *     datasheet for more information.
   *
   * \param bandwidth bw an enumerated datatype specifying one of the 
   *     available accelerometer bandwidths.
   * \note this user preference is applied upon sensor initialization.
   */
  void setAccelBandwidth( bandwidth bw ); // sets bandwidth register according to desired bandwidth
  
  // compass-specific
  /**
   * \brief select the rate at which the compass will output its data.
   *
   * \param compass_rate rate an enumerated dataype specifying one of the 
   *     available compass data rates.
   * \note this user preference is applied upon sensor initialization.
   */
  void setCompassRate( compass_rate rate ); 

  /**
   * \brief select number of repetitions that the compass will sample 
   *     before outputting its average for the X and Y axes.
   *
   * \param an unsigned integer from 0 through 511.
   * \note only odd values are valid
   */
  void setNumRepetitionsXY( uint16_t num_repetitions );
  
  /**
   * \brief select number of repetitions that the compass will sample 
   *     before outputting its average for the Z axis.
   *
   * \param an unsigned integer from 0 through 256.
   */
  void setNumRepetitionsZ( uint16_t num_repetitions );
  
  // These pins determine Accel and Compass addresses:
  /**
   * \brief alert software driver of the harware pin configuration for the
   *     CSB2 pin.  (aka, is CSB2 connected to VDD or GND ?)
   *
   * \param bool val true indicates that CSB2 is connected to VDD.  false 
   *     indicates that CSB2 is connected to GND.
   * \note the value of \a both CSB2 and SDO will determine the i2c 
   *     addresses of both two sensors: compass and accelerometer
   * \note thise settings are only important in i2c mode.
   */
  void setCSB2( bool val );

  /**
   * \brief alert software driver of the harware pin configuration for the
   *     SDO pin.  (aka, is SDO connected to VDD or GND ?)
   *
   * \param bool val true indicates that SDO is connected to VDD.  false 
   *     indicates that SDO is connected to GND.
   * \note the value of \a both CSB2 and SDO will determine the i2c 
   *     addresses of both two sensors: compass and accelerometer
   * \note these settings are only important in i2c mode.
   */
  void setSDO( bool val );
  
  /**
   * \brief a wrapper function that returns the currently specified 
   *     accelerometer range.
   *
   * \return \a accel_range_
   */
  accel_range getAccelRange();
  
  // FIXME: change level of access on these methods:  
  uint8_t getAccelAddress(); // return the i2c addresses based on
  uint8_t getCompassAddress(); // settings in the two previous methods.
  
  /**
   * \brief a wrapper function that returns the currently specified 
   *     accelerometer sensitivity.
   *
   * \note this sensitivity is automatically adjusted when the  
   *     accelerometer range is adjusted.
   * \return \a accel_sensitivity_
   */
  double getAccelSensitivity();

  interface_protocol getProtocol();
  int getFrequency();
  int* getFlags();
  int getPin(); // valid for SPI only


private:
  accel_range accel_range_;
  bool accel_is_filtered_;  
  double accel_sensitivity_;
  bandwidth bw_reg_;
  compass_rate  compass_rate_;
  uint8_t repsXY_;
  uint8_t repsZ_;
  
  bool CSB2;
  bool SDO;
  uint8_t accel_pin_;
  uint8_t compass_pin_;

  /** 
   * \brief  configure the hardware-interface's SPI mode by which it 
   *      communicates with the sensor.
   *
   * \param mode  the SPI mode of the sensor. Valid modes: SPI_MODE0,
   *         SPI_MODE1, SPI_MODE2, SPI_MODE3.
   * \note  see the wikipedia article on SPI for more information on the
   *      corresponding CPOL and CPHA settings.
   * \note   valid modes are defined in the bosch_drivers_common.
   * \note  The user should NOT have to change this value from a value
   *      different from the value set upon instantiation.
   * \return boolean indicating successful input.
   */
  bool setSpiMode( uint8_t mode );

  /** 
   * \brief configure the hardware-interface's SPI byte-order by which it 
   *     will communicate with the sensor.
   *
   * \note byte-order arguments are either MSBFIRST or LSBFIRST, 
   *     both of which are defined in the bosch_drivers_common.
   * \note  The user should NOT have to change this value from a value
   *      different from the value set upon instantiation.
   * \note   in detail, this method directly manipulates the class flags_
   *      value, which is passed to the hardware interface class.
   */
  bool setByteOrder( uint8_t value ); // MSB_FIRST or LSB_FIRST

};

#endif // BMC050_PARAMETERS_H_
