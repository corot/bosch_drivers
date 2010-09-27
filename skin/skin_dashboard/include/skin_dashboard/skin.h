/*
 * Copyright (c) 2010, Bosch LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Bosch LLC nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//\Author Joerg Wagner, Bosch LLC

#ifndef SKIN_H_
#define SKIN_H_

//ROS Headers
#include <ros/ros.h>

//wxwdigets Headers
#include <wx/wx.h>

// messages
#include <com_ecu/com_ecu_meas.h>

//! \brief A namespace with constants
namespace skind_def {
	//constants
	const int max_sensor_number = 192;
	const int anz_byte_per_sensor = 8;
}

//! \brief A Class to subscribe the data provided by the publisher node (com_ecu)
/*!
 * The class Skin handles the subscription of the topic skin_data.
 * The sensor data received from the topic are decrypted and stored in this class. If the publisher node
 * is offline the class initializes the sensor data with default values.
 */
class Skin
{
public:
	//! A constructor to initialize the sensor data stored in this class and to create a Subscriber
	/*!
	 * The construktor uses the function "initialize_sensor_data" to set the sensor data to default values.
	 */
	Skin();

	//! The function "skin_Callback" subscribs data from the topic and decrypts them
	void skin_Callback(const boost::shared_ptr<com_ecu::com_ecu_meas const>& msg);

	//! Initialize all sensor data stored in the object of the class
	/*!
	 * Initialization:
	 * 		- sensor values = 0 (1536 values);
	 * 		- sensor number	= skind_def::max_sensor_number
	 * 		- high-byte of the address which represents the beginnign of the sensor data in the RAM = 1
	 * 		- low-byte of the address which represents the beginnign of the sensor data in the RAM = 0
	 * 		- Status display Flag Go/NoGo (bstatus_go_)= 0
	 * 		- Status display Flag Err/NoErr (bstatus_err_)= 1
	 */
	void initialize_sensor_data();//initialize sensor data

	//! Time stamp of the last subscription
	ros::Time dtime_subscription_;

    ///////////////////////////////////////
	//data structure Sensor Skin
	//////////////////////////////////////

	//! High-byte of the address which represents the beginning of the sensor data in the RAM
	uint8_t u8readadr_hi_;
	//! Low-byte of the address which represents the beginning of the sensor data in the RAM
	uint8_t u8readadr_lo_;
	//! Number of sensor elements connected to the sensor skin
	uint8_t u8sensor_number_;

	//! Status display Go/NoGo
	/*!
	 * Conditions:
	 * 		- "bstatus_go_ == true" equals GO
	 * 		- "bstatus_go_ == false" equals NoGO
	 */
	bool	bstatus_go_;

	//! Status display Err/NoErr
	/*!
	 * Conditions:
	 * 		- "bstatus_err_ == true" equals Err
	 * 		- "bstatus_err_ == false" equals NoErr
	 */
	bool	bstatus_err_;

	//! If true: sensor number was changed
    bool bsens_num_changed;


	//! Data from sensor skin
    /*!
     * sensor_data layout
     *
     * 				Index:		0     1    2      3      4   5    6     7
     *
     * 	sensor_data[0][0..7] = Stat | X | Ini | Limit | U1 | U2 | U3 | Dyn
     *  sensor_data[1][0..7] = Stat | X | Ini ...
     *  .
     *  .
     *  .
     */
	uint8_t u8asensor_data_[skind_def::max_sensor_number][skind_def::anz_byte_per_sensor]; // 192 sensors, 8 Bytes per sensor

private:
	//////
	//ROS
	//////
	ros::NodeHandle nh_;
	ros::Subscriber skin_sub_;
};

#endif /* SKIN_H_ */
