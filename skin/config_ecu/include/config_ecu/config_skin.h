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

#ifndef CONFIG_SKIN_H_
#define CONFIG_SKIN_H_

//wxwidgets stuff
#include <wx/wx.h>

//ROS Headers
#include <ros/ros.h>

// messages
#include <com_ecu/com_ecu_meas.h>

//services
#include <com_ecu/com_ecu_serv.h>

//! \brief A Class to handle the communication with the publisher node (com_ecu)
/*!
 * The class SkinConfig handles the communication with the publisher node.
 * It subscribes the topic "skin_data" and uses the Service "skin_serv".
 * The data received from the topic are stored in this class.
 */
class SkinConfig
{
public:

	//! A constructor to initialize values and to create a Subscriber and Client
	/*!
	 * Default values:
	 * 		- ui8sensor_number_ = 0;
	 * 		- ui8dyn_thres_currentvalue_ = 255;
	 * 		- dtime_subscription_ = ros::Time::now();
	 */
	SkinConfig();

	//! The function "skin_Callback" subscribs data from the topic
	void skin_Callback(const boost::shared_ptr<com_ecu::com_ecu_meas const>& msg);

	//! Calls the Service "skin_serv" with provided parameters
	/*!
	 * OnRequestChanges: - calls the Service skin_serv (request a write to RAM command from node "com_ecu")
	 * 					 - returns true if call of service was successeful else false
	 *
	 * \param start_adr Memory address of first byte in which the node "com_ecu" should write a value
	 * \param steps The node "com_ecu" will increment the address "start_adr" (a copy of "start_adr") by "steps" after one "write to RAM" transaction
	 * \param number_of_bytes Number of bytes which the node "com_ecu" should write into the RAM
	 * \param write_data Data which the node "com_ecu" should write into RAM
	 * \param sens_num_change If "sens_num_change" is true and "write_data" equals 254 the node "com_ecu" will delete the
	 * old end of chain marker
	 *
	 * return The returned value is an indicator for the success of the service call
	 */
	bool OnRequestChanges(const uint8_t start_adr[2], const uint8_t steps, const uint8_t number_of_bytes, uint8_t write_data, bool sens_num_change);

	//! Time stamp of last message
	ros::Time dtime_subscription_;


	//////////
	//Flags
	//////////

	//! if bcom_ecu_node_online == true: node cmo_ecu is online
    bool bcom_ecu_node_online;


    ////////////////////////////////////////////////////////
	//data structure Sensor Skin / configuration interface
	////////////////////////////////////////////////////////

	//! Number of sensors connected to the sensor skin
	uint8_t ui8sensor_number_;
	//! Current value of dynamic threshold
	uint8_t ui8dyn_thres_currentvalue_;

private:

	//////////////////
	//ROS variables
	//////////////////

	ros::NodeHandle nh_;
	ros::Subscriber skin_sub_;
	ros::ServiceClient config_client;

	//! ROS service
	com_ecu::com_ecu_serv srv_;
};

#endif /* CONFIG_SKIN_H_ */
