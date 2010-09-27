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

#include "skin_dashboard/skin.h"

//wxwidgets stuff
#include <wx/wx.h>

Skin::Skin()
{
	//initialize data
	initialize_sensor_data();//function to initialize the sensor data
	dtime_subscription_ = ros::Time::now();
	bsens_num_changed = false;

    //ROS Subscriber
    skin_sub_ = nh_.subscribe("skin_data", 100, &Skin::skin_Callback, this);
}

void Skin::initialize_sensor_data()//function to initialize the sensor data
{
	//initialize data
	u8sensor_number_	= skind_def::max_sensor_number;
	u8readadr_hi_ = 1; //hardware defined values
	u8readadr_lo_ = 0;
	bstatus_go_ = 0;
	bstatus_err_ = 1;
    for(int i=0; i<skind_def::max_sensor_number; i++)
	{
    	for(int j=0; j<skind_def::anz_byte_per_sensor; j++)
		{
    		u8asensor_data_[i][j] = 0;
		}
	}
}

void Skin::skin_Callback(const boost::shared_ptr<com_ecu::com_ecu_meas const>& msg)
{
	//check if sensor number changed
	if(u8sensor_number_ != msg->sensor_number_msg)
	{
		bsens_num_changed = true;//sensor number was changed to a smaller value => GUI needs to be initialized (if the gui is not initialized => old data of the no longer existing sensors are still displayed)
	}
	u8sensor_number_ = msg->sensor_number_msg;
	u8readadr_hi_ = msg->readadr_hi_msg;
	u8readadr_lo_ = msg->readadr_lo_msg;
	bstatus_go_ = msg->status_go_msg;
	bstatus_err_ = msg->status_err_msg;

	//Converting a 1 dimensional uint64 array into a 2 dimensional uint8 array
	std::cout << "skin_Callback" << " \n";

	for(int i = 0; i < skind_def::max_sensor_number; i++)//convert received sensor data into 2 dimensional uint8_t array
	{
		for(int j = 0; j < skind_def::anz_byte_per_sensor ; j++)
		{
			u8asensor_data_[i][j] = ( (255) & (msg->sensor_data_msg[i] >> (8 * j)) );
		}
	}

	dtime_subscription_ = ros::Time::now();
}
