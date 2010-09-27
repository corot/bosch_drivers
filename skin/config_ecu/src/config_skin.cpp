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

#include "config_ecu/config_skin.h"
#include <iostream>

SkinConfig::SkinConfig()
{
	//initialize values
	ui8sensor_number_ = 0;
	ui8dyn_thres_currentvalue_ = 255;
	dtime_subscription_ = ros::Time::now();

    //ROS Subscriber
    skin_sub_ = nh_.subscribe("skin_data", 100, &SkinConfig::skin_Callback, this);//Topic: skin_data

	//ROS Client
	config_client = nh_.serviceClient<com_ecu::com_ecu_serv>("skin_serv");//Service: skin_serv
}

void SkinConfig::skin_Callback(const boost::shared_ptr<com_ecu::com_ecu_meas const>& msg)//callback function
{
	ui8sensor_number_ = msg->sensor_number_msg;
	dtime_subscription_ = msg->header.stamp;
	ui8dyn_thres_currentvalue_ = msg->dyn_thres_cv_msg;

	std::cout << "skin_Callback" << " \n";
}

bool SkinConfig::OnRequestChanges(const uint8_t start_adr[2], const uint8_t steps, const uint8_t number_of_bytes, uint8_t write_data, bool sens_num_change)
{
	if(bcom_ecu_node_online == true)
	{
		srv_.request.header.stamp = ros::Time::now();
		srv_.request.start_adr_srv[0] =  start_adr[0];
		srv_.request.start_adr_srv[1] =  start_adr[1];
		srv_.request.steps_srv = steps;
		srv_.request.number_of_bytes_srv = number_of_bytes;
		srv_.request.write_data_srv = write_data;
		srv_.request.sens_num_change_srv = sens_num_change;

		if(config_client.call(srv_))
		{
			return srv_.response.write_to_skin_successful_srv;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}












