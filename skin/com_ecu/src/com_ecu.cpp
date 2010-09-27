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

//ROS Headers
#include <ros/ros.h>
#include <ros/time.h>

#include "com_ecu/com_ecu.h"
#include "com_ecu/com_ecu_meas.h"//topic

//Common Headers
#include <iostream>
#include <sstream>
#include <string.h>
#include <math.h>

//TODO:
//  - implement a useful application with dynamic reconfigure or delete the code belonging to dynamic reconfigure
//	- change the software to handle 192 instead of 191 sensors
//	- bug-fix: After the execution of command "Ram to Flash" the dynamic threshold is changed to zero.
//	  In the most cases the dynamic threshold could be reset to the old value by restarting the ECU (or several restarts).
//	- Increase the I2C-BUS speed of the ECU (Must be changed in the assembler code of the ECU)
//			*An increased I2C-BUS speed could slow down the evaluation software of the sensor skin
//	- Implementation of error handling routines

Skin::Skin():
bSubDeviceOpen_ (false),
bSubDeviceConfigured_ (false),
bI2CDeviceFound_ (false),
bsub20found_ (false),
strSerial_ ("         ")
{
	int iI2CErr = 0; //Error code for Sub20 API
	int iI2C_freq_set; //Set I2C frequency
	int iI2C_freq_get = 0; //Get I2C frequency

	while( ( subdev_ = sub_find_devices(subdev_) ) && (!bsub20found_) )//scan for subdevice with I2C connection until skin is found
	{
		std::cout << std::endl << "+++++++  Scan for Sub20-Device with connection to Skin  +++++++" << std::endl;

		fd_ = sub_open( subdev_ );//open found subdevice

		// on success, sub_open returns non-zero handle
		if( fd_ == 0 ) {
			//sub_open was not successful
			std::cout << "sub_open: " << sub_strerror(sub_errno) << std::endl;
		}
		else
		{
			//Subdevice successfully opened
			bSubDeviceOpen_ = true;
			std::cout << "Device Handle   : " << fd_ << std::endl;
			//Read Serial number of subdevice
			if( sub_get_serial_number(fd_, const_cast<char*>(strSerial_.c_str()), strSerial_.size() ) >= 0 )
			   std::cout << "Serial Number   : " << strSerial_ << std::endl;
			std::cout << "------------Initializing I2C Interface------------" << std::endl;
			/* Read current I2C frequency */
			iI2C_freq_get = 0;//read => 0
			iI2CErr = sub_i2c_freq(fd_, &iI2C_freq_get);
			std::cout << "Predefined I2C frequency  : " << iI2C_freq_get << std::endl;
			/* Set I2C frequency */
			iI2C_freq_set = skinc_def::I2C_communication_frequency;//Standart Mode: 100kHz (possible frequencies 489 - 100000Hz)
			iI2CErr = sub_i2c_freq(fd_, &iI2C_freq_set);
			std::cout << "Initialized I2C frequency  : " << iI2C_freq_set << std::endl;
			/* Read current I2C frequency */
			iI2C_freq_get = 0;//read => 0
			iI2CErr = sub_i2c_freq( fd_, &iI2C_freq_get );

			/* Configure I2C */
			iI2CErr = sub_i2c_config(fd_, 0x50, 0x00);
			std::cout << "Status config: " << sub_i2c_status << std::endl;

			//verify if sub20 device has accepted the configuration (frequency)
			if (iI2C_freq_get == iI2C_freq_set )
			{
				std::cout<< "Frequency (" << iI2C_freq_set << "Hz) successfully stored \n";
				//Subdevice has been configured successfully
				bSubDeviceConfigured_ = true;
			}
			else
			{
				std::cout<< "ERROR - Frequency:" << iI2C_freq_set << " not accept by device \n";
				//Subdevice could not be configured
				bSubDeviceConfigured_ = false;
			}
		}

		//only execute if SubDevice is configured
		if ( bSubDeviceConfigured_ )
		{
			//scan for slave devices
			std::cout << "----------------Scan for I2C devices--------------" << std::endl;
			int		ianz_slave;//Buffer to store number of devices
			int 	iErrScan;//Error code for Sub20 API
			char    cdevice_buf[128];//Buffer to store device addresses

			iErrScan = sub_i2c_scan( fd_, &ianz_slave, cdevice_buf );//scan for devices
			if( !iErrScan )//scan was successful
			{
				std::cout << "I2C Slaves: " << ianz_slave << std::endl;
				for(int i = 0; i < ianz_slave; i++ )
				{
				  std::cout << (i+1) << ". Slave adress:" << int (cdevice_buf[i]) << std::endl;// display found devices

				  /////////////////////////////////////////////////////////////////
				  //verify if defined (skinc_def::I2CAdr) I2C device exists
				  if( skinc_def::I2CAdr == (int (cdevice_buf[i])) )
				  {
					  bI2CDeviceFound_ = true;//I2C device exists
					  std::cout << "Slave adress " << skinc_def::I2CAdr <<" exists" << std::endl;
				  }
				  /////////////////////////////////////////////////////////////////

				}
			}

			//only execute if I2C device connected to sub20
			if ( bI2CDeviceFound_ )
			{
				/////////////////////////////////////////////
				//verify that connected sensor = sensor skin
				//condition:   value Stat of sensor 1 < 64
				/////////////////////////////////////////////

				std::cout << "----check if connected device is a sensor skin----" << std::endl;

				char	cabuftest[8] = {0};

				//send: Start|Slave Adr.|write|Mem. Adr. MSB/Hi|Mem. Adr. LSB/Lo|Stop
				char camem_adr[2] = {(skinc_def::readadr_hi), skinc_def::readadr_lo};//generate address array (address of first sensor)
				iI2CErr = sub_i2c_write(fd_, skinc_def::I2CAdr, 0x00, 0, camem_adr, skinc_def::I2C_Mem_Adr);//Memory Address Write
				std::cout << "Status write Mem. Adr.: " << sub_i2c_status << std::endl;

				ros::Duration(0.05).sleep();//sleeps for 0.05 seconds

				//receive data from ECU (Start|Slave Adr.|Data[0]|...|Stop)
				iI2CErr = iI2CErr + sub_i2c_read(fd_, skinc_def::I2CAdr,0x00, 0, cabuftest, 8);//read data (8 Byte)
				std::cout << "Status read RAM: " << sub_i2c_status << std::endl;

				if( (iI2CErr == 0)&&( ((int (cabuftest[0]))&0xFF) < 64  ) )//condition for sensor skin:   value Stat < 64
				{
					std::cout << "Connected device is a sensor skin" << std::endl;
					bsub20found_ = true;
				}
				else
				{
					std::cout << "Found sensor is not a sensor skin" << std::endl;
					std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++" <<std::endl;
					bSubDeviceOpen_ = false;//reset flags for next scan
					bSubDeviceConfigured_ = false;
					bI2CDeviceFound_ = false;
					bsub20found_ = false;
				}
			}
			else
			{
				std::cout << "Tested Sub20 is not connected with a I2C device" << std::endl;
			}
		}
		else
		{
			std::cout << " >> Subdevice is not configured " << std::endl;
		}
	}


	//////////////////////////////////////////////////////////////////////////
    //determine number of sensor elements
    //first unused Sensornumber: Limit = 254 (skinc_def::end_of_chain_marker)
	//
	//The software tests the limit of each sensor until end of chain marker
	//is found or the end of the sensor skin is reached
	//max. number of sensors: skinc_def::max_sensor_number
	//////////////////////////////////////////////////////////////////////////

	isensor_number_ = skinc_def::max_sensor_number;

	//only execute if SubDevice is configured and I2C device found
	if ( bSubDeviceConfigured_ & bI2CDeviceFound_ & bsub20found_ )
	{
	    std::cout << "------------Determine number of sensors-----------" << std::endl;

		for(int i = 0; (i < 6)&&(isensor_number_ == skinc_def::max_sensor_number); i++)//read sensor values (in 256 Byte blocks) until Limit = 254 found
		{
			char	cbufskinsens[256] = {0};

			//send: Start|Slave Adr.|write|Mem. Adr. MSB/Hi|Mem. Adr. LSB/Lo|Stop
			char camem_adr[2] = {(i+skinc_def::readadr_hi), skinc_def::readadr_lo};//generate address array
			iI2CErr = sub_i2c_write(fd_, skinc_def::I2CAdr, 0x00, 0, camem_adr, skinc_def::I2C_Mem_Adr);//Memory Address Write
			std::cout << "Status write Mem. Adr.: " << sub_i2c_status << std::endl;

			ros::Duration(0.05).sleep();//sleeps for 0.05 seconds

			//receive data from ECU (Start|Slave Adr.|Data[0]|...|Stop)
			iI2CErr = iI2CErr + sub_i2c_read(fd_, skinc_def::I2CAdr,0x00, 0, cbufskinsens, 256);//read data, max. read size: 256 Byte
			std::cout << "Status read RAM: " << sub_i2c_status << std::endl;

			for(int  k = 4; (k < 256)&&(isensor_number_ == skinc_def::max_sensor_number); k = k + 8)//check limit of sensor elements
			{
				if(((int (cbufskinsens[k-1]))&0xFF) == skinc_def::end_of_chain_marker)//determin sensorelement with limit = 254
				{
					isensor_number_ = int((k+4)/8) + 32*i - 1;//evaluate last sensor element
				}
			}
		}

		std::cout << "sensors: " << isensor_number_ << std::endl;
		binit_skin_ok_ = bsub20found_;

	}
	else
	{
		std::cout << std::endl <<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
		std::cout << "         Error -Sensor skin not found-" << std::endl;
		std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl << std::endl;
	}

	std::cout<< "**************************************************" << std::endl;

}

Skin::~Skin()
{
	// Close SUB20-Device device
	sub_close( fd_ );
	//Reset status
	bSubDeviceOpen_ = false;
	bSubDeviceConfigured_ = false;
	bI2CDeviceFound_ = false;
	bsub20found_ = false;
	std::cout << "-------------------Sub20 closed-------------------" << std::endl;
}

SKINMEAS Skin::GetOneMeas()//Reads one SKIN measurement (192 sensors per 8 Byte)
{
	SKINMEAS 	sMeas;//Measurement results
	int 		iI2CErr;
	char		cbufskinsens[1536] = {0};//Buffer to store sensor data
	char		cbufskinlog[2] = {0};//Buffer to store lof data

    sMeas.bMeasAvailable = false;
    sMeas.dyn_thres_cv = 255;
    sMeas.status = 0;

	//only execute if SubDevice is configured and I2C device found
	if ( bSubDeviceConfigured_ & bI2CDeviceFound_ & bsub20found_ )
	{
		///////////////////////////////////////////////
		//read LOG-Files from sensor skin
		///////////////////////////////////////////////

		//read 2 Byte log files

		iI2CErr = 0;

		//send: Start|Slave Adr.|write|Mem. Adr. MSB/Hi|Mem. Adr. LSB/Lo|Stop
		char camem_adr_log[2] = {(skinc_def::readadr_log_hi), skinc_def::readadr_log_lo};//generate address array
		iI2CErr = sub_i2c_write(fd_, skinc_def::I2CAdr, 0x00, 0, camem_adr_log, skinc_def::I2C_Mem_Adr);//Memory Address Write
		//std::cout << "Status write Mem. Adr.: " << sub_i2c_status << std::endl;

		ros::Duration(0.05).sleep();//sleeps for 0.05 seconds (50msec sleep time required for ECU / Microcontroller of ECU is to slow to handle a I2C communication without sleep time)

		//receive data from ECU (Start|Slave Adr.|Data[0]|...|Stop)
		iI2CErr = iI2CErr + sub_i2c_read(fd_, skinc_def::I2CAdr,0x00, 0, cbufskinlog, 2);//read data, read size: 2 Byte
		//std::cout << "Status read RAM: " << sub_i2c_status << std::endl;

		//extract data out of Byte
		sMeas.dyn_thres_cv = uint8_t ((int (cbufskinlog[1]))&0xFF);//Dyn. threhold
		sMeas.status = uint8_t ((int (cbufskinlog[0]))&0xFF);//Statusbyte (Go/NoGo - Bit0, Err/NoErr - Bit1)


		//std::cout << std::endl << "Read sensor data" << std::endl;


		////////////////////////////////////////////////////
		//Perform complete I2C sensor data read transaction
		///////////////////////////////////////////////////


		for(int k = 0; k < int( ceil((8*double(isensor_number_))/256) ); k++)
		{
			//send: Start|Slave Adr.|write|Mem. Adr. MSB/Hi|Mem. Adr. LSB/Lo|Stop
			char camem_adr[2] = {(k+skinc_def::readadr_hi), skinc_def::readadr_lo};//generate address array
			iI2CErr = iI2CErr + sub_i2c_write(fd_, skinc_def::I2CAdr, 0x00, 0, camem_adr, skinc_def::I2C_Mem_Adr);//Memory Address Write
			//std::cout << "Status write Mem. Adr.: " << sub_i2c_status << std::endl;

			ros::Duration(0.05).sleep();//sleeps for 0.05 seconds (50msec sleep time required for ECU / Microcontroller of ECU is to slow to handle a I2C communication without sleep time)

			if(k < int((8*isensor_number_)/256))//read data of 32 sensors
			{
				//receive data from ECU (Start|Slave Adr.|Data[0]|...|Stop)
				iI2CErr = iI2CErr + sub_i2c_read(fd_, skinc_def::I2CAdr,0x00, 0, &cbufskinsens[k*256], 256);//read data, max. read size: 256 Byte
				//std::cout << "Status read RAM: " << sub_i2c_status << std::endl;
			}
			else//read data of <32 sensors (remaining sensors)
			{
				//receive data from ECU (Start|Slave Adr.|Data[0]|...|Stop)
				iI2CErr = iI2CErr + sub_i2c_read(fd_, skinc_def::I2CAdr,0x00, 0, &cbufskinsens[k*256], int( fmod((8*double(isensor_number_)) , 256) ));//max. read size: 256 Byte
				//std::cout << "Status read RAM: " << sub_i2c_status << std::endl;
			}

		}

		if(iI2CErr != 0)
		{
			sMeas.bMeasAvailable = false;
			std::cout << "--------------------------------------------------------------------------------------------------" << std::endl;
			std::cout << "     !!! ERROR: master read transaction failed !!! / " << std::endl << std::endl;
			std::cout << "--------------------------------------------------------------------------------------------------" << std::endl;
		}
		else
		{
			sMeas.bMeasAvailable = true;
			convert_data_2_uint64_array(cbufskinsens, sMeas.sensor_data);//convert char array to int array
		}
	}
	else
	{
		std::cout << "-------------------------------------------------" << std::endl;
		std::cout << "          Measurement was not retrieved" << std::endl;
		std::cout << "-------------------------------------------------" << std::endl;
		//Measurement was not retrieved
		sMeas.bMeasAvailable = false;
	}

   return sMeas;//return measurement results
}

void Skin::convert_data_2_uint64_array(char* inp_buf_data, uint64_t* outp_sen_data)//convert char array (size 1536) to uint_64 array (size 192)
{
	uint64_t u64help_var;

	for(int i = 0; i < skinc_def::max_sensor_number; i++)
	{
		u64help_var = 0;
		for(int j = 0; j < skinc_def::anz_byte_per_sensor; j++)
		{
			if(j == 1)//Fuer Testmessungen
			{
				std::cout << ( (int (inp_buf_data[(8*i)+j]) )&0xFF) << "  ";
			}

			u64help_var = (( (uint64_t (inp_buf_data[(8*i)+j]) )&0xFF) << (8 * j) ) | u64help_var;//merge 8 Byte into one uint64_t variable
		}
		outp_sen_data[i] = u64help_var;
	}
	std::cout << std::endl;
}

bool Skin::write2ram(uint8_t* start_adr, uint8_t steps, uint8_t number_of_bytes, uint8_t write_data, bool sens_num_change)//writes by the service skin_serv provided data into the RAM of the ECU
{
	//std::cout << "write2ram" << std::endl;

	//only execute if SubDevice is configured and I2C device found
	if ( bSubDeviceConfigured_ & bI2CDeviceFound_ & bsub20found_ )
	{
		int 	iI2CErr = 0;
		char 	cwrite_data = char(write_data);
		int 	imem_adr_wr = (start_adr[0] * 256) + start_adr[1];
		uint8_t uisteps	= steps;
		uint8_t uinumber_of_bytes = number_of_bytes;

		for(int i = 0; i < uinumber_of_bytes; i++)
		{
			iI2CErr = iI2CErr + sub_i2c_write( fd_, skinc_def::I2CAdr, imem_adr_wr, skinc_def::I2C_Mem_Adr, &cwrite_data, skinc_def::write_one_byte);//write data to RAM
			ros::Duration(0.05).sleep();//sleeps for 0.05 seconds (50msec sleep time required for ECU / Microcontroller of ECU is to slow to handle a I2C communication without sleep time)
			imem_adr_wr = imem_adr_wr + uisteps;
		}

		//change variable isensor_number_ if sensor number is changed and delete old end of chain marker
		if((sens_num_change == true) && (write_data == skinc_def::end_of_chain_marker))
		{
			//delete old end of chain marker (new limit = default limit)
			int imem_adr_old_echm;

			imem_adr_old_echm = skinc_def::readadr_hi * 256 + skinc_def::readadr_lo + 3 + 8 * isensor_number_;//address of old end of chain marker
			cwrite_data = char(skinc_def::stat_thres_default);

			iI2CErr = iI2CErr + sub_i2c_write( fd_, skinc_def::I2CAdr, imem_adr_old_echm, skinc_def::I2C_Mem_Adr, &cwrite_data, skinc_def::write_one_byte);//replaces the old end of chain marker with a default value
			//change variable isensor_number_
			isensor_number_ = (imem_adr_wr - 259) / 8;//set sensor number
			//std::cout << "sensor number: " << isensor_number_ << std::endl;
		}
		ros::Duration(0.05).sleep();//sleeps for 0.05 seconds (50msec sleep time required for ECU / Microcontroller of ECU is to slow to handle a I2C communication without sleep time)

		if(iI2CErr == 0)//check if write2ram was successful
		{
			return true;//write2ram was successful
		}
		else
		{
			return false;//write2ram was not successful
		}
	}
	else
	{
		std::cout << "-------------------------------------------------" << std::endl;
		std::cout << "              write2ram failed" << std::endl;
		std::cout << "-------------------------------------------------" << std::endl;
		return false;//write2ram was not successful
	}
}

//dynamic reconfigure routine
void Skin::dyn_rec_callback(com_ecu::MyStuffConfig &config, uint32_t level)
{
	//std::cout << "dynamic reconfigure"  << std::endl;
	/*
	uint8_t mem_adr[2];
	uint8_t wr_data;

	std::cout << "Reconfigure request / num_sens.: "  << config.num_sens << std::endl;
	std::cout << "Reconfigure request / sensor_stat_thres: "  << config.sensor_stat_thres << std::endl;
	std::cout << "Reconfigure request / value_stat_thres: "  << config.value_stat_thres << std::endl;
	std::cout << "Reconfigure request / value_dyn_thres: "  << config.dyn_thres << std::endl;
	std::cout << "Reconfigure request / ram_2_flash: "  << config.ram_2_flash << std::endl;

	//change value dyn_thres
	wr_data = uint8_t(config.dyn_thres);
	mem_adr[0] = 0x00;
	mem_adr[1] = 0xC2;
	write2ram(mem_adr, 0, 1, wr_data, false);

	//change value stat_thres
	wr_data = uint8_t(config.value_stat_thres);

	uint16_t ui16adr_pub;
	ui16adr_pub = skinc_def::readadr_hi * 256 + skinc_def::readadr_lo + 3 + 8 * (config.sensor_stat_thres - 1);//address of sensor

	mem_adr[0] = uint8_t((ui16adr_pub & 0xFF00) >> 8);//convert 16-Bit address into two 8-Bit addresses
	mem_adr[1] = uint8_t(ui16adr_pub & 0xFF);

	write2ram(mem_adr, 0, 1, wr_data, false);

	*/
}

bool Skin::conf_int_challback(com_ecu::com_ecu_serv::Request& req, com_ecu::com_ecu_serv::Response& res)
{
	uint8_t start_adr_zg[2];
	start_adr_zg[0] = req.start_adr_srv[0];
	start_adr_zg[1] = req.start_adr_srv[1];

	res.write_to_skin_successful_srv = write2ram(start_adr_zg, req.steps_srv, req.number_of_bytes_srv, req.write_data_srv, req.sens_num_change_srv);
	return true;
}

// ----------
// -- MAIN --
// ----------
int main(int argc, char **argv)
{
	// ros init
	ros::init(argc, argv, "skin_data_publisher");
	ros::NodeHandle n;

	// create a skin class
	Skin one_skin;
	SKINMEAS sMeas;
	com_ecu::com_ecu_meas msg;//ROS message

	ros::Publisher skin_pub = n.advertise<com_ecu::com_ecu_meas> ("skin_data", 100);
	ros::ServiceServer skin_serv = n.advertiseService("skin_serv", &Skin::conf_int_challback, &one_skin);

	//dynamic reconfigure
	dynamic_reconfigure::Server<com_ecu::MyStuffConfig> srv_dyn_rec;
	dynamic_reconfigure::Server<com_ecu::MyStuffConfig>::CallbackType f = boost::bind(&Skin::dyn_rec_callback, &one_skin, _1, _2);
	srv_dyn_rec.setCallback(f);

	ros::Rate loop_rate(5);

	while (n.ok() && one_skin.binit_skin_ok_)
	{
		//Get one measurement
		sMeas = one_skin.GetOneMeas();

		//only publish if a successful measurement was received
		if (sMeas.bMeasAvailable == true)
		{
			for (int i = 0; i < skinc_def::max_sensor_number; i++)
			{
				msg.sensor_data_msg[i] = sMeas.sensor_data[i];
			}
			msg.header.stamp = ros::Time::now();
			msg.sensor_number_msg = one_skin.isensor_number_;
			msg.readadr_hi_msg = skinc_def::readadr_hi;
			msg.readadr_lo_msg = skinc_def::readadr_lo;
			msg.dyn_thres_cv_msg = sMeas.dyn_thres_cv;
			msg.status_go_msg = !(bool(sMeas.status & 0x01));
			msg.status_err_msg = bool((sMeas.status & 0x02) >> 1);
			skin_pub.publish(msg);

			//std::cout << "publish data" << std::endl;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
