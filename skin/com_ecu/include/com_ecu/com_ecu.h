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

#ifndef COM_ECU_H_
#define COM_ECU_H_

#include <stdint.h>
#include "libsub.h"

#include <com_ecu/com_ecu_serv.h>//service

//dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <com_ecu/MyStuffConfig.h>


//! \brief A namespace with parameters and constants
namespace skinc_def {
	//Define I2C communication parameters
	//! Address of slave (sensor skin)
	const int I2CAdr= 1;
	const int I2C_Mem_Adr = 2;

	//Define Skin Hardware parameters
	//! High-byte of the address which represents the beginning of the sensor data in the RAM
	const uint8_t readadr_hi = 1;
	//! Low-byte of the address which represents the beginning of the sensor data in the RAM
	const uint8_t readadr_lo = 0;

	//address of log files
	//! High-byte of the address which represents the beginnign of the log files in the RAM
	const uint8_t readadr_log_hi = 0x00;
	//! Low-byte of the address which represents the beginnign of the log files in the RAM
	const uint8_t readadr_log_lo = 0xC1;

	//Default values
	//! Default value for dynamic threshold
	const uint8_t stat_thres_default = 10;

	//constants
	//! Maximum number of connectable sensors
	const int max_sensor_number = 192;
	//! Number of bytes per sensor
	const int anz_byte_per_sensor = 8;
	//! I2C communication frequency (standard Mode: 100000Hz)
	const int I2C_communication_frequency = 100000;
	const int write_one_byte = 1;
	//! Value of the end of chain marker
	const int end_of_chain_marker = 254;
}

//! \brief Measurement output
struct SKINMEAS {
	//! This flag is true if a valid measurement is available
	bool   	bMeasAvailable;
	//! Buffer to store sensor data
	uint64_t sensor_data[skinc_def::max_sensor_number];
	//! Value of dynamic threshold
	uint8_t dyn_thres_cv;
	//! Status of skin (Go/NoGo - Bit0, Err/NoErr - Bit1)
	uint8_t status;
};

//! \brief An interface which establishes the communication between the GUIs and the ECU of the sensor skin
/*!
 * The class Skin handles the communication with the sensor skin and the GUIs.
 *
 * It reads the sensor values as well as the log files from the sensor skin
 * and publishes them on the topic "skin_data". Apart from that the class also
 * writes data in the ECU of the skin when a service request is send.
 */
class Skin {
	public:
		//! A constructor to initialize values and to establish the connection with the sensor skin
		/*!
		 * The constructor of the class "Skin" scans all connected USB devices and looks for a SUB20 device
		 * with connection to a sensor skin. If a SUB20 device with a connection to a sensor skin is found
		 * the number of connected sensor elements will be determine. To determin the number of sensor elements the
		 * constructor reads the value "Limit" of each sensor element until the end of chain marker (Limit == 254)
		 * is found. This marker is an indicator for the first unused sensor element address.
		 */
		Skin();

		//! A destructor to reset values and to close the opened sub20 device
		~Skin();

		////////////
		//Methodes:
		////////////

		//! Read one SKINMEAS measurement
		/*!
		 * The function "GetOneMeas" reads the data of the connected sensor elements as well as the log files
		 * from the RAM of the sensor skin.
		 *
		 * To read data from the RAM of the sensor skin a special communication pattern has to be used.
		 *
		 * Communication pattern:
		 * 		1.) Memory address write
		 * 			Start|Slave Adr. (Bit1-7) + Bit0 = 0|Mem. Addr. MSB|Mem. Addr. LSB|Stop
		 *
		 * 		2.) Wait for 50ms
		 *
		 * 		3.) Data read
		 * 			Start|Slave Adr. (Bit1-7) + Bit0 = 1|Data[0]|Data[1]|... |Stop
		 */
		SKINMEAS GetOneMeas(void);

		//! Configuration interface callback
		/*!
		 * This function calls the function "write2ram".
		 *
		 * \sa write2ram()
		 */
		bool conf_int_challback(com_ecu::com_ecu_serv::Request& req, com_ecu::com_ecu_serv::Response& res);

		//! Dynamic reconfigure challback
		void dyn_rec_callback(com_ecu::MyStuffConfig &config, uint32_t level);

		////////////
		//Variables
		////////////

		//! Number of sensor elements connected to the ECU of the sensor skin
		int isensor_number_;
		//! Skin is successfully  initialized
		bool binit_skin_ok_;

	private:
		////////////
		//Methodes:
		////////////

		//! Write the provided data into the RAM of the sensor skin
		/*!
		 * "write2ram" writes in a loop the value "write_data" in the memory address "start_adr" and increments
		 * the memory address "start_adr" by "steps". This loop is repeated "number_of_bytes" times. If the flag
		 * "sens_num_change" is true and "write_data" equals 254 the function also deletes the old end of chain marker
		 * and updates the number of sensor elements.
		 *
		 * \param start_adr Memory address of first byte in which the function writes a value
		 * \param steps The function will increment the address "start_adr" (a copy of "start_adr") by "steps" after one "write to RAM" transaction
		 * \param number_of_bytes Number of bytes which the function should write into the RAM
		 * \param write_data Data which should be written into the RAM
		 * \param sens_num_change If "sens_num_change" is true and "write_data" equals 254 the node "com_ecu" will delete the old end of chain marker
		 *
		 */
		bool write2ram(uint8_t* start_adr, uint8_t steps, uint8_t number_of_bytes, uint8_t write_data, bool sens_num_change);

		//! Converts a char array into an uint64_t array
		/*!
		 * The function "convert_data_2_uint64_array" receives a char array of size 1536 and returns an
		 * uint64 array of size 192.
		 *
		 * \param inp_buf_data Received char array
		 * \param outp_sen_data Returned uint64 array
		 */
		void convert_data_2_uint64_array(char* inp_buf_data, uint64_t* outp_sen_data);

		/////////////////////////////
		//Variables for
		//Xdimax Sub20-Device
		////////////////////////////

		//! Handle for subdevice
		sub_handle fd_;
		sub_device subdev_;
		//! Verify if subdevice is opened
		bool bSubDeviceOpen_;
		//! Verify if subdevice is configured
		bool bSubDeviceConfigured_;
		//! Verify if I2C device exists
		bool bI2CDeviceFound_;
		//! Verify if sub20 device with connection to skin exists
		bool bsub20found_;
		//! Serial number of SUB20 device
		std::string strSerial_;

};//class



#endif /* COM_ECU_H_ */
