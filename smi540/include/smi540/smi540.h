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

//\Author Lukas Marti, Bosch LLC
#ifndef __SMI540__
#define __SMI540__

#include <libsub.h> // sub20 device

//Define SMI540 specifics
namespace cmd540 {
	//Define MM5 commands
	static char chRD_ACT_DATA_64[4] = {0x41, 0x99, 0x00, 0x5C};
	static char chTRIGGER_RESET[4]  = {0x61, 0x03, 0x00, 0xE9};
	//Sepecify MM5 Scale Factor - note: Pos is slightly differen from negative
	const double fSFACC_inv 	= 6667;
	const double fSFGYRO_inv 	= 175;
	//Define enumerated sensor type
	enum eSensorType {eACCEL, eGYRO};
	//Define default sampling rate
	const double  dDEFAULT_RATE_Hz = 20;
}

//Define measurement output
struct ONESMI540MEAS {
	bool   		bMeasAvailable; //indicates true if a valid measurement is available
	double 		dRateZ;
	double 		dAccX;
	double 		dAccY;
	ros::Time 	dtomeas; //time tag measurement immediately in case of other delays
};


//! \brief A Class to interface to the SMI540 over SPI
/*!
 * This class interfaces with the BOSCH SMI 530/540 over SPI using an
 * XDIMAX Sub20 interface device.
 */
class Smi540 {
  public:

  //! Constructor initializes Sub20 device and BMA180ies
  /*!
   * Detects connected Sub20 devices and SMI530/540. Initializes
   * the Sub20 device.
   */
  Smi540();
  //! Destructor closing all open connections
  /*!
   * Closes open SPI connections and disables the Sub20 device.
   */
  ~Smi540();

  //! Polls one measurement from the SMI 530/540
  /*!
   * Sends a measurement request to the SMI530/540 sensor and extracts
   * x/y accel measurement, plus a z-gyro measurement
   */
  ONESMI540MEAS GetOneMeas(void);

  private:
    //subdevice handle
    sub_handle fd;
    //status flag indicating if subdevice has been opened
    bool bSubDeviceOpen;
    //status flag indicating if SPI configuration for SMI530/540 has been stored
    bool bSubDeviceConfigured;
    //Serial number of subdevice
    std::string strSerial;
    /// Converts BMA180 formatted data into double
    double mm5data_to_int(char, char, cmd540::eSensorType);
}; // class

#endif
