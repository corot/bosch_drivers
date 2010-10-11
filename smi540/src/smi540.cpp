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

#include <iostream>
#include <string>
#include <ros/ros.h>
#include "libsub.h"
#include "smi540/smi540.h"
#include "smi540/smi540meas.h"
#include <sstream>
#include <ros/time.h>


Smi540::Smi540():
    bSubDeviceOpen (false),
    bSubDeviceConfigured (false),
    strSerial ("         ")
{
  int iSpiErr; //Error code for Sub20 API
  int iSPI_cfg_set; //Set SPI config
  int iSPI_cfg_get; //Get SPI config

  std::cout << "---opening SUB device---" << " \n";
  //ASSUMPTIONS:
  //  At this point, we only allow one subdevice to be connected. This is driven by the rational:
  //  a) Each Sub20 device needs a different SPI configuration depending on the sensor connected
  //  b) If each subdevice is scanned for sensors being connected, it would require reconfiguration
  //     of the SPI interface for the respective sensor, potentially disabling a previously connfigured
  //     SPI device.
  // SOLUTION: Depending on the SUB20 configuration, define a configuration file specifying which SUB20
  //           interface box connects to which sensor
  fd = sub_open(0);
  // on success, sub_open returns non-zero handle
  if( fd==0 ) {
    //sub_open was not successful
    ROS_INFO("ERROR - Sub20 could not be opened: %s ", sub_strerror(sub_errno) );
  }
  else {
    //Subdevice successfully opened
    bSubDeviceOpen = true;
    std::cout << "Device Handle   : " << fd << " \n";
    //Read Serial number of subdevice
    if( sub_get_serial_number(fd, const_cast<char*>(strSerial.c_str()), strSerial.size() ) >= 0 )
      std::cout << "Serial Number   : " << strSerial << std::endl;
      std::cout << "---Initializing SPI Interface---" << std::endl;
      /* Read current SPI configuration */
      iSpiErr = sub_spi_config( fd, 0, &iSPI_cfg_get );
      std::cout << "Dev Sub config  : " << iSPI_cfg_get << " \n";
      //Important: The SPI interface does not work properly at higher frequencies
      iSPI_cfg_set = SPI_ENABLE|SPI_CPOL_RISE|SPI_SMPL_SETUP|SPI_MSB_FIRST|SPI_CLK_1MHZ;
      /* Configure SPI */
      iSpiErr = sub_spi_config( fd, iSPI_cfg_set, 0 );
      /* Read current SPI configuration */
      iSpiErr = sub_spi_config( fd, 0, &iSPI_cfg_get );

      //verify if sub20 device has accepted the configuration
      if (iSPI_cfg_get == iSPI_cfg_set ) {
        std::cout<< "Configuration   : " << iSPI_cfg_set << " successfully stored \n";
        //Subdevice has been configured successfully
        bSubDeviceConfigured = true;
      }
      else {
        std::cout<< "ERROR - Configuration :" << iSPI_cfg_set << " not accept by device \n";
        //Subdevice could not be configured
        bSubDeviceConfigured = false;
      }

      //only execute if SubDevice has accepted configuration
      if ( bSubDeviceConfigured ) {
        std::cout<< "Configuring SMI530 device \n";
          // SS_CONF(0,SS_LO):
          // SS_N Chipselect 0;
          // SS_LO SS goes low and stays low during entire transfer, after that it goes high
          iSpiErr = sub_spi_transfer( fd, cmd540::chTRIGGER_RESET, 0, 4, SS_CONF(0,SS_LO) );
      };
   };
};

Smi540::~Smi540()
{
  int iSpiErr;
  // Disable SPI
  iSpiErr = sub_spi_config( fd, 0, 0 );
  // Close USB device
  sub_close( fd );
  //Set status
  bSubDeviceOpen       = false;
  bSubDeviceConfigured = false;
};

ONESMI540MEAS Smi540::GetOneMeas(void)
{
  char             chMM5_rx[8]; //Containing
  int              iSpiErr;
  ONESMI540MEAS    sMeas;
  unsigned short   smi540health;

  //initialize measurement with no data -> 0
  sMeas.dRateZ   = 0;
  sMeas.dAccX    = 0;
  sMeas.dAccY    = 0;
  sMeas.bMeasAvailable = false;

  //only execute if SubDevice is configured ]
  if ( bSubDeviceConfigured )
  {
    // SS_N Chipselect 0;
    // SS_LO SS goes low and stays low during entire transfer, after that it goes high
    iSpiErr = sub_spi_transfer( fd, cmd540::chRD_ACT_DATA_64, 0, 4, SS_CONF(0,SS_LO) );
    iSpiErr = sub_spi_transfer( fd, 0, chMM5_rx, 8, SS_CONF(0,SS_LO) );

    if (iSpiErr == 0 ) {
      //Convert the response into scaled sensor measurements
      sMeas.dRateZ          = mm5data_to_int(chMM5_rx[5], chMM5_rx[6], cmd540::eGYRO);
      sMeas.dAccX           = mm5data_to_int(chMM5_rx[3], chMM5_rx[4], cmd540::eACCEL);
      sMeas.dAccY           = mm5data_to_int(chMM5_rx[1], chMM5_rx[2], cmd540::eACCEL);
      sMeas.bMeasAvailable 	= true;
      sMeas.dtomeas         = ros::Time::now();

      //Extract status of channels
      smi540health = (unsigned short)(chMM5_rx[0]&0x23);
    }
    else {
      throw std::string ("SPI transfer error");
      //Measurement was not retrieved
      sMeas.bMeasAvailable = false;
    }
  }
  else {
    //verify that a subdevice is connected
	throw std::string ("No SubDevice connected OR access rights to USB not given");
    //Measurement was not retrieved
    sMeas.bMeasAvailable = false;
  }
  return (sMeas);
}

double Smi540::mm5data_to_int(char chMSB, char chLSB, cmd540::eSensorType eSensor)
{
  short 	s16int;			//2 byte int to build message
  double 	dSensorMeas; 	//Scaled sensor measurement
  //verify if pos or neg
  if ( (chMSB & 0x80) == 0 ) {
    //positive number
    s16int = (short) ((((unsigned short)chMSB)&0xFF)<<8)+(((unsigned short)chLSB)&0xFF);
  }
  else {
    //negative number
    //set MSB (sign) to zero and build 2 complement unsigned
    s16int = (short)(((((unsigned short)chMSB)&0x7F)<<8)+(((unsigned short)chLSB)&0xFF)-32768);
  }

  //Convert data with the respective Scale Factor
  if (eSensor == cmd540::eACCEL) {
    dSensorMeas = ((double)s16int)/cmd540::fSFACC_inv;
  }
  else {
    dSensorMeas = ((double)s16int)/cmd540::fSFGYRO_inv;
  }
  return (dSensorMeas);
}

// ----------
// -- MAIN --
// ----------
int main(int argc, char **argv)
{
  Smi540 		       one_smi540;
  ONESMI540MEAS        sOneMeas;
  smi540::smi540meas   msg;
  double               dRate_Hz;

  ros::init(argc, argv, "smi540");
  ros::NodeHandle n;
  ros::Publisher smi540_pub = n.advertise<smi540::smi540meas>("smi540", 100);

  if (n.getParam("/drivers/smi540/rate_Hz", dRate_Hz) == false ) {
	dRate_Hz = cmd540::dDEFAULT_RATE_Hz;
    ROS_INFO("Using Default Sensor Sampling Rate of %f [Hz]", dRate_Hz);
  };

  ros::Rate loop_rate(dRate_Hz);

  while (n.ok())
  {
    //try to poll one measurement from sensor
	try {
      sOneMeas = one_smi540.GetOneMeas();
	}
    catch ( std::string strType ) {
      std::cout << " An exception occurred: " << strType << std::endl;
      std::exit(1);
    }

    //only publish if a successful measurement was received
    if ( sOneMeas.bMeasAvailable == true ) {
      msg.fAcclX = sOneMeas.dAccX;
      msg.fAcclY = sOneMeas.dAccY;
      msg.fRateZ = sOneMeas.dRateZ;
      msg.header.stamp = sOneMeas.dtomeas;
      smi540_pub.publish(msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
