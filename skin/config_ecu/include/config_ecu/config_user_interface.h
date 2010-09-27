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

#ifndef CONFIG_USER_INTERFACE_H_
#define CONFIG_USER_INTERFACE_H_

//wxwidgets stuff
#include <wx/wx.h>

//commoon Headers
#include <stdint.h>

//ROS Headers
#include "config_ecu/config_skin.h"
#include <ros/ros.h>
#include <ros/time.h>

//! \brief A namespace with IDs
namespace gui_id {
	//ID of menubar
	const int ID_About = 100;

	//ID of Buttons
	const int ID_NUM_SENS_SET = 101;
	const int ID_STAT_THRES_SET = 102;
	const int ID_DYN_THRES_SET = 103;
	const int ID_COM_BTN_RTF = 104;
	const int ID_COM_BTN_IS = 105;

	//ID of Timer
	const int ID_TIMER = 106;
}

//! \brief A namespace with important addresses, replacements ans values
namespace skin_pub {
	/////////////////////////////////
	//addresses of registers in RAM
	/////////////////////////////////

	//! AppData_CommandByte[0] = high-byte, AppData_CommandByte[1] = low-byte
	const uint8_t AppData_CommandByte[2] = {0,0xC0};
	//! AppData_DynLimit[0] = high-byte, AppData_DynLimit[1] = low-byte
	const uint8_t AppData_DynLimit[2] = {0,0xC2};
	//! Address of first sensor / high-byte
	const uint8_t readadr_hi = 1;
	//! Address of first sensor / low-byte
	const uint8_t readadr_lo = 0;


	////////////////
	//replacements
	////////////////

	const bool sens_not_changed = false;
	const bool sens_changed = true;


	////////////////////////////////
	//values for function OnPublish
	////////////////////////////////

	const uint8_t OneByte = 1;
	const uint8_t OneStep = 1;
	const uint8_t NoStep = 0;
	const uint8_t EightSteps = 8;
	const uint8_t EndOfChainMarker = 254;

	//////////////////
	//Default values
	//////////////////

	//! Default value for static threshold
	const uint8_t stat_thres_default = 10;
}

//! \brief A Class to create a configuration user interface
/*!
 * This class will create a user interface which can be used to
 * configurate the parameters of the sensor skin. Apart from that
 * the user interface also provides the possibilities to initialize
 * the sensor skin and to copy the RAM of the sensor skin into the flash.
 * By copying the RAM in the flash the changed parameters are permanently
 * stored in the ECU of the sensor skin.
 *
 * The parameters which can be configured:
 * 		- static threshold of each sensor element
 * 		- dynamic threshold
 * 		- sensor number
 *
 * Additional Buttons
 * 		- "RAM to Flash": copy RAM to Flash
 * 		- "Init Skin": initialize sensor skin
 *
 */
class MyFrame : public wxFrame
{
public:
	//! Constructor to create the layout of the user interface
	MyFrame(const wxString& title, const wxPoint& pos, const wxSize& size);

	////////////////////////////
	//functions of menubar
	////////////////////////////

private:
	//! Close the application
    void OnQuit(wxCommandEvent& event);
    //! Show a Dialog with information about the user interface
    void OnAbout(wxCommandEvent& event);


    ///////////////////////////////////////////////////
    //Functions that are accessed at a button click
    ///////////////////////////////////////////////////

    //! Copy RAM to Flash
    void OnRamToFlash(wxCommandEvent& event);
    //! Set number of sensors
    void OnNumSensSet(wxCommandEvent& event);
    //! Set static threshold
    void OnStatThresSet(wxCommandEvent& event);
    //! Set dynamic threshold
    void OnDynThresSet(wxCommandEvent& event);
    //! Initialize sensor skin
    void OnInitSkin(wxCommandEvent& event);


    //////////////////////////
    //common functions
    //////////////////////////

    //! Trigger a message callback
    /*!
     * The function "subTopic" is called every 250msec by a wxTimer.
     *
     * The main function of "subTopic" is to trigger a message challback. Apart from that
     * "subTopic" also checks if the publisher node "com_ecu" is online.
     */
    void subTopic(wxTimerEvent& evt);

    //! Show a dialog with passed text
    /*!
     * The function "ShowDialog" shows depending on the result of a ROS service different dialog windows.
     *
     * 	1. publisher node is online and parameters were changed successfully: show passed text (wxstr)
     * 	2. publisher node is online and parameters were not changed successfully: show text "Change of skin values failed"
     *  3. else: show text "Subscriber node (com_ecu) is offline"
     *
     *
     * \param wxstr The text which is shown in the dialog if subscriber node is online and "successfullychanged" equals true
     * \param successfullychanged Flag which indicates weather the change of a parameter was successfully or not
     */
    void ShowDialog(wxString wxstr, bool successfullychanged);


    ////////////////////
    //Variables
    ////////////////////

    //Menu Bar

    //! A MenuBar object
    wxMenuBar *menubar;
    //! A Menu object
    wxMenu *file;

    //Text Windows

    //! Text window: number of sensors - new value
    wxTextCtrl *twindow_num_sens_nv;
    //! Text window: static threshold - from SE-Nr.
    wxTextCtrl *twindow_stat_thres_fs;
    //! Text window: static threshold - to SE-Nr.
    wxTextCtrl *twindow_stat_thres_ts;
    //! Text window: static threshold - new value
    wxTextCtrl *twindow_stat_thres_nv;
    //! Text window: dynamic threshold - current value
    wxTextCtrl *twindow_dyn_thres_cv;
    //! Text window: dynamic threshold - new value
    wxTextCtrl *twindow_dyn_thres_nv;

    //Timer
    //! A wxTimer to trigger message callbacks
    /*!
     * The timer "timer_" is used to call the function "subTopic" every 250msec
     *
     * \sa subTopic()
     */
    wxTimer *timer_;

    /////////////////////////////////
    //ROS publisher / Subscriber
    /////////////////////////////////

    //! An object of class SkinConfig
    SkinConfig skin_config_;

    //! Flag for temporary storage of return value from OnRequestChanges
    /*!
     * \sa ShowDialog(), OnRequestChanges()
     */
    bool bchange;

};






#endif /* CONFIG_USER_INTERFACE_H_ */
