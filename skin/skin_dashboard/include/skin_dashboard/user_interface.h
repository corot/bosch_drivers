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

#ifndef USER_INTERFACE_H_
#define USER_INTERFACE_H_

//wxwidgets stuff
#include <wx/wx.h>
#include <wx/grid.h>

//ROS Headers
#include <ros/ros.h>

#include "skin_dashboard/skin.h"
#include <stdint.h>

//! \brief A namespace with IDs
namespace skin_id {
	const int ID_About = 100;
	const int ID_HEX = 101;
	//! ID of the timer which triggers message callbacks
	const int ID_TIMER = 102;
}

//! \brief A Class to create a user interface which displays sensor data
/*!
 * This class will create a user interface which displays the sensor data and
 * some information derived from the sensor data.
 *
 * Displayed information:
 * 		- Sensor data (Stat, X, Ini, Limit, U1, U2, U3, Dyn)
 * 		- Status display (Go/NoGo, Err/NoErr)
 * 		- A static approximation is displayed by "X Col.Bar".
 * 		  If the sensor recognizes an object the color in the colorbar turns to red (el. conductor)
 * 		  or blue (plastic materials)
 * 		- The ErrorBar displays 5 possible errors
 * 		   		A: U2 > 32
 *				B: U3 < 16
 *				C: U3-U1 < 16
 *				D: Err_IO = 1
 *				E: ErrTMS = 1
 */
class MyFrame : public wxFrame
{
public:
	//! Constructor to create the layout of the user interface
    MyFrame(const wxString& title, const wxPoint& pos, const wxSize& size);

private:

    ///////////////
    //functions
    ///////////////

    //! Close the application
    void OnQuit(wxCommandEvent& event);
    //! Show a Dialog with information about the user interface
    void OnAbout(wxCommandEvent& event);
    //! Set the formatting of the displayed values
    void OnCheck(wxCommandEvent& event);

    //! Convert a uint32_t value into a wxString
    /*!
     * The function "hexadezimal" converts a decimal value into a hex-string.
     *
     * \param conv_value Decimal value which should be converted into a hex-string
     * \return Provided data as hex-string
     */
    wxString hexadezimal(uint32_t conv_value);

    //! Displays data on the GUI
    /*!
     * The function "publish_data_2_gui" handles the display of the sensor data.
     * It updates the grid which shows the raw sensor data and derives additional
     * information from the raw sensor data which are shown in the status bar, the
     * X Col.Bar and the  ErrorBar.
     */
    void publish_data_2_gui();

    //! Trigger a message callback
    /*!
     * The function "gridUpdate" is called every 250msec by a wxTimer.
     *
     * The function of "gridUpdate" is to trigger a message challback and to call the function
     * "publish_data_2_gui" which displays the sensor data on the GUI.
     */
    void gridUpdate(wxTimerEvent& evt);


    ////////////////////
    //Variables
    ////////////////////

    //! A MenuBar object
    wxMenuBar *menubar;
    //! A Menu object
    wxMenu *file;
    //! A Grid object
    wxGrid *grid;

    //! A wxTimer to trigger message callbacks
    /*!
     * The timer "timer_" is used to call the function "gridUpdate" every 250msec
     *
     * \sa gridUpdate()
     */
    wxTimer *timer_;


    //status
    //! A static text to display the Go/NoGo part of the status bar
    wxStaticText *text_go_nogo;
    //! A static text to display the Err/NoErr part of the status bar
    wxStaticText *text_err_noerr;

    //! An object of class Skin
    Skin skin_dat_;

    //Data Evaluation
    //! if "bhex_" equals true the data in the GUI are formatted as hex-strings else as decimal values
    bool bhex_;
    //! Variable to store the error conditions of one sensor
    bool berrcond_[5];
 };

#endif /* USER_INTERFACE_H_ */
