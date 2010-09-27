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

#include "config_ecu/config_user_interface.h"
#include "config_ecu/config_skin.h"

//wxwidgets stuff
#include <wx/stattext.h>
#include <wx/statline.h>
#include <wx/checkbox.h>

//common Headers
#include <iostream>
#include <string>
#include <sstream>
#include <math.h>

//TODO:
//	- change the software to handle 192 instead of 191 sensors
//	- Implementation of error handling routines

MyFrame::MyFrame(const wxString& title, const wxPoint& pos, const wxSize& size): wxFrame( NULL, -1, title, pos, size )
{
	//initializations
	skin_config_.bcom_ecu_node_online = false;

	//Timer to execute ros::spinOnce();
	timer_ = new wxTimer(this, gui_id::ID_TIMER);
	timer_->Start(250);//250 = 0.250sec
	Connect(gui_id::ID_TIMER, wxEVT_TIMER, wxTimerEventHandler(MyFrame::subTopic), NULL, this);

	//Menubar
	menubar = new wxMenuBar;
	file = new wxMenu;
	file->Append(gui_id::ID_About, wxT("&About..."));
	file->AppendSeparator();
	file->Append(wxID_EXIT, wxT("&Quit"));
	menubar->Append(file, wxT("&File"));
	SetMenuBar(menubar);

	Connect(gui_id::ID_About, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MyFrame::OnAbout));
	Connect(wxID_EXIT, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MyFrame::OnQuit));

	wxPanel *panel = new wxPanel(this, -1);//main panel

	wxBoxSizer *vbox = new wxBoxSizer(wxVERTICAL);//Layout management

	//headline of Frame
	wxStaticText *headline = new wxStaticText(panel, -1, wxT("Sensor Skin configuration interface"));

	wxFont headline_font = headline->GetFont(); //Set Font of headline
	headline_font.SetWeight(wxBOLD);
	headline_font.SetPointSize(15);
	headline->SetFont(headline_font);

	wxBoxSizer *hbox_headline = new wxBoxSizer(wxHORIZONTAL);
	hbox_headline->Add(headline, 0);

	vbox->Add(hbox_headline, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 10);

	//Line
	wxStaticLine *sline = new wxStaticLine(panel, wxID_ANY);

	wxBoxSizer *hbox_line = new wxBoxSizer(wxHORIZONTAL);
	hbox_line->Add(sline, 1);

	vbox->Add(hbox_line, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 5);

	vbox->Add(-1, 15);//empty space


	/////////////////////////////////
	//Define Fonts

	wxFont heading_font = headline->GetFont();//Set Font of heading
	heading_font.SetWeight(wxBOLD);
	heading_font.SetPointSize(10);

	wxFont normal_font = headline->GetFont();//Set Font of caption
	normal_font.SetWeight(wxLIGHT);
	normal_font.SetPointSize(7);


	/////////////////////////////////
	//Number of sensor elements

	wxBoxSizer *hbox_num_sens = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer *vbox_num_sens_nv = new wxBoxSizer(wxVERTICAL);

	//Statictext <Number of sensor elements>
	wxStaticText *stext_num_sens = new wxStaticText(panel, -1, wxT("Number of sensor elements (1 - 191)"), wxPoint(-1, -1), wxSize(-1, -1));
	stext_num_sens->SetFont(heading_font);
	vbox->Add(stext_num_sens, 0,  wxLEFT, 35);

	vbox->Add(-1, 5);//empty space

	//Statictext <new value>
	wxStaticText *stext_num_sens_nv = new wxStaticText(panel, -1, wxT("new value"), wxPoint(-1, -1), wxSize(-1, -1));
	stext_num_sens_nv->SetFont(normal_font);
	vbox_num_sens_nv->Add(stext_num_sens_nv, 0,  wxALIGN_CENTER_HORIZONTAL);

	//Text window
	twindow_num_sens_nv = new wxTextCtrl(panel, wxID_ANY, wxT(""), wxPoint(-1, -1), wxSize(-1, -1));
	vbox_num_sens_nv->Add(twindow_num_sens_nv, 0);
	hbox_num_sens->Add(vbox_num_sens_nv, 0, wxLEFT, 5);

	//get size of Text Window
	wxSize twindow_size = twindow_num_sens_nv->GetClientSize();

	//Button <set>
	wxButton *btn_num_sens_set = new wxButton(panel, gui_id::ID_NUM_SENS_SET, wxT("set"), wxPoint(-1, -1), wxSize(-1, -1));
	Connect(gui_id::ID_NUM_SENS_SET, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(MyFrame::OnNumSensSet));
	hbox_num_sens->Add(btn_num_sens_set, 0, wxALIGN_BOTTOM | wxLEFT, 5 * 3 + twindow_size.GetWidth() * 2 );

	vbox->Add(hbox_num_sens, 0,  wxLEFT , 35 );


	/////////////////////////////////
	//Static thresholds

	wxBoxSizer *hbox_stat_thres = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer *vbox_stat_thres_fs = new wxBoxSizer(wxVERTICAL);
	wxBoxSizer *vbox_stat_thres_ts = new wxBoxSizer(wxVERTICAL);
	wxBoxSizer *vbox_stat_thres_nv = new wxBoxSizer(wxVERTICAL);

	vbox->Add(-1, 10);//empty space

	//Statictext <Static Thresholds>
	wxStaticText *stext_stat_thres = new wxStaticText(panel, -1, wxT("Static Thresholds"), wxPoint(-1, -1), wxSize(-1, -1));
	stext_stat_thres->SetFont(heading_font);
	vbox->Add(stext_stat_thres, 0,  wxLEFT, 35);

	vbox->Add(-1, 5);//empty space

	//Statictext <from SE-Nr.>
	wxStaticText *stext_stat_thres_fs = new wxStaticText(panel, -1, wxT("from SE-Nr."), wxPoint(-1, -1), wxSize(-1, -1));
	stext_stat_thres_fs->SetFont(normal_font);
	vbox_stat_thres_fs->Add(stext_stat_thres_fs, 0,  wxALIGN_CENTER_HORIZONTAL);

	//Text window
	twindow_stat_thres_fs = new wxTextCtrl(panel, wxID_ANY, wxT(""), wxPoint(-1, -1), wxSize(-1, -1));
	vbox_stat_thres_fs->Add(twindow_stat_thres_fs, 0);
	hbox_stat_thres->Add(vbox_stat_thres_fs, 0, wxLEFT, 5);

	//Statictext <to SE-Nr.>
	wxStaticText *stext_stat_thres_ts = new wxStaticText(panel, -1, wxT("to SE-Nr."), wxPoint(-1, -1), wxSize(-1, -1));
	stext_stat_thres_ts->SetFont(normal_font);
	vbox_stat_thres_ts->Add(stext_stat_thres_ts, 0,  wxALIGN_CENTER_HORIZONTAL);

	//Text window
	twindow_stat_thres_ts = new wxTextCtrl(panel, wxID_ANY, wxT(""), wxPoint(-1, -1), wxSize(-1, -1));
	vbox_stat_thres_ts->Add(twindow_stat_thres_ts, 0);
	hbox_stat_thres->Add(vbox_stat_thres_ts, 0, wxLEFT, 5);

	//Statictext <new value>
	wxStaticText *stext_stat_thres_nv = new wxStaticText(panel, -1, wxT("new value"), wxPoint(-1, -1), wxSize(-1, -1));
	stext_stat_thres_nv->SetFont(normal_font);
	vbox_stat_thres_nv->Add(stext_stat_thres_nv, 0,  wxALIGN_CENTER_HORIZONTAL);

	//Text window
	twindow_stat_thres_nv = new wxTextCtrl(panel, wxID_ANY, wxT(""), wxPoint(-1, -1), wxSize(-1, -1));
	vbox_stat_thres_nv->Add(twindow_stat_thres_nv, 0);
	hbox_stat_thres->Add(vbox_stat_thres_nv, 0, wxLEFT, 5);

	//Button <set>
	wxButton *btn_stat_thres_set = new wxButton(panel, gui_id::ID_STAT_THRES_SET, wxT("set"), wxPoint(-1, -1), wxSize(-1, -1));
	Connect(gui_id::ID_STAT_THRES_SET, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(MyFrame::OnStatThresSet));
	hbox_stat_thres->Add(btn_stat_thres_set, 0, wxALIGN_BOTTOM | wxLEFT, 5 );

	vbox->Add(hbox_stat_thres, 0,  wxLEFT , 35);


	/////////////////////////////////
	//Dynamic thresholds

	wxBoxSizer *hbox_dyn_thres = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer *vbox_dyn_thres_cv = new wxBoxSizer(wxVERTICAL);
	wxBoxSizer *vbox_dyn_thres_nv = new wxBoxSizer(wxVERTICAL);

	vbox->Add(-1, 10);//empty space

	//Statictext <Dynamic Threshold>
	wxStaticText *stext_dyn_thres = new wxStaticText(panel, -1, wxT("Dynamic Threshold"), wxPoint(-1, -1), wxSize(-1, -1));
	stext_dyn_thres->SetFont(heading_font);
	vbox->Add(stext_dyn_thres, 0,  wxLEFT, 35);

	vbox->Add(-1, 5);//empty space

	//Statictext <current value>
	wxStaticText *stext_dyn_thres_cv = new wxStaticText(panel, -1, wxT("current value"), wxPoint(-1, -1), wxSize(-1, -1));
	stext_dyn_thres_cv->SetFont(normal_font);
	vbox_dyn_thres_cv->Add(stext_dyn_thres_cv, 0,  wxALIGN_CENTER_HORIZONTAL);

	//Text window
	twindow_dyn_thres_cv = new wxTextCtrl(panel, wxID_ANY, wxT(""), wxPoint(-1, -1), wxSize(-1, -1));
	vbox_dyn_thres_cv->Add(twindow_dyn_thres_cv, 0);
	hbox_dyn_thres->Add(vbox_dyn_thres_cv, 0, wxLEFT, 5);
	twindow_dyn_thres_cv->SetEditable(false);

	//Statictext <new value>
	wxStaticText *stext_dyn_thres_nv = new wxStaticText(panel, -1, wxT("new value"), wxPoint(-1, -1), wxSize(-1, -1));
	stext_dyn_thres_nv->SetFont(normal_font);
	vbox_dyn_thres_nv->Add(stext_dyn_thres_nv, 0,  wxALIGN_CENTER_HORIZONTAL);

	//Text window
	twindow_dyn_thres_nv = new wxTextCtrl(panel, wxID_ANY, wxT(""), wxPoint(-1, -1), wxSize(-1, -1));
	vbox_dyn_thres_nv->Add(twindow_dyn_thres_nv, 0);
	hbox_dyn_thres->Add(vbox_dyn_thres_nv, 0, wxLEFT, 5);

	//Button <set>
	wxButton *btn_dyn_thres_set = new wxButton(panel, gui_id::ID_DYN_THRES_SET, wxT("set"), wxPoint(-1, -1), wxSize(-1, -1));
	Connect(gui_id::ID_DYN_THRES_SET, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(MyFrame::OnDynThresSet));
	hbox_dyn_thres->Add(btn_dyn_thres_set, 0, wxALIGN_BOTTOM | wxLEFT, 5 * 2 + twindow_size.GetWidth());

	//get size of Button
	wxSize tbutton_size = btn_dyn_thres_set->GetClientSize();

	vbox->Add(hbox_dyn_thres, 0,  wxLEFT , 35);


	/////////////////////////////////
	//Command Buttons

	wxBoxSizer *hbox_com_btn = new wxBoxSizer(wxHORIZONTAL);

	vbox->Add(-1, 10);//empty space

	//Statictext <Command Button>
	wxStaticText *stext_com_btn = new wxStaticText(panel, -1, wxT("Command Buttons"), wxPoint(-1, -1), wxSize(-1, -1));
	stext_com_btn->SetFont(heading_font);
	vbox->Add(stext_com_btn, 0,  wxLEFT, 35);

	vbox->Add(-1, 5);//empty space

	//Button <Ram to flash>
	wxButton *btn_com_btn_rtf = new wxButton(panel, gui_id::ID_COM_BTN_RTF, wxT("Ram to Flash"), wxPoint(-1, -1), wxSize(twindow_size.GetWidth() * 2 + 5, -1));
	Connect(gui_id::ID_COM_BTN_RTF, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(MyFrame::OnRamToFlash));
	hbox_com_btn->Add(btn_com_btn_rtf, 0, wxALIGN_BOTTOM | wxLEFT, 5 );

	//Button <Init Skin>
	wxButton *btn_com_btn_is = new wxButton(panel, gui_id::ID_COM_BTN_IS, wxT("Init Skin"), wxPoint(-1, -1), wxSize(twindow_size.GetWidth() + tbutton_size.GetWidth() + 5, -1));
	Connect(gui_id::ID_COM_BTN_IS, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(MyFrame::OnInitSkin));
	hbox_com_btn->Add(btn_com_btn_is, 0, wxALIGN_BOTTOM | wxLEFT, 5 );

	vbox->Add(hbox_com_btn, 0,  wxLEFT , 35);



	panel->SetSizer(vbox);

	Centre();
}

void MyFrame::subTopic(wxTimerEvent& evt)
{
	ros::spinOnce();
	std::cout << "subTopic" << " \n";

	//check if com_ecu node is online
	//conditions:
	//sensor number != 0 (valid sensor number: 0 < value < 192)
	//dynamic threshold != 255 (valid threshold: 0 < value < 129)
	//time stamp of last message is not older than (ceil(float(skin_config_.ui8sensor_number_) / 32.0) -1 ) * 0.1 + 0.25

	//Time between two calls of function GetOneMeas (node com_ecu)
	//1 Sens - 32 Sens : max 0.21sec		- limit : 0.25
	//33 Sens - 64 Sens : max 0.26sec		- limit : 0.35
	//65 Sens - 96 Sens : max 0.35sec		- limit : 0.45
	//97 Sens - 128 Sens : max 0.46sec		- limit : 0.55
	//129 Sens - 160 Sens : max 0.55sec		- limit : 0.65
	//161 Sens - 192 Sens : max 0.65sec		- limit : 0.75
	//determination of the limit: (ceil(float(skin_config_.ui8sensor_number_) / 32.0) -1 ) * 0.1 + 0.25


	ros::Time dcurrent_time = ros::Time::now();
	if( (skin_config_.ui8sensor_number_ == 0) || (skin_config_.ui8dyn_thres_currentvalue_ == 255) || (( dcurrent_time > (skin_config_.dtime_subscription_ + ros::Duration((ceil(float(skin_config_.ui8sensor_number_) / 32.0) -1 ) * 0.1 + 0.25)))))
	{
		//com_ecu node online
		skin_config_.bcom_ecu_node_online = false;//clear Flag
		twindow_dyn_thres_cv->SetValue(wxT(""));//clear text window twindow_dyn_thres_cv
	}
	else
	{
		//com_ecu node offline
		skin_config_.bcom_ecu_node_online = true;//set Flag
		twindow_dyn_thres_cv->SetValue(wxString::Format(wxT("%i"),skin_config_.ui8dyn_thres_currentvalue_));//write current threshold in text window twindow_dyn_thres_cv
	}

	if (!ros::ok())
	{
		Close();
	}
}

void MyFrame::ShowDialog(wxString wxstr, bool successfullychanged)//show a dialog with passed text
{
	std::cout << skin_config_.bcom_ecu_node_online << std::endl;
	if( (skin_config_.bcom_ecu_node_online == true) && (successfullychanged == true) )//show passed text if subscriber node is online
	{
		wxMessageDialog *dial_info = new wxMessageDialog(NULL, wxstr, wxT("Info"), wxOK);
		dial_info->ShowModal();
	}
	else if( (skin_config_.bcom_ecu_node_online == true) && (successfullychanged == false) )//publisher node online and write into RAM failed
	{
		wxMessageDialog *dial_info = new wxMessageDialog(NULL, wxT("Change of skin values failed"), wxT("Info"), wxOK | wxICON_ERROR);
		dial_info->ShowModal();
	}
	else//show error dialog if subscriber node is offline
	{
		wxMessageDialog *dial_info = new wxMessageDialog(NULL, wxT("Subscriber node (com_ecu) is offline"), wxT("Info"), wxOK | wxICON_ERROR);
		dial_info->ShowModal();
	}
}

void MyFrame::OnQuit(wxCommandEvent& WXUNUSED(event))
{
	Close(true);//close gui
}

void MyFrame::OnAbout(wxCommandEvent& WXUNUSED(event))
{
    wxMessageBox( wxT(".....      text  text  text     ....."), wxT("Sensor Skin"), wxICON_INFORMATION, this );
}

void MyFrame::OnRamToFlash(wxCommandEvent& event)
{
    //copy Ram to Flash
	bchange = skin_config_.OnRequestChanges(skin_pub::AppData_CommandByte, skin_pub::NoStep, skin_pub::OneByte, 4, skin_pub::sens_not_changed);//4: command to write RAM to Flash
	ShowDialog(wxT("RAM was written into Flash"), bchange);
}

void MyFrame::OnInitSkin(wxCommandEvent& event)
{
    //Initialize skin
	bchange = skin_config_.OnRequestChanges(skin_pub::AppData_CommandByte, skin_pub::NoStep, skin_pub::OneByte, 1, skin_pub::sens_not_changed);//1: command to init skin
	ShowDialog(wxT("Skin is initialized"), bchange);
}

void MyFrame::OnNumSensSet(wxCommandEvent& event)
{
	//set number of sensors
	wxString num_sens_nv_wxstring = twindow_num_sens_nv->GetValue();//read value from text window
	long lnum_sens_nv;
	bool bvalid_number = false;

	bvalid_number = num_sens_nv_wxstring.ToLong(&lnum_sens_nv, 10);//check if value is a number

	if((lnum_sens_nv < 1)||(lnum_sens_nv > 191))//check if value < 192 and > 0
	{
		bvalid_number = false;
	}

	if(bvalid_number)//if value is valid: send data
	{
		uint8_t ui8adr_pub[2];
		uint16_t ui16adr_pub;

		ui16adr_pub = skin_pub::readadr_hi * 256 + skin_pub::readadr_lo + 3 + 8 * lnum_sens_nv;//address of new end of chain marker

		ui8adr_pub[0] = uint8_t((ui16adr_pub & 0xFF00) >> 8);//convert 16-Bit address into two 8-Bit addresses
		ui8adr_pub[1] = uint8_t(ui16adr_pub & 0xFF);

		//set new end of chain marker
		bchange = skin_config_.OnRequestChanges(ui8adr_pub, skin_pub::NoStep, skin_pub::OneByte, skin_pub::EndOfChainMarker, skin_pub::sens_changed);

		ros::Duration(0.05).sleep();//sleeps for 0.05 seconds

		//initialize skin
		bchange = skin_config_.OnRequestChanges(skin_pub::AppData_CommandByte, skin_pub::NoStep, skin_pub::OneByte, 1, skin_pub::sens_not_changed);//1: command to init skin
		ShowDialog(wxT("Number of sensors was changed to ") + num_sens_nv_wxstring + wxT("\n\nTo save the changes permanently the RAM has to be written into the Flash"), bchange);
		this->SetFocus();
	}
	else//value is not valid
	{
		ShowDialog(wxT("Please insert a valid number in the text window\n\nvalid number: 0 < number fo sensors < 192"), true);
		twindow_num_sens_nv->SetFocus();
	}

	twindow_num_sens_nv->Clear();//clear window


}

void MyFrame::OnStatThresSet(wxCommandEvent& event)
{
    //set static thresholds
	wxString stat_thres_fs_wxstring = twindow_stat_thres_fs->GetValue();//read value from text window
	wxString stat_thres_ts_wxstring = twindow_stat_thres_ts->GetValue();//read value from text window
	wxString stat_thres_nv_wxstring = twindow_stat_thres_nv->GetValue();//read value from text window

	long lstat_thres_fs;
	long lstat_thres_ts;
	long lstat_thres_nv;

	bool bvalid_number = false;

	bvalid_number = stat_thres_fs_wxstring.ToLong(&lstat_thres_fs, 10) && stat_thres_ts_wxstring.ToLong(&lstat_thres_ts, 10) && stat_thres_nv_wxstring.ToLong(&lstat_thres_nv, 10);//check if values are numbers

	//valid values:
	// 0 < from SE-Nr. <= sensor number
	// 0 < to SE-Nr. < sensor number
	// 0 < new value < 251
	// from SE-Nr. <= to SE-Nr.

	if( (lstat_thres_fs < 1)||(lstat_thres_fs > skin_config_.ui8sensor_number_)||(lstat_thres_ts < 1)||(lstat_thres_ts > skin_config_.ui8sensor_number_)||(lstat_thres_nv < 1)||(lstat_thres_nv > 250)||(lstat_thres_fs > lstat_thres_ts) )//check if values are correct
	{
		bvalid_number = false;
	}

	if(bvalid_number)//if value is valid: send data
	{
		uint8_t ui8adr_pub[2];
		uint16_t ui16adr_pub;

		ui16adr_pub = skin_pub::readadr_hi * 256 + skin_pub::readadr_lo + 3 + 8 * (lstat_thres_fs - 1);//address of first sensor

		ui8adr_pub[0] = uint8_t((ui16adr_pub & 0xFF00) >> 8);//convert 16-Bit address into two 8-Bit addresses
		ui8adr_pub[1] = uint8_t(ui16adr_pub & 0xFF);

		//set new static thresholds
		bchange = skin_config_.OnRequestChanges(ui8adr_pub, skin_pub::EightSteps, uint8_t(lstat_thres_ts - lstat_thres_fs + 1), uint8_t(lstat_thres_nv), skin_pub::sens_not_changed);

		ShowDialog(wxT("Static threshold was changed \n\nmodifications:\n\t from SE-Nr. = ") + stat_thres_fs_wxstring + wxT("\n\t to SE-Nr.   = ") + stat_thres_ts_wxstring + wxT("\n\t new value   = ") + stat_thres_nv_wxstring + wxT("\n\nTo save the changes permanently the RAM has to be written into the Flash"), bchange);
		this->SetFocus();
	}
	else
	{
		ShowDialog(wxT("Please insert a valid number of sensors\n\nvalid values:\n\t 0 < from SE-Nr. <= sens. numb.\n\t 0 < to SE-Nr. <= sens. numb.\n\t 0 < new value < 251\n\t from SE-Nr. <= to SE-Nr."), true);
		twindow_stat_thres_fs->SetFocus();
	}

	twindow_stat_thres_fs->Clear();//clear window
	twindow_stat_thres_ts->Clear();//clear window
	twindow_stat_thres_nv->Clear();//clear window
}

void MyFrame::OnDynThresSet(wxCommandEvent& event)
{
    //set dynamic threshold
	wxString dyn_thres_nv_wxstring = twindow_dyn_thres_nv->GetValue();//read value from text window
	long ldyn_thres_nv;
	bool bvalid_number = false;

	bvalid_number = dyn_thres_nv_wxstring.ToLong(&ldyn_thres_nv, 10);//check if value is a number

	if((ldyn_thres_nv < 1)||(ldyn_thres_nv > 128))//check if value < 129 and > 0
	{
		bvalid_number = false;
	}

	if(bvalid_number)//if value is valid: send data
	{
		//set new dynamic thresholds
		bchange = skin_config_.OnRequestChanges(skin_pub::AppData_DynLimit, skin_pub::NoStep, skin_pub::OneByte, uint8_t(ldyn_thres_nv), skin_pub::sens_not_changed);

		ShowDialog(wxT("Dynamic threshold was changed to ") + dyn_thres_nv_wxstring + wxT("\n\nTo save the changes permanently the RAM has to be written into the Flash"), bchange);
		this->SetFocus();
	}
	else
	{
		ShowDialog(wxT("Please insert a valid number in the text window\n\nvalid number: 0 < dynamic threshold < 129"), true);
		twindow_dyn_thres_nv->SetFocus();
	}

	twindow_dyn_thres_nv->Clear();//clear window
}






