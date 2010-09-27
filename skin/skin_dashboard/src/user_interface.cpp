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

//wxwidgets stuff
#include <wx/stattext.h>
#include <wx/statline.h>
#include <wx/checkbox.h>

//common stuff
#include "skin_dashboard/user_interface.h"
#include "skin_dashboard/skin.h"

#include <math.h>
#include <iostream>

//TODO:
//	- change the software to handle 192 instead of 191 sensors
//	- Implementation of error handling routines

MyFrame::MyFrame(const wxString& title, const wxPoint& pos, const wxSize& size): wxFrame( NULL, -1, title, pos, size )
{
	//Timer to execute ros::spinOnce();
	timer_ = new wxTimer(this, skin_id::ID_TIMER);
	timer_->Start(250);
	Connect(skin_id::ID_TIMER, wxEVT_TIMER, wxTimerEventHandler(MyFrame::gridUpdate), NULL, this);

	//Menubar
	menubar = new wxMenuBar;
	file = new wxMenu;
	file->Append(skin_id::ID_About, wxT("&About..."));
	file->AppendSeparator();
	file->Append(wxID_EXIT, wxT("&Quit"));
	menubar->Append(file, wxT("&File"));
	SetMenuBar(menubar);

	Connect(skin_id::ID_About, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MyFrame::OnAbout));
	Connect(wxID_EXIT, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MyFrame::OnQuit));


	wxPanel *panel = new wxPanel(this, -1);//main panel


	//headline of Frame
	wxStaticText *headline = new wxStaticText(panel, -1, wxT("Sensor Skin Monitor"),wxPoint(15,15));

	wxFont headline_font = headline->GetFont(); //Set Font of headline
	headline_font.SetWeight(wxBOLD);
	headline_font.SetPointSize(15);
	headline->SetFont(headline_font);

	//Line
	wxStaticLine *sl1 = new wxStaticLine(panel, wxID_ANY, wxPoint(15,40), wxSize(730,1));

	//Creates grid and sets size of cells
	const wxString col_title_[18] = {wxT("Adr"),wxT("Stat"),wxT("X"),wxT("Ini"),wxT("Limit"),wxT("U1"),wxT("U2"),wxT("U3"),wxT("Dyn"),wxT(""),wxT("SE-NR."),wxT(""),wxT("A"),wxT("B"),wxT("C"),wxT("D"),wxT("E")};
	grid = new wxGrid(panel, wxID_ANY, wxPoint(30,85), wxSize(699, 770));
	grid->CreateGrid(skind_def::max_sensor_number, 16);

	grid->SetColLabelSize(30);
	grid->SetRowLabelSize(50);

	wxColour label_background_colour;
	label_background_colour = grid->GetLabelBackgroundColour();

	for(int i = 0; i < skind_def::max_sensor_number ; i++)//Sets Row Size + Sets backgroundcolour of column 8, 9 and  10 + set aligns position of text
	{
		grid->SetRowSize(i, 20);
		grid->SetCellBackgroundColour(i, 8, label_background_colour);
		grid->SetCellBackgroundColour(i, 10, label_background_colour);
		grid->SetCellBackgroundColour(i, 9, *wxWHITE);
		for(int j = 0; j < 16; j++)
		{
			grid->SetCellAlignment(i, j, wxALIGN_CENTRE, wxALIGN_CENTRE);
		}
	}

	for(int j = 0; j < 8 ; j++)//Sets Column Size (column 0-7)
	{
		grid->SetColSize(j, 50);
	}

	grid->SetColSize(8, 25);//Sets Column Size (column 8-10)
	grid->SetColSize(9, 80);
	grid->SetColSize(10, 25);

	for(int i = 11 ; i < 16 ; i++)//Sets Column Size (column 11-15)
	{
		grid->SetColSize(i, 20);
	}

	//sets Value of column-labels
	for(int k = 0; k < 16; k++)
	{
		grid->SetColLabelValue((k),col_title_[k+1]);
	}

	//Inserts sensor number in the  9th column, change text colour, set background colour
	for(int i = 0; i < skin_dat_.u8sensor_number_ ; i++)
	{
		grid->SetCellValue(i,9,wxString::Format(wxT("%i"),i+1));
		grid->SetCellTextColour(i,9,wxColour(255,255,0));
	}

	//Disables user manipulations
	grid->EnableEditing(false);
	grid->EnableDragRowSize(false);
	grid->EnableDragColSize(false);

	//Sets the number of pixels per scroll increment
	grid->SetScrollLineY(20);

	//Inserts the text of the first cell
	wxPoint first_cell_point;
	first_cell_point = grid->GetPosition();
	wxSize first_cell_size;
	first_cell_size.Set(grid->GetRowLabelSize(), grid->GetColLabelSize());
	wxColour background_colour = grid->GetLabelBackgroundColour();

	wxPanel *cover_first_cell = new wxPanel(panel, wxID_ANY, first_cell_point, first_cell_size, wxSUNKEN_BORDER);
	cover_first_cell->SetBackgroundColour(background_colour);

	wxStaticText *first_cell_text = new wxStaticText(cover_first_cell, -1, col_title_[0],wxPoint(10, 5), wxDefaultSize);

	wxFont grid_label_font = grid->GetLabelFont();
	first_cell_text->SetFont(grid_label_font);

	//Insert Cover for Col 8
	wxPanel *cover_col8 = new wxPanel(panel, wxID_ANY, wxPoint(479, first_cell_point.y+28), wxSize(grid->GetColSize(8)+1,grid->GetRowSize(0)*38 + 10 - 28), wxSUNKEN_BORDER);
	cover_col8->SetBackgroundColour(background_colour);

	//Insert Cover for Col 10
	wxPanel *cover_col10 = new wxPanel(panel, wxID_ANY, wxPoint(479 + grid->GetColSize(8) + grid->GetColSize(9), first_cell_point.y+28), wxSize(grid->GetColSize(8)+1,grid->GetRowSize(0)*38 + 10 - 28), wxSUNKEN_BORDER);
	cover_col10->SetBackgroundColour(background_colour);

	//Textlabel "X Col Bar"
	wxPanel *textlabel_x_col_bar = new wxPanel(panel, wxID_ANY, wxPoint(479 + grid->GetColSize(8),first_cell_point.y - grid->GetColLabelSize()), wxSize(grid->GetColSize(9) + 1, grid->GetColLabelSize()), wxSUNKEN_BORDER);
	textlabel_x_col_bar->SetBackgroundColour(background_colour);

	wxStaticText *text_x_col_bar = new wxStaticText(textlabel_x_col_bar, -1, wxT("X Col.Bar"),wxPoint(3, 5), wxDefaultSize);

	text_x_col_bar->SetFont(grid_label_font);

	//TextLabel "ErrorBar"
	wxPanel *textlabel_errorbar = new wxPanel(panel, wxID_ANY, wxPoint(479 + grid->GetColSize(8) + grid->GetColSize(9)+ grid->GetColSize(10),first_cell_point.y - grid->GetColLabelSize()), wxSize(grid->GetColSize(11)*5 + 1, grid->GetColLabelSize()), wxSUNKEN_BORDER);
	textlabel_errorbar->SetBackgroundColour(background_colour);

	wxStaticText *text_errorbar = new wxStaticText(textlabel_errorbar, -1, wxT("ErrorBar"),wxPoint(16, 5), wxDefaultSize);

	text_errorbar->SetFont(grid_label_font);

	//Checkbox for switch between Hex and Dec
	wxCheckBox *cb_hex_dec = new wxCheckBox(panel, skin_id::ID_HEX, wxT("Hex"), wxPoint(100,55), wxDefaultSize);
	Connect(skin_id::ID_HEX, wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(MyFrame::OnCheck));
	cb_hex_dec->SetValue(false);
	bhex_ = cb_hex_dec->GetValue();

	//Statusbar (Go/NoGo, NoErr/Err)
	wxStaticText *text_status = new wxStaticText(panel, -1, wxT("Status:"),wxPoint(200, 57), wxDefaultSize);
	text_status->SetFont(grid_label_font);

	text_go_nogo = new wxStaticText(panel, -1, wxT(""),wxPoint(270, 57), wxDefaultSize, wxALIGN_CENTRE);
	text_err_noerr = new wxStaticText(panel, -1, wxT(""),wxPoint(320, 57), wxDefaultSize, wxALIGN_CENTRE);


	//Creating the address row

	for(int k = 0; k < skind_def::max_sensor_number; k++)
	{
		if(bhex_)
		{
			grid->SetRowLabelValue(k,hexadezimal(skin_dat_.u8readadr_lo_+256*skin_dat_.u8readadr_hi_+skind_def::anz_byte_per_sensor*k));
		}
		else
		{
			grid->SetRowLabelValue(k, wxString::Format(wxT("%i"),(skin_dat_.u8readadr_lo_+256*skin_dat_.u8readadr_hi_+skind_def::anz_byte_per_sensor*k)));
		}
	}

	//display initialze data
	publish_data_2_gui();

	Centre();

}


void MyFrame::gridUpdate(wxTimerEvent& evt)//trigger a new message callback
{
	ros::spinOnce();
	std::cout << "OneUpdate" << " \n";
	publish_data_2_gui();//publish data to GUI

	if (!ros::ok())
	{
		Close();
	}
}


void MyFrame::OnQuit(wxCommandEvent& WXUNUSED(event))
{
	Close(true);
}


void MyFrame::OnAbout(wxCommandEvent& WXUNUSED(event))
{
	wxMessageBox( wxT(".....      text  text  text     ....."), wxT("Sensor Skin Monitor"), wxICON_INFORMATION, this );
}


void MyFrame::OnCheck(wxCommandEvent& event)//switch between Hex and Dec
{
	if(bhex_)
	{
		bhex_ = false;
	}
	else
	{
		bhex_ = true;
	}
}


wxString MyFrame::hexadezimal(uint32_t conv_value)//convert a dec. value into a hex-string
{
	const wxString ziffern[16] = {wxT("0"),wxT("1"),wxT("2"),wxT("3"),wxT("4"),wxT("5"),wxT("6"),wxT("7"),wxT("8"),wxT("9"),wxT("A"),wxT("B"),wxT("C"),wxT("D"),wxT("E"),wxT("F")};
	int i = 0;
	wxString ts;

	ts = wxT("");
	for(i = 3 ; i >= 0 ; i--)
	{
		ts = ts + ziffern[((conv_value >> (4*i)) & (15))]; // 15 = 000FH = 1111B
	}
	return ts;
}


void MyFrame::publish_data_2_gui()
{
	std::cout << "publish_data_2_gui" << std::endl;

	//clear gui if sensor number was changed
	if(skin_dat_.bsens_num_changed == true)
	{
		grid->ClearGrid();
		skin_dat_.bsens_num_changed = false;

		//Inserts sensor number in the  9th column, change text colour, set background colour
		for(int i = 0; i < skind_def::max_sensor_number ; i++)
		{
			grid->SetCellValue(i,9,wxString::Format(wxT("%i"),i+1));
			grid->SetCellTextColour(i,9,wxColour(255,255,0));
		}

		//clear background colour
		for(int k = skin_dat_.u8sensor_number_; k < skind_def::max_sensor_number; k++)
		{
			grid->SetCellBackgroundColour(k, 9, *wxWHITE);//x Col.Bar
			grid->SetCellBackgroundColour(k, 11, *wxWHITE);//ErrorBar
			grid->SetCellBackgroundColour(k, 12, *wxWHITE);//ErrorBar
			grid->SetCellBackgroundColour(k, 13, *wxWHITE);//ErrorBar
			grid->SetCellBackgroundColour(k, 14, *wxWHITE);//ErrorBar
			grid->SetCellBackgroundColour(k, 15, *wxWHITE);//ErrorBar
		}
	}

	//check if publisher node is online (online: time stamp of the last subscription < (ceil(float(skin_dat_.u8sensor_number_) / 32.0) -1 ) * 0.1 + 0.3 ago)
	//1 Sens - 32 Sens 		- limit : 0.25
	//33 Sens - 64 Sens 	- limit : 0.35
	//65 Sens - 96 Sens 	- limit : 0.45
	//97 Sens - 128 Sens 	- limit : 0.55
	//129 Sens - 160 Sens 	- limit : 0.65
	//161 Sens - 192 Sens 	- limit : 0.75
	//determination of the limit: (ceil(float(skin_dat_.u8sensor_number_) / 32.0) -1 ) * 0.1 + 0.3

	ros::Time dcurrent_time = ros::Time::now();
	if( ( dcurrent_time > (skin_dat_.dtime_subscription_ + ros::Duration((ceil(float(skin_dat_.u8sensor_number_) / 32.0) -1 ) * 0.1 + 0.3))) )
	{
		skin_dat_.initialize_sensor_data();//reset data to initial values

		text_go_nogo->SetLabel(wxT(""));//status diplay is empty if publisher node is offline
		text_err_noerr->SetLabel(wxT(""));
	}
	else
	{
		//status diplay
		if(skin_dat_.bstatus_go_ == true)
		{
			text_go_nogo->SetLabel(wxT("Go"));
			text_go_nogo->SetForegroundColour(*wxGREEN);
		}
		else
		{
			text_go_nogo->SetLabel(wxT("NoGo"));
			text_go_nogo->SetForegroundColour(*wxRED);
		}

		if(skin_dat_.bstatus_err_ == false)
		{
			text_err_noerr->SetLabel(wxT("NoErr"));
			text_err_noerr->SetForegroundColour(*wxGREEN);
		}
		else
		{
			text_err_noerr->SetLabel(wxT("Err"));
			text_err_noerr->SetForegroundColour(*wxRED);
		}
	}

	//create new address-row depending on bhex_
	for(int l = 0; l < skind_def::max_sensor_number; l++)
	{
		if(bhex_)
		{
			grid->SetRowLabelValue(l,hexadezimal(skin_dat_.u8readadr_lo_+256*skin_dat_.u8readadr_hi_+skind_def::anz_byte_per_sensor*l));
		}
		else
		{
			grid->SetRowLabelValue(l, wxString::Format(wxT("%i"),(skin_dat_.u8readadr_lo_+256*skin_dat_.u8readadr_hi_+skind_def::anz_byte_per_sensor*l)));
		}
	}

	for(int k = 0; k < skin_dat_.u8sensor_number_; k++)
	{
		//publish skin_data to main grid (Col0 to Col7)
		for(int j = 0; j < 8; j++)
		{
			//write data to grid
			if(bhex_)
			{
				grid->SetCellValue(k,j,hexadezimal(skin_dat_.u8asensor_data_[k][j]));//convert data to hex-string
			}
			else
			{
				grid->SetCellValue(k,j,wxString::Format(wxT("%i"),skin_dat_.u8asensor_data_[k][j]));
			}
		}


		///////////////////////////////////////////////////
		//         Creating the ErrorBar
		///////////////////////////////////////////////////

		//self diagnosis display for each sensor element (errorbar)
		//  green: no error
		//  red:   error found
		//  white: no sensor element
		//  error conditions:
		//    A: U2 > 32		(= berrcond_[0])
		//    B: U3 < 16		(= berrcond_[1])
		//    C: U3-U1 < 16		(= berrcond_[2])
		//    D: Err_IO = 1		(= berrcond_[3])
		//    E: ErrTMS = 1		(= berrcond_[4])
		//    U1 = sensor_data[k][4]
		//	  U2 = sensor_data[k][5]
		//    U3 = sensor_data[k][6]
		//    Stat  = sensor_data[k][0]
		//    Limit = sensor_data[k][3]


		berrcond_[0] = ( (skin_dat_.u8asensor_data_[k][5]) > 32 );
		berrcond_[1] = ( (skin_dat_.u8asensor_data_[k][6]) < 16 );
		berrcond_[2] = ( ((skin_dat_.u8asensor_data_[k][6]) - (skin_dat_.u8asensor_data_[k][4])) < 16 );
		berrcond_[3] = ( ((skin_dat_.u8asensor_data_[k][0]) & 16) > 0 );	//masking of Stat: if ((sensor_data[k][0]) & 16)>0 -> Err_IO = 1
		berrcond_[4] = ( ((skin_dat_.u8asensor_data_[k][0]) & 32) > 0 );    //masking of Stat: if ((sensor_data[k][0]) & 32)>0 -> ErrTMS = 1

		wxColour errcol;

		for(int i = 0; i < 5; i++)
		{
			errcol = *wxWHITE;
			if((skin_dat_.u8sensor_number_ > 0) & (skin_dat_.u8sensor_number_ <= skind_def::max_sensor_number))//number of sensors:  0 < Sennr <= 192
			{
				if( ((skin_dat_.u8asensor_data_[k][0]) & 1) == 1)//(Sensorelement is evaluated)
				{
					if(berrcond_[i])
					{
						errcol = *wxRED;
					}
					else
					{
						errcol = *wxGREEN;
					}
				}
				grid->SetCellBackgroundColour(k, i + 11, errcol);
			}
		}

		//sensor element readings (X COL.BAR)
		//    Limit = sensor_data[k][3]
		//    X     = sensor_data[k][1]
		//    Ini   = sensor_data[k][2]

		////////////////////////////////////////////
		//    Creating the X. Col.Bar
		////////////////////////////////////////////

		float tr;
		wxColour statcol;

		if((skin_dat_.u8asensor_data_[k][3]) != 0)
		{
			tr = float ( (skin_dat_.u8asensor_data_[k][1]) - (skin_dat_.u8asensor_data_[k][2]) ) / float (skin_dat_.u8asensor_data_[k][3]); // tr = (X - Ini) / Limit
		}
		else
		{
			tr = 2;
		}


		if(tr > 0)
		{
			if(tr > 1)
			{
				tr = 1;
			}
			statcol.Set(int (tr * 255)*65536);
			grid->SetCellBackgroundColour(k, 9, statcol);
		}
		else
		{
			tr = -tr;
			if(tr > 1)
			{
				tr = 1;
			}
			statcol.Set(int (tr * 255));
			grid->SetCellBackgroundColour(k, 9, statcol);
		}
	}

	grid->ForceRefresh();
}








