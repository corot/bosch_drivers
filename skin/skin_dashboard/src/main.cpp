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

//ROS Header
#include <ros/ros.h>

//Prog-Header
#include "skin_dashboard/main.h"
#include "skin_dashboard/user_interface.h"

IMPLEMENT_APP(MyApp)

bool MyApp::OnInit()
{
    // create our own copy of argv, with regular char*s.
    my_argv_ =  new char*[ argc ];
    for ( int j = 0; j < argc; ++j )
    {
      my_argv_[ j ] = strdup( wxString( argv[ j ] ).mb_str() );
    }

	// Initialize ROS
	ros::init(argc, my_argv_, "skin_dashboard");
	nh_.reset(new ros::NodeHandle);

    //Create GUI
    MyFrame *frame = new MyFrame( wxT("Bosch RTC"), wxDefaultPosition, wxSize(758, 900));
    frame->Show(true);
    SetTopWindow(frame);

    return true;
}


int MyApp::OnExit()
{
  for ( int j = 0; j < argc; ++j )
  {
    free( my_argv_[ j ] );
  }
  delete [] my_argv_;


  return 0;
}












