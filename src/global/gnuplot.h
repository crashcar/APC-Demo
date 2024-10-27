/****************************************************************************/
/// @file    gnuplot.h
 
 
/// @date    July 2017
///
/****************************************************************************/
// VENTOS, Vehicular Network Open Simulator; see http:?
// Copyright (C) 2013-2015
/****************************************************************************/
//
// This file is part of VENTOS.
// VENTOS is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful}},
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef GNUPLOT_H
#define GNUPLOT_H

#include <fstream>
#include <omnetpp.h>


namespace VENTOS {


class gnuplot
{
private:
    FILE *plotterPtr = NULL;

public:
    gnuplot();
    virtual ~gnuplot();

    double getVersion();
    void sendCommand(const char *fmt, ...);
    void flush();
};

}

#endif

