/****************************************************************************/
/// @file    ApplToPhyControlInfo.h
 
 
/// @date    Feb 207
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
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef APPLTOPHYCONTROLINFO_M_H
#define APPLTOPHYCONTROLINFO_M_H

#include <omnetpp.h>

namespace VENTOS {

class ApplToPhyControlInfo : public ::omnetpp::cObject
{
  protected:
    int mcs;
    double txPower_mW;

  private:
    void copy(const ApplToPhyControlInfo& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const ApplToPhyControlInfo&);

  public:

    ApplToPhyControlInfo(const char *name=nullptr, int kind=0);
    ApplToPhyControlInfo(const ApplToPhyControlInfo& other);
    virtual ~ApplToPhyControlInfo();
    ApplToPhyControlInfo& operator=(const ApplToPhyControlInfo& other);

    // getter methods
    virtual int getMcs() const;
    virtual double getTxPower_mW() const;

    // setter methods
    virtual void setMcs(int mcs);
    virtual void setTxPower_mW(double txPower_mW);
};

}

#endif
