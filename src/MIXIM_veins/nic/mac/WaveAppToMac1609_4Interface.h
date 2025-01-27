//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
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

#include "NetwToMacControlInfo.h"
#include "MIXIM_veins/nic/Consts80211p.h"

#ifndef ___WAVEAPPTOMAC1609_4INTERFACE_H_
#define ___WAVEAPPTOMAC1609_4INTERFACE_H_

namespace Veins {

/**
 * @brief
 * Interface between WaveApplication Layer and Mac1609_4
 *
 
 *
 * @ingroup macLayer
 */
class WaveAppToMac1609_4Interface
{
public:
    virtual const LAddress::L2Type& getMACAddress() = 0;
    virtual bool isChannelSwitchingActive() = 0;
    virtual omnetpp::simtime_t getSwitchingInterval() =  0;
    virtual bool isCurrentChannelCCH() = 0;
    virtual void changeServiceChannel(int channelNumber) = 0;
    virtual ~WaveAppToMac1609_4Interface() {} ;
};

}

#endif /* ___WAVEAPPTOMAC1609_4INTERFACE_H_ */
