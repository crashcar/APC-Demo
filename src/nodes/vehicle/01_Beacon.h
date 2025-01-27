/****************************************************************************/
/// @file    Beacon.h
 
 
/// @date    August 2013
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

#ifndef ApplVBeacon_H
#define ApplVBeacon_H

#include <deque>

#include "baseAppl/04_BaseWaveApplLayer.h"
#include "msg/BeaconVehicle_m.h"

namespace VENTOS {

class ApplVBeacon : public BaseWaveApplLayer
{
private:
    typedef BaseWaveApplLayer super;

protected:
    double sonarDist;
    omnetpp::cOutVector positionXVector;
    omnetpp::cOutVector positionYVector;
    omnetpp::cOutVector speedVector;
    omnetpp::cOutVector accVector;

public:
    ~ApplVBeacon();
    virtual void initialize(int stage);
    virtual void finish();

protected:
    virtual void handleSelfMsg(omnetpp::cMessage*);
    virtual void sendBeacon();

    bool isBeaconFromFrontVehicle(BeaconVehicle*);
    bool isBeaconFromMyPlatoon(BeaconVehicle*);
    bool isBeaconFromMyPlatoonLeader(BeaconVehicle*);

    virtual std::string getPlatoonId();
    virtual int getPlatoonDepth();
    virtual void handlePositionUpdate(omnetpp::cObject* obj);

private:
    BeaconVehicle* generateBeacon();
};

}

#endif
