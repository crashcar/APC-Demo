/****************************************************************************/
/// @file    Intersection.h
 
 
/// @date    December 2023
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

#ifndef APPLRSUINTERSECTION_H
#define APPLRSUINTERSECTION_H

#include "nodes/rsu/05_CRL.h"
#include "msg/dataMsg_m.h"
#include "msg/PltInfo_m.h"
#include "msg/PltCtrl_m.h"
#include "optimizeSpeedAcc.h"

#define WEST_STOP_LINE_X    -14.0
#define VEH_LENGTH          5.0
#define GREEN_DURATION      30.0
#define RED_DURATION        30.0
#define YELLOW_DURATION     3.0
#define V_MAX               16.66
#define V_MIN               0.0
#define LAMBDA1             1000.0
#define LAMBDA2             1.0
#define LAMBDA3             1.0


namespace VENTOS {

class ApplRSUIntersection : public ApplRSUCRL
{
private:
    typedef ApplRSUCRL super;
    typedef struct
    {
        double refVelocity;
        double refAcc;
        int optSize;
    }CtrlValue;
    typedef enum Stage {
            GO_STAGE,
            WAIT_STAGE
    }Stage;
    void sendPltCtrl(std::string receiverID, std::string receivingPlatoonID, double refSpeed, double refAcc, int optSize); //sendPltCtrl.msg after calculate
    CtrlValue getCtrlValue(double TG, TraCICoord pos, double speed, double maxAccel, double maxDecel);
    Stage determineStage(double distance, double vCurrent, double& nextRedTime, double& nextGreenTime);
    double calculateLeaderArrivalTime(double maxAccel, double vCurrent, double D);

public:
    ~ApplRSUIntersection();
    virtual void initialize(int stage);
    virtual void finish();

protected:
    virtual void handleSelfMsg(omnetpp::cMessage*);
    void onBeaconVehicle(BeaconVehicle* wsm);
    void onBeaconRSU(BeaconRSU* wsm);
    void onDataMsg(dataMsg *wsm);
    void onPltInfo(PltInfo* wsm); //receive PltInfo.msg from leader entering ZONE

    void executeEachTimeStep();
};

}

#endif
