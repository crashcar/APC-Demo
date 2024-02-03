/****************************************************************************/
/// @file    Intersection.cc
/// @author  yijun pu
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

#include "06_Intersection.h"

namespace VENTOS {

Define_Module(VENTOS::ApplRSUIntersection);

ApplRSUIntersection::~ApplRSUIntersection()
{

}


void ApplRSUIntersection::initialize(int stage)
{
    super::initialize(stage);
}


void ApplRSUIntersection::finish()
{
    super::finish();
}


void ApplRSUIntersection::executeEachTimeStep()
{
    // call the super method
    super::executeEachTimeStep();
}


void ApplRSUIntersection::handleSelfMsg(omnetpp::cMessage* msg)
{
    super::handleSelfMsg(msg);
}


void ApplRSUIntersection::onBeaconVehicle(BeaconVehicle* wsm)
{
    // pass it down!
    super::onBeaconVehicle(wsm);
}


void ApplRSUIntersection::onBeaconRSU(BeaconRSU* wsm)
{
    // pass it down!
    super::onBeaconRSU(wsm);
}


void ApplRSUIntersection::onDataMsg(dataMsg *wsm)
{
    // do not pass it down!
}


// receive PltInfo.msg from leader entering ZONE
void ApplRSUIntersection::onPltInfo(PltInfo* wsm)
{
    if((strcmp(wsm->getReceiverID(), myFullId) == 0))
    {
        // collect value from wsm
        std::string sender = wsm->getSenderID();
        std::string sendingPlatoonID = wsm->getSendingPlatoonID();
        double TG = wsm->getTG();
        TraCICoord pos = wsm->getPos();
        double vCurrent = wsm->getSpeed();
        double maxAccel = wsm->getMaxAccel();
        double maxDecel = wsm->getMaxDecel();

        // get control value
        CtrlValue cValue = getCtrlValue(TG, pos, vCurrent, maxAccel, maxDecel);

        // send PltCtrl.msg
        sendPltCtrl(sender, sendingPlatoonID, cValue.refVelocity, cValue.refAcc, cValue.optSize);
    }
}

ApplRSUIntersection::CtrlValue ApplRSUIntersection::getCtrlValue(double TG, TraCICoord pos, double vCurrent, double maxAccel, double maxDecel)
{
    double distance = abs(pos.x - WEST_STOP_LINE_X);
    double D = distance - VEH_LENGTH;
    double nextRedTime, nextGreenTime;

    Stage currentStage = determineStage(distance, vCurrent, nextRedTime, nextGreenTime);

    // return value
    double refVelocity;
    double refAcc;
    int optSize;

    // calculate CtrlValue for different stage
    switch(currentStage)
    {
        case GO_STAGE:
        {
            // optSize
            double leaderArrivalTime = calculateLeaderArrivalTime(maxAccel, vCurrent, distance);
            double adjTG = 1.05 * TG;
            optSize = floor((nextRedTime - leaderArrivalTime) / adjTG) + 1;

            // speed acc
            refVelocity = V_MAX;
            refAcc = maxAccel;
            
            break;
        }
        case WAIT_STAGE:
        {
            // optSize
            optSize = floor(GREEN_DURATION / TG) + 1;
            // speed acc
            solveOptimizationProblem(LAMBDA1, LAMBDA2, LAMBDA3, D, vCurrent, nextGreenTime, refAcc, refVelocity);
            
            break;
        }
    }

   LOG_INFO << boost::format("CONTROL INFO:\nrefVelocity: %.2f, refAcc: %.2f, optSize:%.2f\n")
                            %refVelocity%refAcc%optSize
                            << std::flush;

    ApplRSUIntersection::CtrlValue cValue;
    cValue.refVelocity = refVelocity;
    cValue.refAcc = refAcc;
    cValue.optSize = optSize;
    return cValue;
}

ApplRSUIntersection::Stage ApplRSUIntersection::determineStage(double distance, double vCurrent, double& nextRedTime, double& nextGreenTime)
{
    Stage currentStage;
    std::string stageString;

    // TrafficLight
    int nextSwitchTimeMs = TraCI->TLGetNextSwitchTime("2");
    double nextSwitchTime = nextSwitchTimeMs / 1000;
    double currentTime = omnetpp::simTime().dbl();
    double remainingTime = nextSwitchTime - currentTime;
    std::string state = TraCI->TLGetState("2");
    char nowSignal = state[17];

    if(remainingTime < 0)
    {
        throw omnetpp::cRuntimeError("TrafficLight next switch time wrong!");
    }

    // determine stage
    if(nowSignal == 'G') // now green
    {
        double threshold = distance / vCurrent;
        if(remainingTime > threshold)
        {
            currentStage = GO_STAGE;
            stageString = "GO_STAGE";
            nextRedTime = remainingTime;
        }
        else
        {
            currentStage = WAIT_STAGE;
            stageString = "WAIT_STAGE";
            nextGreenTime = remainingTime + RED_DURATION + YELLOW_DURATION;
        }
    }
    else if(nowSignal == 'r')   // now red
    {
        double threshold = distance / V_MAX;
        if(remainingTime > threshold)
        {
            currentStage = WAIT_STAGE;
            stageString = "WAIT_STAGE";
            nextGreenTime = remainingTime;
        }
        else
        {
            currentStage = GO_STAGE;
            stageString = "GO_STAGE";
            nextRedTime = remainingTime + GREEN_DURATION;
        }
    }
    else // now yellow
    {
        currentStage = WAIT_STAGE;
        stageString = "WAIT_STAGE";
        nextGreenTime = remainingTime + RED_DURATION;
    }
    
   LOG_INFO << boost::format("STAGE INFO:\ncurrentStage: %s, distance: %.2f, nowSignal: %s, remainingTime: %.2f, "
            "nextGreenTime: %.2f, nextRedTime: %.2f\n")
            %stageString%distance%nowSignal%remainingTime%nextGreenTime%nextRedTime
            << std::flush;

    return currentStage;
}

double ApplRSUIntersection::calculateLeaderArrivalTime(double maxAccel, double vCurrent, double D) 
{
    double leaderArrivalTime;
    if(V_MAX * V_MAX < 2 * maxAccel * D)
    {
        double t1 = (V_MAX - vCurrent) / maxAccel,
               d2 = D - ((V_MAX * V_MAX - vCurrent * vCurrent) / (2 * maxAccel)),
               t2 = d2 / V_MAX;
        leaderArrivalTime = t1 + t2;
        std::cout << "calculated leaderArrivalTime (situation 2): " << leaderArrivalTime << std::endl;

    } else {

        double vThreshold = std::sqrt(V_MAX * V_MAX - 2 * maxAccel * D);
        if(vCurrent < vThreshold)
        {
            double t1 = (V_MAX - vCurrent) / maxAccel,
                d2 = D - ((V_MAX * V_MAX - vCurrent * vCurrent) / (2 * maxAccel)),
                t2 = d2 / V_MAX;
            leaderArrivalTime = t1 + t2;
            std::cout << "calculated leaderArrivalTime (situation 2): " << leaderArrivalTime << std::endl;

        } else {
            leaderArrivalTime = (std::sqrt(2 * maxAccel * D + vCurrent * vCurrent) - vCurrent) / maxAccel;
            std::cout << "calculated leaderArrivalTime (situation 1): " << leaderArrivalTime << std::endl;
        }
    }
    return leaderArrivalTime;
}

// send PltCtrl.msg
void ApplRSUIntersection::sendPltCtrl(std::string receiverID, std::string receivingPlatoonID, double refSpeed, double refAcc, int optSize)
{
    PltCtrl* wsm = new PltCtrl("pltCtrl", TYPE_PLATOON_CONTROL);

    wsm->setWsmVersion(1);
    wsm->setSecurityType(1);
    wsm->setChannelNumber(Veins::Channels::CCH);
    wsm->setDataRate(1);
    wsm->setPriority(dataPriority);
    wsm->setPsid(0);

    wsm->setSenderID(SUMOID.c_str());
    wsm->setReceiverID(receiverID.c_str());
    wsm->setReceivingPlatoonID(receivingPlatoonID.c_str());
    wsm->setRefSpeed(refSpeed);
    wsm->setRefAcc(refAcc);
    wsm->setOptSize(optSize);

    // add header length
    wsm->addBitLength(headerLength);

    // add payload length
    wsm->addBitLength(dataLengthBits);

    send(wsm, lowerLayerOut);
}

}
