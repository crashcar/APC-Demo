/****************************************************************************/
/// @file    LQF_MWM.h
 
/// @date    Jul 2015
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

// This algorithm is implemented according to the paper, but suffers from starvation.
// It performs the scheduling per phase (and not per cycle)

#ifndef TRAFFICLIGHTLQFMWM_H
#define TRAFFICLIGHTLQFMWM_H

#include "trafficLight/TSC/05_OJF.h"

namespace VENTOS {

class TrafficLight_LQF_MWM : public TrafficLightOJF
{
private:
    typedef TrafficLightOJF super;

    std::string currentInterval;
    double intervalDuration;
    std::string nextGreenInterval;

    omnetpp::cMessage* intervalChangeEVT = NULL;

    double nextGreenTime;

    std::vector<std::string> phases = {phase1_5, phase2_6, phase3_7, phase4_8};

    std::map<std::string /*TLid*/, std::string /*first green interval*/> firstGreen;

    std::map<std::pair<std::string /*TLid*/, int /*link*/>, std::string /*lane*/> link2Lane;

    // list of all 'incoming lanes' in each TL
    std::unordered_map< std::string /*TLid*/, std::vector<std::string> > incomingLanes_perTL;

    typedef struct sortedEntry
    {
        double totalWeight;
        int oneCount;
        int maxVehCount;
        std::string phase;
    } sortedEntry_t;

    typedef struct sortFunc
    {
        bool operator()(sortedEntry_t p1, sortedEntry_t p2)
        {
            if( p1.totalWeight < p2.totalWeight )
                return true;
            else if( p1.totalWeight == p2.totalWeight && p1.oneCount < p2.oneCount)
                return true;
            else
                return false;
        }
    } sortFunc_t;

    // batch of all non-conflicting movements, sorted by total weight + oneCount per batch
    typedef std::priority_queue< sortedEntry_t /*type of each element*/, std::vector<sortedEntry_t> /*container*/, sortFunc_t > priorityQ;

public:
    virtual ~TrafficLight_LQF_MWM();
    virtual void initialize(int);
    virtual void finish();
    virtual void handleMessage(omnetpp::cMessage *);

protected:
    void virtual initialize_withTraCI();
    void virtual executeEachTimeStep();

private:
    void chooseNextInterval(std::string);
    void chooseNextGreenInterval(std::string);
};

}

#endif
