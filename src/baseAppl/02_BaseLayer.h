/* -*- mode:c++ -*- ********************************************************
 * file:        BaseLayer.h
 *
 
 *
 * copyright:   (C) 2006 Telecommunication Networks Group (TKN) at
 *              Technische Universitaet Berlin, Germany.
 *
 *              This program is free software; you can redistribute it
 *              and/or modify it under the terms of the GNU General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later
 *              version.
 *              For further information see file COPYING
 *              in the top level directory
 ***************************************************************************
 * part of:     framework implementation developed by tkn
 * description: basic layer class
 *              subclass to create your own layer
 **************************************************************************/

#ifndef BASE_LAYER_H
#define BASE_LAYER_H

#include "global/MiXiMDefs.h"
#include "PassedMessage.h"
#include "baseAppl/01_BaseModule.h"

/**
 * @brief A very simple layer template
 *
 * This module provides basic abstractions that ease development of a
 * network or MAC layer.
 *
 * @ingroup baseModules
 
 */

class MIXIM_API BaseLayer : public BaseModule
{
public:

    /** @brief SignalID for packets. */
    const static simsignalwrap_t catPacketSignal;

    /** @brief Signal for passed messages.*/
    const static simsignalwrap_t catPassedMsgSignal;

    /** @brief Signal for dropped packets.*/
    const static simsignalwrap_t catDroppedPacketSignal;

protected:

    /** @name gate ids*/
    /*@{*/
    int upperLayerIn;
    int upperLayerOut;
    int lowerLayerIn;
    int lowerLayerOut;
    int upperControlIn;
    int upperControlOut;
    int lowerControlIn;
    int lowerControlOut;
    /*@}*/

    /** @brief The last message passed through this layer. This variable will be only not NULL if we are
     * in statistic recording mode.*/
    PassedMessage *passedMsg = NULL;

public:

    BaseLayer() : passedMsg(NULL) {}
    BaseLayer(unsigned stacksize): passedMsg(NULL) {}
    virtual ~BaseLayer();

    /** @brief Initialization of the module and some variables*/
    virtual void initialize(int);

    /** @brief Called every time a message arrives*/
    virtual void handleMessage( omnetpp::cMessage* );

    /** @brief Called when the simulation has finished.*/
    virtual void finish();

protected:

    /**
     * @name Handle Messages
     * @brief Functions to be redefined by the programmer
     *
     * These are the functions provided to add own functionality to
     * your modules. These functions are called whenever
     * a self message or a data message from the upper or
     * lower layer arrives respectively.
     *
     **/
    /*@{ */

    /** @brief Handle self messages such as timer... */
    virtual void handleSelfMsg(omnetpp::cMessage* msg) = 0;

    /** @brief Handle messages from upper layer
     *
     * This function is pure virtual here, because there is no
     * reasonable guess what to do with it by default.
     */
    virtual void handleUpperMsg(omnetpp::cMessage *msg) = 0;

    /** @brief Handle messages from lower layer */
    virtual void handleLowerMsg(omnetpp::cMessage *msg) = 0;

    /** @brief Handle control messages from lower layer */
    virtual void handleLowerControl(omnetpp::cMessage *msg) = 0;

    /** @brief Handle control messages from upper layer */
    virtual void handleUpperControl(omnetpp::cMessage *msg) = 0;

    /*@}*/


    /**
     * @name Convenience Functions
     * @brief Functions for convenience - NOT to be modified
     *
     * These are functions taking care of message encapsulation and
     * message sending. Normally you should not need to alter these.
     *
     * All these functions assume that YOU do all the necessary handling
     * of control information etc. before you use them.
     **/
    /*@{*/

    /** @brief Sends a message to the lower layer
     *
     * Short hand for send(msg, lowerLayerOut);
     *
     * You have to take care of encapsulation We recommend that you
     * use a pair of functions called encapsMsg/decapsMsg.
     */
    void sendDown(omnetpp::cMessage *msg);

    /** @brief Sends a message to the upper layer
     *
     * Short hand for send(msg, upperLayerOut);
     * You have to take care of decapsulation and deletion of
     * superflous frames. We recommend that you use a pair of
     * functions decapsMsg/encapsMsg.
     */
    void sendUp(omnetpp::cMessage *msg);

    /** @brief Sends a control message to an upper layer */
    void sendControlUp(omnetpp::cMessage *msg);

    /** @brief Sends a control message to a lower layer */
    void sendControlDown(omnetpp::cMessage *msg);

    void recordPacket(PassedMessage::direction_t dir, PassedMessage::gates_t gate, const omnetpp::cMessage *m);
};

#endif
