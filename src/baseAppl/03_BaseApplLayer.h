/* -*- mode:c++ -*- ********************************************************
 * file:        BaseApplLayer.h
 *
 
 *
 * copyright:   (C) 2004 Telecommunication Networks Group (TKN) at
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
 * description: application layer: general class for the application layer
 *              subclass to create your own application layer
 **************************************************************************/

#ifndef BASE_APPL_LAYER_H
#define BASE_APPL_LAYER_H

#include <assert.h>
#include <baseAppl/02_BaseLayer.h>

#include "global/MiXiMDefs.h"
#include "MIXIM_veins/nic/mac/SimpleAddress.h"

/**
 * @brief Base class for the application layer
 *
 * This is the generic class for all application layer modules. If you
 * want to implement your own application layer you have to subclass your
 * module from this class.
 *
 * @ingroup applLayer
 * @ingroup baseModules
 *
 
 **/

class MIXIM_API BaseApplLayer : public BaseLayer
{
public:

	/** @brief The message kinds this layer uses.*/
	enum BaseApplMessageKinds
	{
		/** Stores the id on which classes extending BaseAppl should
		 * continue their own message kinds.*/
		LAST_BASE_APPL_MESSAGE_KIND = 25000,
	};

	/** @brief The control message kinds this layer uses.*/
	enum BaseApplControlKinds
	{
		/** Stores the id on which classes extending BaseAppl should
		 * continue their own control kinds.*/
		LAST_BASE_APPL_CONTROL_KIND = 25500,
	};

protected:

	/**
	 * @brief Length of the ApplPkt header
	 **/
	int headerLength;

public:

	BaseApplLayer() : BaseLayer() { }
	BaseApplLayer(unsigned stacksize) : BaseLayer(stacksize) { }

	/** @brief Initialization of the module and some variables*/
	virtual void initialize(int);

protected:
	/**
	 * @name Handle Messages
	 * @brief Functions to redefine by the programmer
	 *
	 * These are the functions provided to add own functionality to your
	 * modules. These functions are called whenever a self message or a
	 * data message from the upper or lower layer arrives respectively.
	 *
	 **/
	/*@{*/

	/**
	 * @brief Handle self messages such as timer...
	 *
	 * Define this function if you want to process timer or other kinds
	 * of self messages
	 **/
	virtual void handleSelfMsg(omnetpp::cMessage* msg)
	{
		EV << "BaseApplLayer: handleSelfMsg not redefined; delete msg \n";
		delete msg;
	};

	/**
	 * @brief Handle messages from lower layer
	 *
	 * Redefine this function if you want to process messages from lower
	 * layers.
	 *
	 * The basic application layer just silently deletes all messages it
	 * receives.
	 **/
	virtual void handleLowerMsg(omnetpp::cMessage* msg)
	{
		EV << "BaseApplLayer: handleLowerMsg not redefined; delete msg \n";
		delete msg;
	};

	/**
	 * @brief Handle control messages from lower layer
	 *
	 * The basic application layer just silently deletes all messages it
	 * receives.
	 **/
	virtual void handleLowerControl(omnetpp::cMessage* msg)
	{
		EV << "BaseApplLayer: handleLowerControl not redefined; delete msg \n";
		delete msg;
	};

	/** @brief Handle messages from upper layer
	 *
	 * This function is pure virtual here, because there is no
	 * reasonable guess what to do with it by default.
	 */
	virtual void handleUpperMsg(omnetpp::cMessage *msg)
	{
        delete msg;
		throw omnetpp::cRuntimeError("Application has no upper layers!");
	}

	/** @brief Handle control messages from upper layer */
	virtual void handleUpperControl(omnetpp::cMessage *msg)
	{
        delete msg;
		throw omnetpp::cRuntimeError("Application has no upper layers!");
	}

	/*@}*/

	/** @brief Sends a message delayed to the lower layer*/
	void sendDelayedDown(omnetpp::cMessage *, omnetpp::simtime_t_cref);

	/**
	 * @brief Return my application layer address
	 *
	 * We use the node module index as application address
	 **/
	virtual const LAddress::L3Type myApplAddr() const
	{
		return LAddress::L3Type( getParentModule()->getIndex() );
	};

};

#endif
