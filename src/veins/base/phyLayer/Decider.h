/* -*- mode:c++ -*- ********************************************************
 *
 *
 *
 */

#ifndef DECIDER_H_
#define DECIDER_H_

#include <omnetpp.h>

#include "veins/base/utils/MiXiMDefs.h"
#include "veins/base/phyLayer/DeciderToPhyInterface.h"
#include "veins/base/messages/ChannelSenseRequest_m.h"
#include "veins/base/phyLayer/ChannelState.h"

using Veins::AirFrame;



/**
 * @brief A class to represent the result of a processed packet (that is not
 * noise) by the Decider.
 *
 * It stores information (i.e. basically whether a packet has been received
 * correctly) for the MACLayer that is handed up to the MACLayer by the PhyLayer
 * together with the received packet. (see also DeciderToPhyInterface)
 *
 * You can subclass DeciderResult to create a more detailed result.
 *
 * @ingroup decider
 */
class MIXIM_API DeciderResult
{
protected:
	/** Stores if the AirFrame for this result was received correct.*/
	bool isCorrect;
public:

	virtual ~DeciderResult() {}

	/**
	 * @brief Initializes the DeciderResult with the passed bool, or true
	 * if omitted.
	 */
	DeciderResult(bool isCorrect = true):
		isCorrect(isCorrect) {}

	/**
	 * @brief A Function that returns a very basic result about the Signal.
	 */
	 virtual bool isSignalCorrect() const;

};


/**
 * @brief The basic Decider class
 *
 * The Deciders tasks are:
 * 	1.	decide which packets should be handed up to the MAC Layer (primary task)
 * 	2.	decide whether the channel is busy/idle at a time point or
 * 		during a time interval (channel sensing)
 *
 * BasePhyLayer hands every receiving AirFrame several times to the
 * "processSignal()"-function and is returned a time point when to do so again.
 *
 * @ingroup decider
 */
class MIXIM_API Decider
{
protected:
	/** @brief A pointer to the physical layer of this Decider. */
	DeciderToPhyInterface* phy;

	/** @brief simtime that tells the Phy-Layer not to pass an AirFrame again */
	const simtime_t notAgain;

	/** @brief Defines what an AirFrameVector shall be here */
	typedef DeciderToPhyInterface::AirFrameVector AirFrameVector;

public:

	/**
	 * @brief Initializes the Decider with a pointer to its PhyLayer
	 */
	Decider(DeciderToPhyInterface* phy);

	virtual ~Decider() {}

	/**
	 * @brief This function processes a AirFrame given by the PhyLayer and
	 * returns the time point when Decider wants to be given the AirFrame again.
	 */
	virtual simtime_t processSignal(AirFrame* frame);

	/**
	 * @brief A function that returns information about the channel state
	 *
	 * It is an alternative for the MACLayer in order to obtain information
	 * immediately (in contrast to sending a ChannelSenseRequest,
	 * i.e. sending a cMessage over the OMNeT-control-channel)
	 */
	virtual ChannelState getChannelState();

	/**
	 * @brief This function is called by the PhyLayer to hand over a
	 * ChannelSenseRequest.
	 *
	 * The MACLayer is able to send a ChannelSenseRequest to the PhyLayer
	 * that calls this function with it and is returned a time point when to
	 * re-call this function with the specific ChannelSenseRequest.
	 *
	 * The Decider puts the result (ChannelState) to the ChannelSenseRequest
	 * and "answers" by calling the "sendControlMsg"-function on the
	 * DeciderToPhyInterface, i.e. telling the PhyLayer to send it back.
	 */
	virtual simtime_t handleChannelSenseRequest(ChannelSenseRequest* request);

	/**
	 * @brief Method to be called by an OMNeT-module during its own finish(),
	 * to enable a decider to do some things.
	 */
	virtual void finish() {}

	/**
	 * @brief Called by phy layer to indicate that the channel this radio
	 * currently listens to has changed.
	 *
	 * Sub-classing deciders which support multiple channels should override
	 * this method to handle the effects of channel changes on ongoing
	 * receptions.
	 *
	 * @param newChannel The new channel the radio has changed to.
	 */
	virtual void channelChanged(int newChannel) {}

	/**
	 * @brief Notifies the decider that phy layer is starting a transmission.
	 *
	 * This helps the decider interrupting a current reception. In a standard
	 * 802.11 MAC, this should never happen, but in other MAC layers you might
	 * decide to interrupt an ongoing reception and start transmitting. Thank
	 * to this method, the decider can flag the ongoing frame as non received
	 * because of the transmission.
	 */
	virtual void switchToTx() {}

};


#endif /*DECIDER_H_*/
