/* -*- mode:c++ -*- *******************************************************
 * file:        NetwToMacControlInfo.h
 *
 * author:      Daniel Willkomm
 *
 * copyright:   (C) 2005 Telecommunication Networks Group (TKN) at
 *              Technische Universitaet Berlin, Germany.
 *
 *              This program is free software; you can redistribute it
 *              and/or modify it under the terms of the GNU General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later
 *              version.
 *              For further information see file COPYING
 *              in the top level directory
 **************************************************************************
 * part of:     framework implementation developed by tkn
 * description: - control info to pass next hop to the MAC layer
 **************************************************************************/

#ifndef NETWTOMACCONTROLINFO_H
#define NETWTOMACCONTROLINFO_H

#include <omnetpp.h>

#include "veins/base/utils/MiXiMDefs.h"
#include "veins/base/utils/SimpleAddress.h"

/**
 * @brief Control info to pass next hop L2 addr from netw to MAC layer
 *
 * Control Info to pass interface information from the network to the
 * MAC layer Currently the only information necessary is the MAC
 * address of the next hop, which has to be determined by ARP or some
 * similar mechanism
 *
 * @ingroup baseUtils
 * @ingroup macLayer
 * @ingroup netwLayer
 * @author Daniel Willkomm
 **/
class MIXIM_API NetwToMacControlInfo : public cObject
{
  protected:
    /** @brief MAC address of the sending or receiving node*/
	LAddress::L2Type nextHopMac;

  public:
    /** @brief Default constructor*/
    NetwToMacControlInfo(const LAddress::L2Type& addr = LAddress::L2NULL())
      : cObject()
      , nextHopMac(addr)
    {
    };

    /** @brief Destructor*/
    virtual ~NetwToMacControlInfo() {};

    /** @brief Getter method */
    virtual const LAddress::L2Type& getNextHopMac() const {
    	return nextHopMac;
    };
    virtual const LAddress::L2Type& getDest() const {
    	return nextHopMac;
    };

    /** @brief Setter method */
    virtual void setNextHopMac(const LAddress::L2Type& addr) {
    	nextHopMac = addr;
    };
    virtual void setDest(const LAddress::L2Type& addr) {
    	nextHopMac = addr;
    };

    /**
     * @brief Attaches a "control info" structure (object) to the message pMsg.
     *
     * This is most useful when passing packets between protocol layers
     * of a protocol stack, the control info will contain the destination MAC address.
     *
     * The "control info" object will be deleted when the message is deleted.
     * Only one "control info" structure can be attached (the second
     * setL3ToL2ControlInfo() call throws an error).
     *
     * @param pMsg		The message where the "control info" shall be attached.
     * @param pDestAddr	The MAC address of the message receiver.
     */
    static cObject *const setControlInfo(cMessage *const pMsg, const LAddress::L2Type& pDestAddr);
    /**
     * @brief Extracts the MAC address from the "control info" structure (object).
     *
     * Extract the destination MAC address from the "control info" which was prev. set by NetwToMacControlInfo::setControlInfo().
     *
     * @param pCtrlInfo	The "control info" structure (object) prev. set by NetwToMacControlInfo::setControlInfo().
     * @return The MAC address of message receiver.
     */
    static const LAddress::L2Type& getDestFromControlInfo(const cObject *const pCtrlInfo);
};


#endif
