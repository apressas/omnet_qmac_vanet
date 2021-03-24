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

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

const simsignalwrap_t BaseWaveApplLayer::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);

void BaseWaveApplLayer::initialize(int stage) {
								BaseApplLayer::initialize(stage);

								if (stage==0) {
																myMac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(
																								getParentModule());
																assert(myMac);

																myId = getParentModule()->getIndex();

																headerLength = par("headerLength").longValue();
																double maxOffset = par("maxOffset").doubleValue();
																sendBeacons = par("sendBeacons").boolValue();
																beaconLengthBits = par("beaconLengthBits").longValue();
																beaconPriority = par("beaconPriority").longValue();

																sendData = par("sendData").boolValue();
																dataLengthBits = par("dataLengthBits").longValue();
																dataOnSch = par("dataOnSch").boolValue();
																dataPriority = par("dataPriority").longValue();

																sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
																NoAckEvt = new cMessage("beacon evt", NO_ACK_EVT);

																beaconInterval = par("beaconInterval").doubleValue();


																//TWO APP TYPES
																//for 50 cars

//TODO 2 apps 256 - 1024byte

																//simulate asynchronous channel access
																double offSet = dblrand() * (beaconInterval/2);
																offSet = offSet + floor(offSet/0.050)*0.050;
																individualOffset = dblrand() * maxOffset;

																findHost()->subscribe(mobilityStateChangedSignal, this);

																originalBeacons=0;
																originalBeaconsTrained=0;

																trainTime = par("trainTime").doubleValue();
																txTime=0;
																seqNo=0;
																ack1=0;
																WATCH(ack1);
//if (myId==0){
																if(uniform(0,1)<main_app_retran_prob) { //first application
																								rateType=1;
//beaconInterval = 0.1;
																								//IN Single-hope
																								ACKprob=2/(main_app_retran_prob*neighbours);
																								//IN MULTIHOP
																								ACKprob = 6.0/neighbours;
																								//multihop rertransmission fixed probability
																								//ACKprob = 0.1;
																								scheduleAt(simTime() + offSet, sendBeaconEvt);

																								beaconLengthBits = 1744;
																								//beaconLengthBits = 720;
																}  //256bytes
																else{ //second application
																								rateType=2;
																								//beaconInterval = 0.05;
																								beaconLengthBits = 7884;
																								//beaconLengthBits = 3788;
																								ACKprob=2/((1-main_app_retran_prob)*neighbours);
																								scheduleAt(simTime() + offSet, sendBeaconEvt);
																} //1024bytes
								}
}
/*
   int BaseWaveApplLayer::combine(int a, int b) { //concatenate two ints for msgId
   int times = 1;
   while (times <= b)
      times *= 10;
   return a*times + b;
   }
 */

WaveShortMessage*  BaseWaveApplLayer::prepareWSM(std::string name, int lengthBits, t_channel channel, int priority, int rcvId, int serial) {
								WaveShortMessage* wsm =  new WaveShortMessage(name.c_str());
								wsm->addBitLength(headerLength);

								wsm->addBitLength(lengthBits);

								switch (channel) {
								case type_SCH: wsm->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
								case type_CCH: wsm->setChannelNumber(Channels::CCH); break;
								}
								wsm->setPsid(0);
								wsm->setPriority(priority);
								wsm->setWsmVersion(1);
								wsm->setTimestamp(simTime());
								wsm->setSenderAddress(myId);
								wsm->setRecipientAddress(rcvId);
								wsm->setSenderPos(curPosition);
								wsm->setSerial(serial);

								wsm->setTxTime(simTime().dbl());
								wsm->setRateType(rateType);
								//Make a message identification
								std::string IdStr = std::to_string(myId);
								std::string seqStr = std::to_string(seqNo);
								std::string MsgIdentification = IdStr+':'+seqStr;
								wsm->setMsgId(MsgIdentification.c_str()); //msgId+sequence for Quality Delivery Ratio

								cMsgId = MsgIdentification.c_str();

								if (name == "beacon") {
																DBG << "Creating Beacon with Priority " << priority << " at Applayer at " << wsm->getTimestamp() << std::endl;
								}
								if (name == "data") {
																DBG << "Creating Data with Priority " << priority << " at Applayer at " << wsm->getTimestamp() << std::endl;
								}

								return wsm;
}



void BaseWaveApplLayer::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
								Enter_Method_Silent();
								if (signalID == mobilityStateChangedSignal) {
																handlePositionUpdate(obj);
								}
}

void BaseWaveApplLayer::handlePositionUpdate(cObject* obj) {
								ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
								curPosition = mobility->getCurrentPosition();
}

void BaseWaveApplLayer::handleLowerMsg(cMessage* msg) {

								WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
								ASSERT(wsm);

								if (std::string(wsm->getName()) == "beacon") {
																onBeacon(wsm);
								}
								else if (std::string(wsm->getName()) == "data") {
																onData(wsm);
								}
								else {
																DBG << "unknown message (" << wsm->getName() << ")  received\n";
								}
								delete(msg);
}

void BaseWaveApplLayer::handleSelfMsg(cMessage* msg) {
								switch (msg->getKind()) {
								case SEND_BEACON_EVT: {

																sendWSM(prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1));
																seqNo++;
																//findHost()->getDisplayString().updateWith("r=15,red");  //RED : original messages
																if(simTime()>(startTime+trainTime)) {
																								originalBeaconsTrained++;
																}
																else{
																								originalBeacons++;
																}
																pdrbeacons++;


																txTime = simTime().dbl();

																scheduleAt(simTime() + beaconInterval, sendBeaconEvt);

																scheduleAt(simTime()+0.089, NoAckEvt); //schedule a NO ack event in xx ms (this is the timer <100ms)
																break;
								}
								case NO_ACK_EVT: {
																if (ack1==0) { //there was no ack received in 100ms;
																								//findHost()->getDisplayString().updateWith("r=15,yellow");  //no ACK for 100ms

																								WaveShortMessage* ctrl2=prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1);
																								ctrl2->setAcknowledged(0);
																								sendWSM(ctrl2);

																								rxTime = 0;
																								txTime = 0;
																								RTT = 0;

																}else if(ack1==1) {
																								ack1=0;
																}
																// NO ACKNOWLEDGEMENT SENT TILL NEW MESSAGE WAS CREATED
               if (ack1==0){
                       WaveShortMessage* ctrl2=prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1);
                       ctrl2->setAcknowledged(0);
                       sendWSM(ctrl2);
                }
                else if (ack1==1){
                                ack1=0;
                }

																break;
								}
								default: {
																if (msg)
																								DBG << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
																break;
								}
								}
}

void BaseWaveApplLayer::sendWSM(WaveShortMessage* wsm) {
								sendDelayedDown(wsm,individualOffset);
}

void BaseWaveApplLayer::finish() {
								if (sendBeaconEvt->isScheduled()) {
																cancelAndDelete(sendBeaconEvt);
								}
								else {
																delete sendBeaconEvt;
								}

								if (NoAckEvt->isScheduled()) {
																cancelAndDelete(NoAckEvt);
								}
								else {
																delete NoAckEvt;
								}

								findHost()->unsubscribe(mobilityStateChangedSignal, this);

}

BaseWaveApplLayer::~BaseWaveApplLayer() {

}
