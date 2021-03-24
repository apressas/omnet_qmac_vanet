//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
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
#include "EvalApp.h"

#include "veins/modules/application/EvalApp/EvalApp.h"
#include "veins/modules/mac/qMAC/qMAC.h"
using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t EvalApp::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

Define_Module(EvalApp);

void EvalApp::initialize(int stage) {
								BaseWaveApplLayer::initialize(stage);
								if (stage == 0) {
																mobility = TraCIMobilityAccess().get(getParentModule());
																traci = mobility->getCommandInterface();
																traciVehicle = mobility->getVehicleCommandInterface();
																annotations = AnnotationManagerAccess().getIfExists();
																ASSERT(annotations);

																sentMessage = false;
																lastDroveAt = simTime();
																findHost()->subscribe(parkingStateChangedSignal, this);
																isParking = false;
																sendWhileParking = par("sendWhileParking").boolValue();

																myMac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(
																								getParentModule());
																assert(myMac);
																myId = getParentModule()->getIndex(); //this node's ID

																//ADDITIONS
																rebroadSent=0;
																rebroadRec=0;
																originalRec=0;
																originalMultiRec=0;
																originalBeacons=0;

																ACK=0;
																RxTrained=0;
																RxMultiTrained=0;
																reduntantACK=0;
																vACK.clear();
																vUnique.clear();
																Vvehicle.clear();

																WATCH_VECTOR(vACK);

																rec_time = simTime().dbl()+150;
																rec_time1= simTime().dbl()+150;
																rec_timeTP= simTime().dbl()+150;
																rec_timeVehicles= simTime().dbl()+150;


																startTime = simTime().dbl() + 150;
																trainTime = par("trainTime").doubleValue();
																cars = par("cars").doubleValue();
																txTime = 0;
																rxTime = 0;
																RTT = 0;
																lifetime = 0;
																throughputA=0;
																throughputB=0;
																throughput=0;
																ackowledgements=0;
																unique=0;


//directory = par("appsaves").str();

								}
}

void EvalApp::onBeacon(WaveShortMessage* wsm) { //when beacon is received from other nodes (->from lower layer)

								delay = simTime().dbl() - wsm->getTxTime();

								//record unique packets from single-multi-hop paths
								//if (!(wsm->getMsgId()==cMsgId)){ //do not consider the packets I sent! originally came from me?
								//IF MSGID FOUND IN VECTOR
								//if(delay<0.1){

								if(wsm->getHops()==0) {
																if(std::find(Vvehicle.begin(),Vvehicle.end(),wsm->getSenderAddress()) !=Vvehicle.end()) {
																}
																else{
																								Vvehicle.push_back(wsm->getSenderAddress());
																}
								}

								//NEIGHBOURS CALCULATION
								if(simTime().dbl()>rec_timeVehicles+0.5) { //every 500ms
																neighbours = Vvehicle.size();
																rec_timeVehicles = simTime().dbl();
																std::ofstream myfile59;
																myfile59.open (directory + "neighbours_" + std::to_string(myId) +".csv",std::ios_base::app);
																myfile59<<Vvehicle.size()<<","<<ACKprob<<","<<rec_timeVehicles<<std::endl;
																myfile59.close();
																Vvehicle.clear();
								}


								//recording of unique packets for multihop

								if(std::find(vUnique.begin(),vUnique.end(),wsm->getMsgId()) != vUnique.end()) {
																//do nothing, already received the packet
																//copies_received++;
								}
								//IF MSGID NOT FOUND IN VECTOR, new packet arrived
								else{
																vUnique.push_back(wsm->getMsgId());      //ADD reception

																unique_received++;

																//do not leave the vector be large because simulation time increases.


																if (vUnique.size()>10000) {     //includes the IDs of the latest packets, does not need older ones. this loop speeds up simulation.

																								for (int i=0; i<1000; i++) {
																																vUnique.pop_back();
																								}
																}


																if(myId==30) {

																								//fairness multihop

																								for (int carId=0; carId<100; carId++) {
																																if (wsm->getSenderAddress()==carId) {
																																								tp_per_vehicle[carId]++;

																																}
																								}
																}



																//delay multihop
																if((myId==30) && (wsm->getExplore()==0)) {
																								delayunq = simTime().dbl() - wsm->getTxTime();

																								std::ofstream myfile32;
																								myfile32.open(directory + "delayunq_" + std::to_string(myId) +".csv",std::ios_base::app);
																								myfile32<<delayunq<<","<<simTime().dbl()<<","<<wsm->getRateType()<<","<<wsm->getHops()<<std::endl;
																								myfile32.close();
																}


								}



								//goodput for trained
								throughput++;

								if(wsm->getRateType()==1) {
																throughputA++;
								}
								else if(wsm->getRateType()==2) {
																throughputB++;
								}

/*
   //RECORD LATENCY FROM APP2APP
    latency=simTime().dbl()-(wsm->getTxTime());
    std::ofstream myfile12;
    myfile12.open (directory + "latency.csv",std::ios_base::app);
    myfile12<<latency<<","<<simTime().dbl()<<std::endl;
    myfile12.close();
    latency = 0;
 */

//TODO THROUGHPUT RECORDING for Fairness

								//for 1 node only
/*
    if(myId==30){


     for (int carId=0; carId<vehicles; carId++){
      if (wsm->getSenderAddress()==carId){
        tp_per_vehicle[carId]++;

      }
     }
     }
 */
/*
     delay = simTime().dbl() - wsm->getTxTime();

     std::ofstream myfile55;
     myfile55.open(directory + "delay_" + std::to_string(myId) +".csv",std::ios_base::app);
     myfile55<<delay<<","<<rec_timeTP<<","<<wsm->getRateType()<<std::endl;
     myfile55.close();
 */

								//record delay for every packet, either from single or multihop paths

								if(myId==30) {

																std::ofstream myfile55;
																myfile55.open(directory + "delay_" + std::to_string(myId) +".csv",std::ios_base::app);
																myfile55<<delay<<","<<simTime().dbl()<<","<<wsm->getRateType()<<","<<wsm->getHops()<<std::endl;
																myfile55.close();


																if(simTime().dbl()>rec_timeTP+0.5) { //every 500ms
																								rec_timeTP = simTime().dbl();
																								for (int carId=0; carId<100; carId++) {
																																std::ofstream myfile5;
																																myfile5.open (directory + "TP_" + std::to_string(carId) +".csv",std::ios_base::app);
																																myfile5<<tp_per_vehicle[carId]<<","<<rec_timeTP<<std::endl;
																																myfile5.close();
																								}
																}
								}



								if(simTime().dbl()>rec_time+1) { //every 1s

																//throughput for APP A
																rec_time = simTime().dbl();
																std::ofstream myfile2;
																myfile2.open (directory + "Athroughput_" + std::to_string(myId) +".csv",std::ios_base::app);
																myfile2<<throughputA<<","<<rec_time<<std::endl;
																myfile2.close();

																//throughput for APP B
																std::ofstream myfile3;
																myfile3.open (directory + "Bthroughput_" + std::to_string(myId) +".csv",std::ios_base::app);
																myfile3<<throughputB<<","<<rec_time<<std::endl;
																myfile3.close();
/*
    if((par("learn").longValue()==1 && simTime()>startTime+trainTime) || (par("learn").longValue()==0)){
        throughput++;
    }
 */
																//RECORD throughput
																std::ofstream myfile4;
																myfile4.open (directory + "throughput_" + std::to_string(myId) +".csv",std::ios_base::app);
																myfile4<<throughput<<","<<rec_time<<std::endl;
																myfile4.close();

																//RECORD goodput
																std::ofstream myfile5;
																myfile5.open (directory + "unique_rec_" + std::to_string(myId) +".csv",std::ios_base::app);
																myfile5<<unique_received<<","<<rec_time<<std::endl;
																myfile5.close();

								}

								/*
								   //IF MSGID NOT FOUND
								   if(!(std::find(vUnique.begin(),vUnique.end(),wsm->getMsgId()) != vUnique.end())){
								    //ADD RECEPTION
								    vUnique.push_back(wsm->getMsgId());
								 */

								if(simTime()>(startTime+trainTime)) {
																RxTrained++;
								}
								else{         // RECORD
																originalRec++;
								}



								if (wsm->getIsRebroadcast()==false && wsm->getRateType()==rateType) { //if it is ORIGINAL message//SAME APP ONLY RETRANSMISSIONS.

																// RETRANSMIT
																//TODO ONLY IN MULTIHOP
																ACKprob = 6.0/neighbours;

																if(uniform(0,1)<ACKprob) { //10% retransmission probability
																								WaveShortMessage *copy = wsm->dup();
																								copy->setIsRebroadcast(true);
																								copy->setHops(copy->getHops()+1);
																								//copy->setRateType(rateType);
																								sendWSM(copy);
																								//findHost()->getDisplayString().updateWith("r=15,blue");
																								rebroadSent++;
																}
								}
								//ELSE IF RETRANSMISSION
								else if(wsm->getIsRebroadcast()==true) { //if it is rebroadcast
																rebroadRec++;

																if (wsm->getMsgId()==cMsgId) { //it originally came from me?
																								//IF MSGID FOUND IN VECTOR
																								if(std::find(vACK.begin(),vACK.end(),wsm->getMsgId()) != vACK.end()) {
																																reduntantACK++;
																								}
																								//IF MSGID NOT FOUND IN VECTOR
																								else{

																																vACK.push_back(wsm->getMsgId()); //ADD ACK

																																//do not leave the vector be large because simulation time increases.


																																if (vACK.size()>500) { //includes the IDs of the latest packets, does not need older ones.
																																								for (int i=0; i<100; i++) {                                                                                                 vACK.pop_back();}
																																}


																																//this is the first ACK
																																//record time instant for RTT
																																rxTime = simTime().dbl();
																																RTT = rxTime-txTime;

																																lifetime = simTime().dbl() - startTime;
																																if (RTT <1) {
																																								if(par("learn").longValue()==0) {
																																																std::ofstream myfile2;

																																																//myfile1.open (directory + std::to_string(myId) + "delay.csv",std::ios_base::app);
																																																myfile2.open (directory + "delay.csv",std::ios_base::app);

																																																myfile2<<RTT<<","<<cars<<","<<simTime()<<","<<rxTime<<std::endl;
																																																myfile2.close();
																																																rxTime = 0;
																																																txTime = 0;
																																																RTT = 0;
																																								}
																																								else if(simTime().dbl()>(startTime+trainTime)) {
																																																std::ofstream myfile2;

																																																//myfile1.open (directory + std::to_string(myId) + "delay.csv",std::ios_base::app);
																																																myfile2.open (directory + "delay.csv",std::ios_base::app);
																																																myfile2<<RTT<<","<<rateType<<","<<cars<<","<<simTime()<<std::endl;
																																																myfile2.close();
																																																rxTime = 0;
																																																txTime = 0;
																																																RTT = 0;
																																								}
																																}
																																//findHost()->getDisplayString().updateWith("r=15,yellow");  //RED : original messages
																																//goodput!
																																ACK++;

																																if(simTime().dbl()>rec_time1+1) { //every 1s
																																								std::ofstream myfile2;
																																								//myfile1.open (directory + std::to_string(myId) + "delay.csv",std::ios_base::app);
																																								myfile2.open (directory + "ACK_" + std::to_string(myId) +".csv",std::ios_base::app);
																																								myfile2<<ACK<<","<<rateType<<","<<rec_time1<<std::endl;
																																								myfile2.close();
																																								rec_time1 = simTime().dbl();
																																}

																																//CONTROL MESSAGES ABOUT THE ACKNOWLEDGEMENTS - FEEDBACK. ACK

																																WaveShortMessage *ctrl = wsm->dup();
																																ctrl->setAcknowledged(1);
																																sendWSM(ctrl);
																																ack1=1;
																																//delete(ctrl);
																								}
																}
								}
								//ackowledgements = vACK.size();
								//unique = vUnique.size();
}
/*
    if(uniform(0,1)<0.001){
        //cars = manager->getManagedHosts().size();
        PDR = (pdrreceived/double(cars))/double(pdrbeacons);
        //record pdr/cars over time
        std::ofstream myfile1;
        myfile1.open (directory  + "PDR_" + std::to_string(myId)+".csv",std::ios_base::app);
        myfile1<<PDR<<","<<cars<<","<<simTime()<<std::endl;
        myfile1.close();
        //findHost()->bubble("rcved");
        //delete(wsm);

        //RESET VALUES WITH 0.5% probability.
        pdrreceived=0;
        pdrbeacons=0;
    }
 */

void EvalApp::onData(WaveShortMessage* wsm) {
								findHost()->getDisplayString().updateWith("r=16,green");
								annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobility->getPositionAt(simTime()), "blue"));

								if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getWsmData(), 9999);
								if (!sentMessage) sendMessage(wsm->getWsmData());
}

void EvalApp::sendMessage(std::string blockedRoadId) {
								sentMessage = true;

								t_channel channel = dataOnSch ? type_SCH : type_CCH;
								WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
								wsm->setWsmData(blockedRoadId.c_str());
								sendWSM(wsm);
}

void EvalApp::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
								/*Enter_Method_Silent();
								   if (signalID == mobilityStateChangedSignal) {
								   handlePositionUpdate(obj);
								   }
								   else if (signalID == parkingStateChangedSignal) {
								   handleParkingUpdate(obj);
								   }*/
}
void EvalApp::handleParkingUpdate(cObject* obj) {
								/*isParking = mobility->getParkingState();
								   if (sendWhileParking == false) {
								   if (isParking == true) {
								   (FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
								   }
								   else {
								   Coord pos = mobility->getCurrentPosition();
								   (FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
								   }
								   }*/
}
void EvalApp::handlePositionUpdate(cObject* obj) {
								BaseWaveApplLayer::handlePositionUpdate(obj);
/*
   // stopped for for at least 10s?
   if (mobility->getSpeed() < 1) {
   if (simTime() - lastDroveAt >= 10) {
   findHost()->getDisplayString().updateWith("r=16,green");
   if (!sentMessage) sendMessage(mobility->getRoadId());
   }
   }
   else {
   lastDroveAt = simTime();
   }*/
}
void EvalApp::sendWSM(WaveShortMessage* wsm) {
								//if (isParking && !sendWhileParking) return;
								sendDelayedDown(wsm,individualOffset);
}

void EvalApp::csvWrite(std::string filename,int A){
								std::ofstream myfile;
								myfile.open (filename,std::ios_base::app);
								myfile<<A<<std::endl;
								myfile.close();
}

void EvalApp::finish(){

								csvWrite(directory+"ACK.csv",ACK);
//csvWrite(directory+"ACK.csv",rateType);

								csvWrite(directory+"OriginalSent.csv",originalBeacons);
								csvWrite(directory+"OriginalMultiRec.csv",originalMultiRec);

								csvWrite(directory+"OriginalReceived.csv",originalRec);
								csvWrite(directory+"Original_packets_after_trained.csv",originalBeaconsTrained);
								csvWrite(directory+"RxTrained.csv",RxTrained);
								csvWrite(directory+"RxMultiTrained.csv",RxMultiTrained);

								csvWrite(directory+"RebroadSent.csv",rebroadSent);

								csvWrite(directory+"throughput.csv",throughput);

								csvWrite(directory+"rebroad_rec.csv",rebroadRec);
								csvWrite(directory+"vector.csv",ackowledgements);
								csvWrite(directory+"unique.csv",unique);

								recordScalar("RebroadSent",rebroadSent);
								recordScalar("originalSent",originalBeacons);
								recordScalar("originalReceived",originalRec);
								recordScalar("RebroadReceived",rebroadRec);
								recordScalar("ACKs",ACK);
								recordScalar("ReduntantACKs",reduntantACK);

}
