//A.S.P.

#include "Dissemination.h"
#include "veins/modules/application/Dissemination/Dissemination.h"
#include <string>
#include <vector>  //vector definition
#include <algorithm> //vector operations

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t Dissemination::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

Define_Module(Dissemination);

void Dissemination::initialize(int stage) {
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

        rebroadSent=0;
        vMsg.clear();
        reduntant=0;
        HOPS=0;
//ADDITION FOR DISSEMINATION
        generateMessageEvt = new cMessage("generate Message", GENERATE_MESSAGE);

		if(myId == 2) { // vehicle 2 is in the middle of the simulation scenario at 300s
				// TODO generate and broadcast (using generateMessageEvt) a message after the warmup period
				scheduleAt(simTime(), generateMessageEvt);
				}

	}
}

void Dissemination::onBeacon(WaveShortMessage* wsm) { //when beacon is received from other nodes (->from lower layer)

    //findHost()->getDisplayString().updateWith("r=15,green");

    if (wsm->getIsRebroadcast()==false){ //if it is original message
        originalRec++;
    }
    else{
    rebroadRec++;
   }

    if(wsm->getHops()<1){
        if(uniform(0,1)<0.3){ //10% retransmission probability
            WaveShortMessage *copy = wsm->dup();
            copy->setIsRebroadcast(true);
            copy->setHops(wsm->getHops()+1);
            HOPS = copy->getHops();
            WATCH(HOPS);
            sendWSM(copy);
            findHost()->getDisplayString().updateWith("r=15,blue"); //BLUE: RETRANSMIT
            rebroadSent++;
           // WATCH(copy->getHops());
        }
    }

//Percentage of quality information received by the nodes
    if (std::find(vMsg.begin(),vMsg.end(),wsm->getMsgId()) != vMsg.end()){
        findHost()->bubble("found it"); //did it find myId in vector?
        findHost()->getDisplayString().updateWith("r=15,green");  //GREEN : FOUND IT ALREADY
        reduntant++;
        //TODO mistake here, it should never enter this loop without rebroadcasting!!
    }
    else{
        vMsg.push_back(wsm->getMsgId()); //ADD IT!!
        //findHost()->getDisplayString().updateWith("r=15,pink");
    }
}

void Dissemination::onData(WaveShortMessage* wsm) {
	findHost()->getDisplayString().updateWith("r=16,green");
	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobility->getPositionAt(simTime()), "blue"));

	if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getWsmData(), 9999);
	if (!sentMessage) sendMessage(wsm->getWsmData());
}

void Dissemination::sendMessage(std::string blockedRoadId) {
	sentMessage = true;

	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
	wsm->setWsmData(blockedRoadId.c_str());
	sendWSM(wsm);
}


void Dissemination::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
	/*Enter_Method_Silent();
	if (signalID == mobilityStateChangedSignal) {
		handlePositionUpdate(obj);
	}
	else if (signalID == parkingStateChangedSignal) {
		handleParkingUpdate(obj);
	}*/
}
void Dissemination::handleParkingUpdate(cObject* obj) {
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
void Dissemination::handlePositionUpdate(cObject* obj) {
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
void Dissemination::sendWSM(WaveShortMessage* wsm) {
	//if (isParking && !sendWhileParking) return;
	sendDelayedDown(wsm,individualOffset);
}

void Dissemination::finish(){
    recordScalar("RebroadSent",rebroadSent);
    recordScalar("originalSent",originalBeacons);
    recordScalar("originalReceived",originalRec);
    recordScalar("RebroadReceived",rebroadRec);
    recordScalar("QualityReceived",vMsg.size());
    recordScalar("ReduntantPackets",reduntant);
}
