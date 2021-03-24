//DISSEMINATION DELAY SIMULATION

#include "App.h"
#include <string>
#include <vector>  //vector definition
#include <algorithm> //vector operations

const simsignalwrap_t App::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

Define_Module(App);

void App::initialize(int stage) {
	BaseApplLayer::initialize(stage);

	if (stage==0) {
		myMac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(
		            getParentModule());
		assert(myMac);

		myId = getParentModule()->getIndex();

		traci = TraCIMobilityAccess().get(getParentModule());
		annotations = AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);

		headerLength = par("headerLength").longValue();
		double maxOffset = par("maxOffset").doubleValue();
		beaconLengthBits = par("beaconLengthBits").longValue();
		beaconPriority = par("beaconPriority").longValue();

		sendBeacons = par("sendBeacons").boolValue();
		sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
 //BEACONING FFS!!

		sendData = par("sendData").boolValue();
		dataLengthBits = par("dataLengthBits").longValue();

		//simulate asynchronous channel access
		double offSet = dblrand() * (par("beaconInterval").doubleValue()/2);
		offSet = offSet + floor(offSet/0.050)*0.050;
		individualOffset = dblrand() * maxOffset;

		if (sendBeacons) {
			scheduleAt(simTime() + offSet, sendBeaconEvt);
		}

		findHost()->subscribe(mobilityStateChangedSignal, this);

		generateMessageEvt = new cMessage("generate Message", GENERATE_MESSAGE);

		if(myId == 8) { // vehicle 218 is in the middle of the simulation scenario at 300s
			// TODO generate and broadcast (using generateMessageEvt) a message after the warmup period
		    scheduleAt(simTime(), generateMessageEvt);
		}
	}
}

FloodingMessage*  App::prepareWSM(std::string name, int lengthBits, t_channel channel, int priority, int rcvId, int serial) {
    FloodingMessage* flm =		new FloodingMessage(name.c_str());
    flm->addBitLength(headerLength);
    flm->addBitLength(lengthBits);

	switch (channel) {
		case type_SCH: flm->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
		case type_CCH: flm->setChannelNumber(Channels::CCH); break;
	}

	flm->setPsid(0);
	flm->setPriority(priority);
	flm->setWsmVersion(1);
	flm->setTimestamp(simTime());
	flm->setSenderAddress(myId);
	flm->setRecipientAddress(rcvId);
	flm->setSenderPos(curPosition);
	flm->setSerial(serial);

	if (name == "beacon") {
		DBG << "Creating Beacon with Priority " << priority << " at Applayer at " << flm->getTimestamp() << std::endl;
	}
	if (name == "data") {
		DBG << "Creating Data with Priority " << priority << " at Applayer at " << flm->getTimestamp() << std::endl;
	}
	return flm;
}

void App::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj) {
	Enter_Method_Silent();
	if (signalID == mobilityStateChangedSignal) {
		handlePositionUpdate(obj);
	}
}

void App::handlePositionUpdate(cObject* obj) {
	ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
	curPosition = mobility->getCurrentPosition();
}

void App::handleLowerMsg(cMessage* msg) {
    // TODO: here the magic happens ;-); ideally you need exactly 13 characters to implement simple flooding, maybe a lot more for the statistics and to avoid continuous rebroadcasts (advanced flooding)

	FloodingMessage* flm = dynamic_cast<FloodingMessage*>(msg);
	ASSERT(flm);

	if(flm->getSenderAddress() == 8) { //check whether its the right type of message
	    findHost()->getDisplayString().updateWith("r=15,green");
	    //REBROADCAST
	    FloodingMessage *copy = flm->dup();
	    sendWSM(copy);
        //findHost()->bubble("rcved");
	    //check whether this node has gotten the msg before
	        if(Collector::vNodes.size()>99){
	             findHost()->bubble("ALL OF THEM GOT IT!!!!");
	             endSimulation();
	         }
	         //add this node's id (myId) to the Collector vector
	         if (std::find(Collector::vNodes.begin(),Collector::vNodes.end(),myId) != Collector::vNodes.end()){ //use the getters to access vector in Message Class (for security)
	             findHost()->bubble("found it"); //did it find myId in vector?
	             findHost()->getDisplayString().updateWith("r=15,red");
	         }
	         else{ //if this iD is found here
	             Collector::vNodes.push_back(myId);
	             findHost()->bubble("ADDED");
	             findHost()->getDisplayString().updateWith("r=15,green");
	         }
	         WATCH_VECTOR(Collector::vNodes);
	    //traci->commandSetColor(Veins::TraCIColor::fromTkColor("white"));
	}
	delete(flm);
}

void App::handleSelfMsg(cMessage* msg) {
	switch (msg->getKind()) {
	    //PRIORITY MESSAGE
		case GENERATE_MESSAGE: {
			sendWSM(generateMessage());
			EV<<"SEND MESSAGE"<<endl;
			break;
		}
		//PERIODIC BEACONING
		case SEND_BEACON_EVT: {
		   sendWSM(prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1));
		   //seqNo++;
		   findHost()->getDisplayString().updateWith("r=15,yellow");
		   //yellow: ORIGINAL BEACONS
		   originalBeacons++;
		   scheduleAt(simTime() + par("beaconInterval").doubleValue(), sendBeaconEvt);
		    break;
		 }
		default: {
			if (msg)
				DBG << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
			break;
		}
	}
}

bool App::sendWSM(FloodingMessage* wsm) {
//check whether sender address is correct.
    if(true) {
			// TODO check whether the message has not been sent before
        sendDelayedDown(wsm,individualOffset);
        return true;
    } else {
        delete wsm;
        return false;
    }
}

void App::finish() {
	// TODO Store scalars here
	findHost()->unsubscribe(mobilityStateChangedSignal, this);
}

FloodingMessage* App::generateMessage() {
    FloodingMessage* flm = prepareWSM("flm", dataLengthBits, type_CCH, beaconPriority, 0, -1);
    flm->setMsgNo(intuniform(0, 1000));
    flm->setSenderAddress(myId);
    //traci->commandSetColor(Veins::TraCIColor::fromTkColor("violet"));
    return flm;
}

App::~App() {
}
