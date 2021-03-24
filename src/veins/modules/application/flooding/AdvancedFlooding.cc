#include "AdvancedFlooding.h"

const simsignalwrap_t AdvancedFlooding::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

Define_Module(AdvancedFlooding);

void AdvancedFlooding::initialize(int stage) {
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

		sendData = par("sendData").boolValue();
		dataLengthBits = par("dataLengthBits").longValue();

		//simulate asynchronous channel access
		double offSet = dblrand() * (par("beaconInterval").doubleValue()/2);
		offSet = offSet + floor(offSet/0.050)*0.050;
		individualOffset = dblrand() * maxOffset;

		findHost()->subscribe(mobilityStateChangedSignal, this);

		generateMessageEvt = new cMessage("generate Message", GENERATE_MESSAGE);

		if(myId == 5) { // vehicle 218 is in the middle of the simulation scenario at 300s
			// TODO generate and broadcast (using generateMessageEvt) a message after the warmup period
		    scheduleAt(simTime()+1, generateMessageEvt);
		}
	}
}

FloodingMessage*  AdvancedFlooding::prepareWSM(std::string name, int lengthBits, t_channel channel, int priority, int rcvId, int serial) {
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

void AdvancedFlooding::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj) {
	Enter_Method_Silent();
	if (signalID == mobilityStateChangedSignal) {
		handlePositionUpdate(obj);
	}
}

void AdvancedFlooding::handlePositionUpdate(cObject* obj) {
	ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
	curPosition = mobility->getCurrentPosition();
}

void AdvancedFlooding::handleLowerMsg(cMessage* msg) {

	FloodingMessage* flm = dynamic_cast<FloodingMessage*>(msg);
	ASSERT(flm);

	if(flm->getSenderAddress()== 2) {
	    findHost()->getDisplayString().updateWith("r=16,green");
	    //sendWSM(flm);
		// TODO: here the magic happens ;-); ideally you need exactly 13 characters to implement simple flooding, maybe a lot more for the statistics and to avoid continuous rebroadcasts (advanced flooding)
	    //traci->commandSetColor(Veins::TraCIColor::fromTkColor("white"));
}
}

void AdvancedFlooding::handleSelfMsg(cMessage* msg) {
	switch (msg->getKind()) {
		case GENERATE_MESSAGE: {
			sendWSM(generateMessage());
			EV<<"SEND MESSAGE"<<endl;
			break;
		}
		default: {
			if (msg)
				DBG << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
			break;
		}
	}
}

bool AdvancedFlooding::sendWSM(FloodingMessage* wsm) {
//check whether sender address is correct.
    if(true) { // TODO check whether the message has not been sent before
        sendDelayedDown(wsm,individualOffset);
        return true;
    } else {
        delete wsm;
        return false;
    }
}

void AdvancedFlooding::finish() {
	// TODO Store scalars here
	findHost()->unsubscribe(mobilityStateChangedSignal, this);

}

FloodingMessage* AdvancedFlooding::generateMessage() {
    FloodingMessage* flm = prepareWSM("flm", dataLengthBits, type_CCH, beaconPriority, 0, -1);
    flm->setMsgNo(intuniform(0, 1000));
    //traci->commandSetColor(Veins::TraCIColor::fromTkColor("violet"));
    return flm;
}

AdvancedFlooding::~AdvancedFlooding() {

}
