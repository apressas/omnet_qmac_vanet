#ifndef FLOODING_H_
#define FLOODING_H_

#include <map>
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/utility/Consts80211p.h"
//#include <WaveShortMessage_m.h>
#include "veins/modules/messages/FloodingMessage_m.h"
#include "veins/base/connectionManager/ChannelAccess.h"
#include <veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h>

#include "veins/modules/mobility/traci/TraCIMobility.h"

#include "veins/collector/collector.h"  //counts the up-to-date nodes!!!

using Veins::TraCIMobility;
using Veins::AnnotationManager;

#ifndef DBG
#define DBG EV
#endif

class App : public BaseApplLayer {

	public:
		~App();
		virtual void initialize(int stage);
		virtual void finish();

		virtual  void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj);

		enum WaveApplMessageKinds {
			SERVICE_PROVIDER = LAST_BASE_APPL_MESSAGE_KIND,
			SEND_BEACON_EVT, GENERATE_MESSAGE
		};

	protected:

		TraCIMobility* traci;
		AnnotationManager* annotations;

		static const simsignalwrap_t mobilityStateChangedSignal;

		/** @brief handle messages from below */
		virtual void handleLowerMsg(cMessage* msg);
		/** @brief handle self messages */
		virtual void handleSelfMsg(cMessage* msg);

		virtual FloodingMessage* prepareWSM(std::string name, int dataLengthBits, t_channel channel, int priority, int rcvId, int serial=0);
		virtual bool sendWSM(FloodingMessage* wsm);

		virtual void handlePositionUpdate(cObject* obj);

		virtual FloodingMessage* generateMessage();

	protected:
		int beaconLengthBits;
		int beaconPriority;
		bool sendData;
		bool sendBeacons;
		simtime_t individualOffset;
		int dataLengthBits;
		bool dataOnSch;
		int dataPriority;
		Coord curPosition;
		int mySCH;
		int myId;

		long originalBeacons;


		cMessage* sendBeaconEvt;
		cMessage* generateMessageEvt;

		WaveAppToMac1609_4Interface* myMac;
};

#endif /* FLOODING_H_ */
