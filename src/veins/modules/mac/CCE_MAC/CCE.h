//
// Copyright (C) 2012 David Eckhoff <eckhoff@cs.fau.de>
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

#ifndef ___CCE_H_
#define ___CCE_H_

#include <assert.h>
#include <omnetpp.h>
#include "WaveAppToMac1609_4Interface.h"
#include <queue>
#include <stdint.h>
#include "veins/base/modules/BaseLayer.h"
#include "veins/base/phyLayer/MacToPhyControlInfo.h"
#include "veins/modules/phy/PhyLayer80211p.h"
#include "veins/modules/utility/Consts80211p.h"
#include "veins/base/utils/FindModule.h"
#include "veins/modules/messages/Mac80211Pkt_m.h"
#include "veins/modules/messages/WaveShortMessage_m.h"
#include "veins/base/modules/BaseMacLayer.h"

#include "veins/modules/utility/ConstsPhy.h"

#include <iostream>
#include <fstream>
/**
 * @brief
 * Manages timeslots for CCH and SCH listening and sending.
 *
 * @author David Eckhoff : rewrote complete model
 * @author Christoph Sommer : features and bug fixes
 * @author Michele Segata : features and bug fixes
 * @author Stefan Joerer : features and bug fixes
 * @author Christopher Saloman: initial version
 *
 * @ingroup macLayer
 *
 * @see BaseWaveApplLayer
 * @see Mac1609_4
 * @see PhyLayer80211p
 * @see Decider80211p
 */
class CCE : public BaseMacLayer,
	public WaveAppToMac1609_4Interface {

	public:

		enum t_access_category {
			AC_BK = 0,
			AC_BE = 1,
			AC_VI = 2,
			AC_VO = 3
		};

		class EDCA {
			public:
				class EDCAQueue {
					public:

						std::queue<WaveShortMessage*> queue;
						int aifsn; //number of aifs slots for this queue
						int cwMin; //minimum contention window
						int cwMax; //maximum contention size
						int cwCur; //current contention window
						int currentBackoff; //current Backoff value for this queue
						bool txOP;

						EDCAQueue() {	};
						EDCAQueue(int aifsn,int cwMin, int cwMax, t_access_category ac):aifsn(aifsn),cwMin(cwMin),cwMax(cwMax),cwCur(cwMin),currentBackoff(0),txOP(false) {
						};
				};

				EDCA(cModule *owner, t_channel channelType,int maxQueueLength = 0):owner(owner),numQueues(0),maxQueueSize(maxQueueLength),channelType(channelType) {
					statsNumInternalContention = 0;
					statsNumBackoff = 0;
					statsSlotsBackoff = 0;

				};
                const cObject *getThisPtr() const  {return NULL;}
                const char *getClassName() const {return "Mac1609_4::EDCA"; }
				/*
				 * Currently you have to call createQueue in the right order. First Call is priority 0, second 1 and so on...
				 */
				int createQueue(int aifsn, int cwMin, int cwMax,t_access_category);
				int queuePacket(t_access_category AC,WaveShortMessage* cmsg);
				void backoff(t_access_category ac);
				simtime_t startContent(simtime_t idleSince, bool guardActive);
				void stopContent(bool allowBackoff, bool generateTxOp);
				void postTransmit(t_access_category);
				void revokeTxOPs();

				void cleanUp();

				/** @brief return the next packet to send, send all lower Queues into backoff */
				WaveShortMessage* initiateTransmit(simtime_t idleSince);

			public:
				cModule *owner;
				std::map<t_access_category,EDCAQueue> myQueues;
				int numQueues;
				uint32_t maxQueueSize;
				simtime_t lastStart; //when we started the last contention;
				t_channel channelType;

				/** @brief Stats */
				long statsNumInternalContention;
				long statsNumBackoff;
				long statsSlotsBackoff;
				/** @brief Id for debug messages */
				std::string myId;

				long ContentionWindow;
				int multiplier=8;
		};

	public:
		~CCE() { };

		void changeServiceChannel(int channelNumber);

		/**
		 * @brief Change the default tx power the NIC card is using
		 *
		 * @param txPower_mW the tx power to be set in mW
		 */
		void setTxPower(double txPower_mW);

		/**
		 * @brief Change the default MCS the NIC card is using
		 *
		 * @param mcs the default modulation and coding scheme
		 * to use
		 */
		void setMCS(enum PHY_MCS mcs);

		/**
		 * @brief Change the phy layer carrier sense threshold.
		 *
		 * @param ccaThreshold_dBm the cca threshold in dBm
		 */
		void setCCAThreshold(double ccaThreshold_dBm);

	protected:
		/** @brief States of the channel selecting operation.*/

	protected:
		/** @brief Initialization of the module and some variables.*/
		virtual void initialize(int);

		/** @brief Delete all dynamically allocated objects of the module.*/
		virtual void finish();

		/** @brief Handle messages from lower layer.*/
		virtual void handleLowerMsg(cMessage*);

		/** @brief Handle messages from upper layer.*/
		virtual void handleUpperMsg(cMessage*);

		/** @brief Handle control messages from upper layer.*/
		virtual void handleUpperControl(cMessage* msg);


		/** @brief Handle self messages such as timers.*/
		virtual void handleSelfMsg(cMessage*);

		/** @brief Handle control messages from lower layer.*/
		virtual void handleLowerControl(cMessage* msg);

		/** @brief Set a state for the channel selecting operation.*/
		void setActiveChannel(t_channel state);

		simtime_t timeLeftInSlot() const;
		simtime_t timeLeftTillGuardOver() const;

		bool guardActive() const;

		void attachSignal(Mac80211Pkt* mac, simtime_t startTime, double frequency, uint64_t datarate, double txPower_mW);
		Signal* createSignal(simtime_t start, simtime_t length, double power, uint64_t bitrate, double frequency);

		/** @brief maps a application layer priority (up) to an EDCA access category. */
		t_access_category mapPriority(int prio);

		void channelBusy();
		void channelBusySelf(bool generateTxOp);
		void channelIdle(bool afterSwitch = false);

		void setParametersForBitrate(uint64_t bitrate);

		simtime_t getFrameDuration(int payloadLengthBits, enum PHY_MCS mcs = MCS_DEFAULT) const;

		void csvWrite(std::string filename,int A);
		double reward_function(int CW);

	protected:
		/** @brief Self message to indicate that the current channel shall be switched.*/
		cMessage* nextChannelSwitch;

		/** @brief Self message to wake up at next MacEvent */
		cMessage* nextMacEvent;

		/** @brief Last time the channel went idle */
		simtime_t lastIdle;
		simtime_t lastBusy;

		/** @brief Current state of the channel selecting operation.*/
		t_channel activeChannel;

		/** @brief access category of last sent packet */
		t_access_category lastAC;

		/** @brief Stores the frequencies in Hz that are associated to the channel numbers.*/
		std::map<int,double> frequency;

		int headerLength;

		bool useSCH;
		int mySCH;

		std::map<t_channel,EDCA*> myEDCA;

		bool idleChannel;

		/** @brief stats */
		long statsReceivedPackets;
		long statsReceivedBroadcasts;
		long statsSentPackets;
		long statsTXRXLostPackets;
		long statsSNIRLostPackets;
		long statsDroppedPackets;
		long statsNumTooLittleTime;
		long statsNumInternalContention;
		long statsNumBackoff;
		long statsSlotsBackoff;
		simtime_t statsTotalBusyTime;

		long CWMin;
		long CWMax;

		/** @brief This MAC layers MAC address.*/
		int myMacAddress;

		/** @brief The power (in mW) to transmit with.*/
		double txPower;

		/** @brief the bit rate at which we transmit */
		uint64_t bitrate;

		/** @brief N_DBPS, derived from bitrate, for frame length calculation */
		double n_dbps;

		/** @brief Id for debug messages */
		std::string myId;

		Mac80211pToPhy11pInterface* phy11p;

		//tell to anybody which is interested when the channel turns busy or idle
		simsignal_t sigChannelBusy;
		//tell to anybody which is interested when a collision occurred
		simsignal_t sigCollision;

		//MY STUFF for QLearn

		int ackReceived;
		int ackNotReceived;

        //int cw[7]={7,15,31,63,127,255,511};

		int cw[7]={3,7,15,31,63,127,255};

		int action[3]={0,1,2};

		//double Q[7][3];
		double Q[7][3]= {{-100, 0, 0} ,
                        {0, 0, 0} ,
                        {0, 0, 0},
                        {0, 0, 0} ,
                        {0, 0, 0} ,
                        {0, 0, 0} ,
                        {0, 0, -100}};

		/*
		double Q[7][3]={{-100, -2.69722, -1.74539 },
		{-2.23163, -2.36565 ,-2.07269 },
		{-2.1957, -2.25124 ,-1.84354 },
		{-1.66148, -2.34799, -2.04798 },
		{-2.18641, -2.55335, -2.25102 },
		{-2.48494, -2.64355, -2.59244 },
		{-2.52685, -2.41738, -100 }};
*/
		double gamma1=0.9; //discount rate was 0.7 before
		double alpha=0.5;  //learning rate
		double epsilon=1;

        double reward_delay[7]={1,0.86,0.71,0.57,0.42,0.286,0.14};
		//delay requirements
		double reward=0;
		int currentAction=0;
		int currentState=0;
		int ContentionWindow=3;
		double startTime=0;

		int cw_best=3;
		int CWnew=3;
		int explore=0;
        std::vector<int> vCW; //vector that contains 100 samples of CW of successful receipt transmissions.
        int counter;
        double reward_max=0.0;
        double lifetime=0;
        double trainTime;
        double converge_at_time=1000;
        double postCumulativeReward;
        double preCumulativeReward;

        std::vector<double> CWfrequencies={0, 0, 0, 0,0,0,0};
        std::vector<double> sortedfrequencies;
        double reward_CCE[7]={0.14,0.286,0.42,0.57,0.71,0.86,1};
        double cw_frequency=0;
        int i;
        int index=0;

        int myRateType=0;
        double recTime=0.0;
        int packetspersec=0;
        int window=0;
        double refresh=0.0;

        int maximum(int state, bool returnIndexOnly);

	//Where will the results be saved
        std::string directory = "/Volumes/LocalDataHD/a/ap/ap431/RESULTS/";

};

#endif /* ___MAC1609_4_H_*/
