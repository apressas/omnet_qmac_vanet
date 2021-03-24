#include "qMAC.h"
#include <iterator>
#include <map>
#include "veins/modules/phy/DeciderResult80211.h"
#include "veins/base/phyLayer/PhyToMacControlInfo.h"
#include "veins/modules/messages/PhyControlMessage_m.h"
#include <math.h>       /* exp */

#if OMNETPP_VERSION >= 0x500
#define OWNER owner->
#else
#define OWNER
#endif

#define DBG_MAC EV
//#define DBG_MAC std::cerr << "[" << simTime().raw() << "] " << myId << " "

Define_Module(qMAC);

void qMAC::initialize(int stage) {
	BaseMacLayer::initialize(stage);
	if (stage == 0) {

		phy11p = FindModule<Mac80211pToPhy11pInterface*>::findSubModule(
		             getParentModule());
		assert(phy11p);

		//this is required to circumvent double precision issues with constants from CONST80211p.h
		assert(simTime().getScaleExp() == -12);

		sigChannelBusy = registerSignal("sigChannelBusy");
		sigCollision = registerSignal("sigCollision");

		CWMin = par("CWmin").longValue();
		CWMax = par("CWmax").longValue();

		txPower = par("txPower").doubleValue();
		bitrate = par("bitrate").longValue();
		n_dbps = 0;
		setParametersForBitrate(bitrate);

		//mac-adresses
		myMacAddress = intuniform(0,0xFFFFFFFE);
		myId = getParentModule()->getParentModule()->getFullPath();
		//create frequency mappings
		frequency.insert(std::pair<int, double>(Channels::CRIT_SOL, 5.86e9));
		frequency.insert(std::pair<int, double>(Channels::SCH1, 5.87e9));
		frequency.insert(std::pair<int, double>(Channels::SCH2, 5.88e9));
		frequency.insert(std::pair<int, double>(Channels::CCH, 5.89e9));
		frequency.insert(std::pair<int, double>(Channels::SCH3, 5.90e9));
		frequency.insert(std::pair<int, double>(Channels::SCH4, 5.91e9));
		frequency.insert(std::pair<int, double>(Channels::HPPS, 5.92e9));

		//create two edca systems
		myEDCA[type_CCH] = new EDCA(this, type_CCH,par("queueSize").longValue());
		myEDCA[type_CCH]->myId = myId;
		myEDCA[type_CCH]->myId.append(" CCH");
		myEDCA[type_CCH]->createQueue(2,(((CWMin+1)/4)-1),(((CWMin +1)/2)-1),AC_VO);
		myEDCA[type_CCH]->createQueue(3,(((CWMin+1)/2)-1),CWMin,AC_VI);
		myEDCA[type_CCH]->createQueue(6,CWMin,CWMax,AC_BE);
		myEDCA[type_CCH]->createQueue(9,CWMin,CWMax,AC_BK);

		myEDCA[type_SCH] = new EDCA(this, type_SCH,par("queueSize").longValue());
		myEDCA[type_SCH]->myId = myId;
		myEDCA[type_SCH]->myId.append(" SCH");
		myEDCA[type_SCH]->createQueue(2,(((CWMin+1)/4)-1),(((CWMin +1)/2)-1),AC_VO);
		myEDCA[type_SCH]->createQueue(3,(((CWMin+1)/2)-1),CWMin,AC_VI);
		myEDCA[type_SCH]->createQueue(6,CWMin,CWMax,AC_BE);
		myEDCA[type_SCH]->createQueue(9,CWMin,CWMax,AC_BK);

		useSCH = par("useServiceChannel").boolValue();
		if (useSCH) {
			//set the initial service channel
			switch (par("serviceChannel").longValue()) {
				case 1: mySCH = Channels::SCH1; break;
				case 2: mySCH = Channels::SCH2; break;
				case 3: mySCH = Channels::SCH3; break;
				case 4: mySCH = Channels::SCH4; break;
				default: throw cRuntimeError("Service Channel must be between 1 and 4"); break;
			}
		}

		headerLength = par("headerLength");

		nextMacEvent = new cMessage("next Mac Event");

		if (useSCH) {
			// introduce a little asynchronization between radios, but no more than .3 milliseconds
			uint64_t currenTime = simTime().raw();
			uint64_t switchingTime = SWITCHING_INTERVAL_11P.raw();
			double timeToNextSwitch = (double)(switchingTime
							   - (currenTime % switchingTime)) / simTime().getScale();
			if ((currenTime / switchingTime) % 2 == 0) {
				setActiveChannel(type_CCH);
			}
			else {
				setActiveChannel(type_SCH);
			}

			// channel switching active
			nextChannelSwitch = new cMessage("Channel Switch");
			simtime_t offset = dblrand() * par("syncOffset").doubleValue();
			scheduleAt(simTime() + offset + timeToNextSwitch, nextChannelSwitch);
		}
		else {
			// no channel switching
			nextChannelSwitch = 0;
			setActiveChannel(type_CCH);
		}

		//stats
		statsReceivedPackets = 0;
		statsReceivedBroadcasts = 0;
		statsSentPackets = 0;
		statsTXRXLostPackets = 0;
		statsSNIRLostPackets = 0;
		statsDroppedPackets = 0;
		statsNumTooLittleTime = 0;
		statsNumInternalContention = 0;
		statsNumBackoff = 0;
		statsSlotsBackoff = 0;
		statsTotalBusyTime = 0;

		idleChannel = true;
		lastBusy = simTime();
		channelIdle(true);

		ackReceived=0;
		ackNotReceived=0;

		ContentionWindow=((CWMin+1)/4)-1;
        startTime = simTime().dbl();
        trainTime = par("trainTime").doubleValue();

        postCumulativeReward = 0;
        preCumulativeReward =0;

        //directory = par("macsaves").str();

		//initialize Q table
/*
		for(int count1=0; count1<3; count1++){
		    for(int count = 0; count < 7; count ++){
		           Q[count][count1] = 0 ;
		        }
		}
*/


}
}

void qMAC::handleSelfMsg(cMessage* msg) {
	if (msg == nextChannelSwitch) {
		ASSERT(useSCH);

		scheduleAt(simTime() + SWITCHING_INTERVAL_11P, nextChannelSwitch);

		switch (activeChannel) {
			case type_CCH:
				DBG_MAC << "CCH --> SCH" << std::endl;
				channelBusySelf(false);
				setActiveChannel(type_SCH);
				channelIdle(true);
				phy11p->changeListeningFrequency(frequency[mySCH]);
				break;
			case type_SCH:
				DBG_MAC << "SCH --> CCH" << std::endl;
				channelBusySelf(false);
				setActiveChannel(type_CCH);
				channelIdle(true);
				phy11p->changeListeningFrequency(frequency[Channels::CCH]);
				break;
		}
		//schedule next channel switch in 50ms

	}
	else if (msg ==  nextMacEvent) {

		//we actually came to the point where we can send a packet
		channelBusySelf(true);
		WaveShortMessage* pktToSend = myEDCA[activeChannel]->initiateTransmit(lastIdle);

		lastAC = mapPriority(pktToSend->getPriority());

		DBG_MAC << "MacEvent received. Trying to send packet with priority" << lastAC << std::endl;

		//send the packet
		Mac80211Pkt* mac = new Mac80211Pkt(pktToSend->getName(), pktToSend->getKind());
		mac->setDestAddr(LAddress::L2BROADCAST());
		mac->setSrcAddr(myMacAddress);
		mac->encapsulate(pktToSend->dup());

		enum PHY_MCS mcs;
		double txPower_mW;
		uint64_t datarate;
		PhyControlMessage *controlInfo = dynamic_cast<PhyControlMessage *>(pktToSend->getControlInfo());
		if (controlInfo) {
			//if MCS is not specified, just use the default one
			mcs = (enum PHY_MCS)controlInfo->getMcs();
			if (mcs != MCS_DEFAULT) {
				datarate = getOfdmDatarate(mcs, BW_OFDM_10_MHZ);
			}
			else {
				datarate = bitrate;
			}
			//apply the same principle to tx power
			txPower_mW = controlInfo->getTxPower_mW();
			if (txPower_mW < 0) {
				txPower_mW = txPower;
			}
		}
		else {
			mcs = MCS_DEFAULT;
			txPower_mW = txPower;
			datarate = bitrate;
		}

		simtime_t sendingDuration = RADIODELAY_11P + getFrameDuration(mac->getBitLength(), mcs);
		DBG_MAC << "Sending duration will be" << sendingDuration << std::endl;
		if ((!useSCH) || (timeLeftInSlot() > sendingDuration)) {
			if (useSCH) DBG_MAC << " Time in this slot left: " << timeLeftInSlot() << std::endl;
			// give time for the radio to be in Tx state before transmitting
			phy->setRadioState(Radio::TX);

			double freq = (activeChannel == type_CCH) ? frequency[Channels::CCH] : frequency[mySCH];

			attachSignal(mac, simTime()+RADIODELAY_11P, freq, datarate, txPower_mW);
			MacToPhyControlInfo* phyInfo = dynamic_cast<MacToPhyControlInfo*>(mac->getControlInfo());
			assert(phyInfo);
			DBG_MAC << "Sending a Packet. Frequency " << freq << " Priority" << lastAC << std::endl;
			sendDelayed(mac, RADIODELAY_11P, lowerLayerOut);
			statsSentPackets++;
		}
		else {   //not enough time left now
			DBG_MAC << "Too little Time left. This packet cannot be send in this slot." << std::endl;
			statsNumTooLittleTime++;
			//revoke TXOP
			myEDCA[activeChannel]->revokeTxOPs();
			delete mac;
			channelIdle();
			//do nothing. contention will automatically start after channel switch
		}
	}
}

void qMAC::handleUpperControl(cMessage* msg) {
	assert(false);
}

void qMAC::handleUpperMsg(cMessage* msg) {

	WaveShortMessage* thisMsg;
	if ((thisMsg = dynamic_cast<WaveShortMessage*>(msg)) == NULL) {
		error("WaveMac only accepts WaveShortMessages");
	}

	lifetime = simTime().dbl() - startTime;

  if (par("learn").longValue()==1){

        //epsilon decay function; training
        //linear decay
        /*
        if ((lifetime<trainTime) && (epsilon>0.1)){
	            epsilon = 1 - (lifetime/trainTime); //start reducing e for 6 minutes after
	            alpha = 1 - (lifetime/trainTime);
	    }
	    else{
	            epsilon=0.1;
	            alpha = 0.1;
	    }
*/
          //if(lifetime<trainTime){
        if ((lifetime<trainTime) && (epsilon>0.05)){ //EPSILON>0.05
                        epsilon = exp(-3*lifetime/trainTime);
                        alpha = exp(-3*lifetime/trainTime);
                        //epsilon = 1 - lifetime/trainTime;
                        //alpha = 1 - lifetime/trainTime;

                    }//AT 410sec
/*
                    else if(lifetime>(trainTime+50) && (epsilon>=0.1) && lifetime<(trainTime+100+50)){
                        epsilon = 1 - (lifetime-trainTime-100)/trainTime;
                        alpha = 1 - (lifetime-trainTime-100)/trainTime;
                        //epsilon = exp(-3*(lifetime-trainTime-100)/trainTime);
                        //alpha = exp(-3*(lifetime-trainTime-100)/trainTime);
                                    }
*/
                    else{

                            epsilon=0.05;
                            alpha = 0.05;

                            //epsilon=0.00;
                            //alpha = 0.00;
                    }

	    //std::cout << epsilon << "\n";
	if(thisMsg->getAcknowledged()<2){
    if (thisMsg->getAcknowledged()==1){ //ACK RECEIVED

				if(currentAction==1){ //no action -steady cw
					reward=0;
				}
				else{ //currentAction increase 1, decrease -1

				reward=reward_v[currentState];   //DELAY OPTIMIZATION
					//reward=1;//give positive reward (binary)
				}

				if(simTime()>(startTime+trainTime)) ackReceived++;

				//chooseAction (epsilon-greedy)
                if(uniform(0,1)<epsilon){
                    //DO NOT VISIT Q(1,1) and Q(7,3);
                    if(currentState==0){
                        currentAction=action[(rand() % 2)+1];
                    }
                    else if (currentState==6){
                        currentAction=action[rand() % 2];
                    }
                    else
                        currentAction=action[rand() % 3];
                }
                else{  //exploit
                    currentAction=action[maximum(currentState,true)];
                }

                findHost()->getDisplayString().updateWith("r=15,green");

				//cancelEvent(nextMacEvent);
                //drop message
				delete thisMsg;

            }
     else if (thisMsg->getAcknowledged()==0){ //NO ACK RECEIVED

                if(simTime()>(startTime+trainTime)) ackNotReceived++;

                reward=-1;

				//chooseAction (epsilon-greedy)

				if(uniform(0,1)<epsilon){
				    //DO NOT VISIT Q(1,1) and Q(7,3);
				    if(currentState==0){
				        currentAction=action[(rand() % 2)+1];
				    }
				    else if (currentState==6){
				        currentAction=action[rand() % 2];
				    }
				    else
				        currentAction=action[rand() % 3];

				}
				else{
				      currentAction=action[maximum(currentState,true)];
				}


            findHost()->getDisplayString().updateWith("r=15,red");
            delete thisMsg;

       }
    preCumulativeReward=preCumulativeReward+Q[currentState][currentAction];
    //record postCumulativeReward over time
    /*
      std::ofstream myfile4;
      myfile4.open (directory  + "PrecumReward_" + myId+".csv",std::ios_base::app);
                        myfile4<<preCumulativeReward<<","<<simTime()<<std::endl;
                        myfile4.close();
                        */
                   std::ofstream myfile2;
            myfile2.open (directory  +"PrecumReward.csv" ,std::ios_base::app);
                                            myfile2<<preCumulativeReward<<","<<simTime()<<std::endl;
                                            myfile2.close();
        //observe the new s' after action. update Q table
        //s=s'
        //update Contention Window when message comes from upper layer

    //std::cout << "reward for this iteration: " << reward << std::endl;

        if (currentState<7){
            if(currentState<6 && currentAction==2){
                Q[currentState][currentAction]=alpha * (reward + (gamma1 * Q[currentState+1][maximum(currentState+1,true)]))+(1-alpha) * Q[currentState][currentAction];
                //go to the next CW state
                currentState=currentState+1;
                ContentionWindow=cw[currentState];
                //currentAction=1;//do once per notification
            }

            else if(currentState==6 && currentAction==2){
                 //Q[6][2]=-10;
                 Q[currentState][currentAction]=alpha * (reward + (gamma1 * Q[0][maximum(0,true)]))+(1-alpha) * Q[currentState][currentAction];
                 //currentAction=1;
                 //go to the FIRST !! CW state (3) if at 255b
                 /*
                 currentState=0;
                 ContentionWindow=cw[currentState];
                 currentAction=0;
                 */
            }
            else if(currentState==0 && currentAction==0){
                Q[currentState][currentAction]=alpha * (reward + (gamma1 * Q[currentState][maximum(currentState,true)]))+(1-alpha) * Q[currentState][currentAction];
                //currentAction=1;
                //stay on the same stage
            }
            else if(currentState>0 && currentAction==0){
                Q[currentState][currentAction]=alpha * (reward + (gamma1 * Q[currentState-1][maximum(currentState-1,true)]))+(1-alpha)*Q[currentState][currentAction];
                //go to the previous CW state
                currentState=currentState-1;
                ContentionWindow=cw[currentState];
                //currentAction=1;//do once
            }else if(currentAction==1){
                Q[currentState][currentAction]=alpha * (reward + (gamma1 * Q[currentState][maximum(currentState,true)]))+(1-alpha) * Q[currentState][currentAction];
            }
            postCumulativeReward=postCumulativeReward+Q[currentState][currentAction];
            //record postCumulativeReward over time
            /*
                    std::ofstream myfile3;
                    myfile3.open (directory  + "cumR_" +myId+".csv",std::ios_base::app);
                    myfile3<<postCumulativeReward<<","<<simTime()<<std::endl;
                    myfile3.close();
                    */
               std::ofstream myfile2;
        myfile2.open (directory  +"postCumulativeReward.csv" ,std::ios_base::app);
                                        myfile2<<postCumulativeReward<<","<<simTime()<<std::endl;
                                        myfile2.close();
            //if currentAction==keep do not alter the contention window bro
             std::ofstream myfile1;
                   myfile1.open(directory + myId +"_Qtable.csv",std::ios_base::app);

                   for(int count1=0; count1<7; count1++){
                      for(int count = 0; count < 3; count ++){
                           myfile1 << Q[count1][count] << " " ;
                      }
                      myfile1 << "\n";
                  }
                   myfile1 << "\n";
                   myfile1 << "\n";
                   myfile1.close();
        }

        if ((lifetime>trainTime) && (lifetime<trainTime+0.1)){
                        std::ofstream myfile1;
                           myfile1.open(directory + myId +"_Q_converge_table.csv",std::ios_base::app);

                           for(int count1=0; count1<7; count1++){
                              for(int count = 0; count < 3; count ++){
                                   myfile1 << Q[count1][count] << " " ;
                              }
                              myfile1 << "\n";
                          }
                           myfile1 << "\n";
                           myfile1 << "\n";
                          myfile1.close();
                    }
        return;
	}
  }
	else if (par("learn").longValue()==0){
		if(thisMsg->getAcknowledged()<2){
			delete thisMsg;
			return;
		}
	}
	myRateType = thisMsg->getRateType();



	   //actual 802.11p code from now on
	t_access_category ac = mapPriority(thisMsg->getPriority());

	DBG_MAC << "Received a message from upper layer for channel "
	                            << thisMsg->getChannelNumber() << " Access Category (Priority):  "
	                            << ac << std::endl;

	t_channel chan;

	//rewrite SCH channel to actual SCH the Mac1609_4 is set to
	if (thisMsg->getChannelNumber() == Channels::SCH1) {
		ASSERT(useSCH);
		thisMsg->setChannelNumber(mySCH);
		chan = type_SCH;
	}

	//put this packet in its queue
	if (thisMsg->getChannelNumber() == Channels::CCH) {
		chan = type_CCH;
	}

	//update CW
	if(par("learn").longValue()==1){
	    myEDCA[chan]->myQueues[ac].cwMin=ContentionWindow;
	}

	int num = myEDCA[chan]->queuePacket(ac,thisMsg);

	//packet was dropped in Mac
	if (num == -1) {
		statsDroppedPackets++;
		return;
	}

	//if this packet is not at the front of a new queue we dont have to reevaluate times
	DBG_MAC << "sorted packet into queue of EDCA " << chan << " this packet is now at position: " << num << std::endl;

	if (chan == activeChannel) {
		DBG_MAC << "this packet is for the currently active channel" << std::endl;
	}
	else {
		DBG_MAC << "this packet is NOT for the currently active channel" << std::endl;
	}

	if (num == 1 && idleChannel == true && chan == activeChannel) { //channel not busy ->SEND!!

		simtime_t nextEvent = myEDCA[chan]->startContent(lastIdle,guardActive());

		if (nextEvent != -1) {
			if ((!useSCH) || (nextEvent <= nextChannelSwitch->getArrivalTime())) {
				if (nextMacEvent->isScheduled()) {
					cancelEvent(nextMacEvent);
				}
				scheduleAt(nextEvent,nextMacEvent);
				DBG_MAC << "Updated nextMacEvent:" << nextMacEvent->getArrivalTime().raw() << std::endl;
			}
			else {
				DBG_MAC << "Too little time in this interval. Will not schedule nextMacEvent" << std::endl;
				//it is possible that this queue has an txop. we have to revoke it
				myEDCA[activeChannel]->revokeTxOPs();
				statsNumTooLittleTime++;
			}
		}
		else {
			cancelEvent(nextMacEvent);
		}
	}
	if (num == 1 && idleChannel == false && myEDCA[chan]->myQueues[ac].currentBackoff == 0 && chan == activeChannel) {
		myEDCA[chan]->backoff(ac);
	}

	   //RECORD CW variation over time
	    std::ofstream myfile;
	    myfile.open (directory + myId + "_CW.csv",std::ios_base::app);
	    myfile<<myEDCA[chan]->myQueues[ac].cwMin<<","<<myRateType<<","<<epsilon<<","<<simTime()<<std::endl;
	    myfile.close();
	    /*
	   csvWrite(directory + myId + "_CW.csv",myEDCA[chan]->myQueues[ac].cwMin);
	   csvWrite(directory + myId + "_CW.csv",",");
	   csvWrite(directory + myId + "_CW.csv",simTime());
	   	   */

}


void qMAC::handleLowerControl(cMessage* msg) {
	if (msg->getKind() == MacToPhyInterface::TX_OVER) {

		DBG_MAC << "Successfully transmitted a packet on " << lastAC << std::endl;

		phy->setRadioState(Radio::RX);

		//message was sent
		//update EDCA queue. go into post-transmit backoff and set cwCur to cwMin
		myEDCA[activeChannel]->postTransmit(lastAC);
		//channel just turned idle.
		//don't set the chan to idle. the PHY layer decides, not us.

		if (guardActive()) {
			throw cRuntimeError("We shouldnt have sent a packet in guard!");
		}
	}
	else if (msg->getKind() == Mac80211pToPhy11pInterface::CHANNEL_BUSY) {
		channelBusy();
	}
	else if (msg->getKind() == Mac80211pToPhy11pInterface::CHANNEL_IDLE) {
		channelIdle();
	}
	else if (msg->getKind() == Decider80211p::BITERROR || msg->getKind() == Decider80211p::COLLISION) {
		statsSNIRLostPackets++;
		DBG_MAC << "A packet was not received due to biterrors" << std::endl;
	}
	else if (msg->getKind() == Decider80211p::RECWHILESEND) {
		statsTXRXLostPackets++;
		DBG_MAC << "A packet was not received because we were sending while receiving" << std::endl;
	}
	else if (msg->getKind() == MacToPhyInterface::RADIO_SWITCHING_OVER) {
		DBG_MAC << "Phylayer said radio switching is done" << std::endl;
	}
	else if (msg->getKind() == BaseDecider::PACKET_DROPPED) {
		phy->setRadioState(Radio::RX);
		DBG_MAC << "Phylayer said packet was dropped" << std::endl;
	}
	else {
		DBG_MAC << "Invalid control message type (type=NOTHING) : name=" << msg->getName() << " modulesrc=" << msg->getSenderModule()->getFullPath() << "." << std::endl;
		assert(false);
	}

	if (msg->getKind() == Decider80211p::COLLISION) {
		emit(sigCollision, true);
	}

	delete msg;
}

void qMAC::setActiveChannel(t_channel state) {
	activeChannel = state;
	assert(state == type_CCH || (useSCH && state == type_SCH));
}

void qMAC::csvWrite(std::string filename,int A){
	std::ofstream myfile;
	myfile.open (filename,std::ios_base::app);
	myfile<<A<<std::endl;
	myfile.close();
}

int qMAC::maximum(int state, bool returnIndexOnly){
        // if returnIndexOnly = true, a Q matrix index is returned.
        // if returnIndexOnly = false, a Q matrix element is returned.

            int winner;
            bool foundNewWinner;
            bool done = false;

            winner = 0;

            do {
                foundNewWinner = false;
                for(int i = 0; i <= (3 - 1); i++){ //qSize instead of 3
                    if((i < winner) || (i > winner)){     //Avoid self-comparison.
                        if(Q[state][i] > Q[state][winner]){
                            winner = i;
                            foundNewWinner = true;
                        }
                    }
                } // i

                if(foundNewWinner == false){
                    done = true;
                }

            } while(done == false);

            if(returnIndexOnly == true){
                return winner;
            }else{
                return Q[state][winner];
            }
        }

void qMAC::finish() {
	//clean up queues.

	for (std::map<t_channel,EDCA*>::iterator iter = myEDCA.begin(); iter != myEDCA.end(); iter++) {
		statsNumInternalContention += iter->second->statsNumInternalContention;
		statsNumBackoff += iter->second->statsNumBackoff;
		statsSlotsBackoff += iter->second->statsSlotsBackoff;
		iter->second->cleanUp();
		delete iter->second;
	}

	myEDCA.clear();

	if (nextMacEvent->isScheduled()) {
		cancelAndDelete(nextMacEvent);
	}
	else {
		delete nextMacEvent;
	}
	if (nextChannelSwitch && nextChannelSwitch->isScheduled())
		cancelAndDelete(nextChannelSwitch);

	//stats
	recordScalar("ReceivedUnicastPackets",statsReceivedPackets);
	recordScalar("ReceivedBroadcasts",statsReceivedBroadcasts);
	recordScalar("SentPackets",statsSentPackets);
	recordScalar("SNIRLostPackets",statsSNIRLostPackets);
	recordScalar("RXTXLostPackets",statsTXRXLostPackets);
	recordScalar("TotalLostPackets",statsSNIRLostPackets+statsTXRXLostPackets);
	recordScalar("DroppedPacketsInMac",statsDroppedPackets);
	recordScalar("TooLittleTime",statsNumTooLittleTime);
	recordScalar("TimesIntoBackoff",statsNumBackoff);
	recordScalar("SlotsBackoff",statsSlotsBackoff);
	recordScalar("NumInternalContention",statsNumInternalContention);
	recordScalar("totalBusyTime",statsTotalBusyTime.dbl());

	csvWrite(directory+"RXtot.csv",statsReceivedBroadcasts);
	csvWrite(directory+"TXtot.csv",statsSentPackets);
	csvWrite(directory+"CW.csv",CWMin);
	csvWrite(directory+"ACK_rx.csv",ackReceived);
	csvWrite(directory+"ACK_no_rx.csv",ackNotReceived);

	//csvWrite(directory+"CW2.csv",ContentionWindow);
}

void qMAC::attachSignal(Mac80211Pkt* mac, simtime_t startTime, double frequency, uint64_t datarate, double txPower_mW) {

	simtime_t duration = getFrameDuration(mac->getBitLength());

	Signal* s = createSignal(startTime, duration, txPower_mW, datarate, frequency);
	MacToPhyControlInfo* cinfo = new MacToPhyControlInfo(s);

	mac->setControlInfo(cinfo);
}

Signal* qMAC::createSignal(simtime_t start, simtime_t length, double power, uint64_t bitrate, double frequency) {
	simtime_t end = start + length;
	//create signal with start at current simtime and passed length
	Signal* s = new Signal(start, length);

	//create and set tx power mapping
	ConstMapping* txPowerMapping = createSingleFrequencyMapping(start, end, frequency, 5.0e6, power);
	s->setTransmissionPower(txPowerMapping);

	Mapping* bitrateMapping = MappingUtils::createMapping(DimensionSet::timeDomain(), Mapping::STEPS);

	Argument pos(start);
	bitrateMapping->setValue(pos, bitrate);

	pos.setTime(phyHeaderLength / bitrate);
	bitrateMapping->setValue(pos, bitrate);

	s->setBitrate(bitrateMapping);

	return s;
}

/* checks if guard is active */
bool qMAC::guardActive() const {
	if (!useSCH) return false;
	if (simTime().dbl() - nextChannelSwitch->getSendingTime() <= GUARD_INTERVAL_11P)
		return true;
	return false;
}

/* returns the time until the guard is over */
simtime_t qMAC::timeLeftTillGuardOver() const {
	ASSERT(useSCH);
	simtime_t sTime = simTime();
	if (sTime - nextChannelSwitch->getSendingTime() <= GUARD_INTERVAL_11P) {
		return GUARD_INTERVAL_11P
		       - (sTime - nextChannelSwitch->getSendingTime());
	}
	else
		return 0;
}

/* returns the time left in this channel window */
simtime_t qMAC::timeLeftInSlot() const {
	ASSERT(useSCH);
	return nextChannelSwitch->getArrivalTime() - simTime();
}

/* Will change the Service Channel on which the mac layer is listening and sending */
void qMAC::changeServiceChannel(int cN) {
	ASSERT(useSCH);
	if (cN != Channels::SCH1 && cN != Channels::SCH2 && cN != Channels::SCH3 && cN != Channels::SCH4) {
		throw cRuntimeError("This Service Channel doesnt exit: %d",cN);
	}

	mySCH = cN;

	if (activeChannel == type_SCH) {
		//change to new chan immediately if we are in a SCH slot,
		//otherwise it will switch to the new SCH upon next channel switch
		phy11p->changeListeningFrequency(frequency[mySCH]);
	}
}

void qMAC::setTxPower(double txPower_mW) {
	txPower = txPower_mW;
}
void qMAC::setMCS(enum PHY_MCS mcs) {
	ASSERT2(mcs != MCS_DEFAULT, "invalid MCS selected");
	bitrate = getOfdmDatarate(mcs, BW_OFDM_10_MHZ);
	setParametersForBitrate(bitrate);
}

void qMAC::setCCAThreshold(double ccaThreshold_dBm) {
	phy11p->setCCAThreshold(ccaThreshold_dBm);
}

void qMAC::handleLowerMsg(cMessage* msg) {
	Mac80211Pkt* macPkt = static_cast<Mac80211Pkt*>(msg);
	ASSERT(macPkt);

	WaveShortMessage*  wsm =  dynamic_cast<WaveShortMessage*>(macPkt->decapsulate());

	//pass information about received frame to the upper layers
	DeciderResult80211 *macRes = dynamic_cast<DeciderResult80211 *>(PhyToMacControlInfo::getDeciderResult(msg));
	ASSERT(macRes);
	DeciderResult80211 *res = new DeciderResult80211(*macRes);
	wsm->setControlInfo(new PhyToMacControlInfo(res));

	long dest = macPkt->getDestAddr();

	DBG_MAC << "Received frame name= " << macPkt->getName()
	        << ", myState=" << " src=" << macPkt->getSrcAddr()
	        << " dst=" << macPkt->getDestAddr() << " myAddr="
	        << myMacAddress << std::endl;

	if (macPkt->getDestAddr() == myMacAddress) {
		DBG_MAC << "Received a data packet addressed to me." << std::endl;
		statsReceivedPackets++;
		sendUp(wsm);
	}
	else if (dest == LAddress::L2BROADCAST()) {
		statsReceivedBroadcasts++;
		sendUp(wsm);
	}
	else {
		DBG_MAC << "Packet not for me, deleting..." << std::endl;
		delete wsm;
	}

	delete macPkt;
}

int qMAC::EDCA::queuePacket(t_access_category ac,WaveShortMessage* msg) {

	if (maxQueueSize && myQueues[ac].queue.size() >= maxQueueSize) {
		delete msg;
		return -1;
	}
	myQueues[ac].queue.push(msg);
	return myQueues[ac].queue.size();
}

int qMAC::EDCA::createQueue(int aifsn, int cwMin, int cwMax,t_access_category ac) {

	if (myQueues.find(ac) != myQueues.end()) {
		throw cRuntimeError("You can only add one queue per Access Category per EDCA subsystem");
	}

	EDCAQueue newQueue(aifsn,cwMin,cwMax,ac);
	myQueues[ac] = newQueue;

	return ++numQueues;
}

qMAC::t_access_category qMAC::mapPriority(int prio) {
	//dummy mapping function
	switch (prio) {
		case 0: return AC_BK;
		case 1: return AC_BE;
		case 2: return AC_VI;
		case 3: return AC_VO;
		default: throw cRuntimeError("MacLayer received a packet with unknown priority"); break;
	}
	return AC_VO;
}

WaveShortMessage* qMAC::EDCA::initiateTransmit(simtime_t lastIdle) {

	//iterate through the queues to return the packet we want to send
	WaveShortMessage* pktToSend = NULL;

	simtime_t idleTime = simTime() - lastIdle;

	DBG_MAC << "Initiating transmit at " << simTime() << ". I've been idle since " << idleTime << std::endl;

	for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
		if (iter->second.queue.size() != 0) {
			if (idleTime >= iter->second.aifsn* SLOTLENGTH_11P + SIFS_11P && iter->second.txOP == true) {

				DBG_MAC << "Queue " << iter->first << " is ready to send!" << std::endl;

				iter->second.txOP = false;
				//this queue is ready to send
				if (pktToSend == NULL) {
					pktToSend = iter->second.queue.front();
				}
				else {
					//there was already another packet ready. we have to go increase cw and go into backoff. It's called internal contention and its wonderful

					statsNumInternalContention++;
					iter->second.cwCur = std::min(iter->second.cwMax,iter->second.cwCur*2);
					iter->second.currentBackoff = OWNER intuniform(0,iter->second.cwCur);
					DBG_MAC << "Internal contention for queue " << iter->first  << " : "<< iter->second.currentBackoff << ". Increase cwCur to " << iter->second.cwCur << std::endl;
				}
			}
		}
	}

	if (pktToSend == NULL) {
		throw cRuntimeError("No packet was ready");
	}
	return pktToSend;
}

simtime_t qMAC::EDCA::startContent(simtime_t idleSince,bool guardActive) {

	DBG_MAC << "Restarting contention." << std::endl;

	simtime_t nextEvent = -1;

	simtime_t idleTime = SimTime().setRaw(std::max((int64_t)0,(simTime() - idleSince).raw()));;

	lastStart = idleSince;

	DBG_MAC << "Channel is already idle for:" << idleTime << " since " << idleSince << std::endl;

	//this returns the nearest possible event in this EDCA subsystem after a busy channel

	for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
		if (iter->second.queue.size() != 0) {

			/* 1609_4 says that when attempting to send (backoff == 0) when guard is active, a random backoff is invoked */

			if (guardActive == true && iter->second.currentBackoff == 0) {
				//cw is not increased
				iter->second.currentBackoff = OWNER intuniform(0,iter->second.cwCur);
				statsNumBackoff++;
			}

			simtime_t DIFS = iter->second.aifsn * SLOTLENGTH_11P + SIFS_11P;

			//the next possible time to send can be in the past if the channel was idle for a long time, meaning we COULD have sent earlier if we had a packet
			simtime_t possibleNextEvent = DIFS + iter->second.currentBackoff * SLOTLENGTH_11P;


			DBG_MAC << "Waiting Time for Queue " << iter->first <<  ":" << possibleNextEvent << "=" << iter->second.aifsn << " * "  << SLOTLENGTH_11P << " + " << SIFS_11P << "+" << iter->second.currentBackoff << "*" << SLOTLENGTH_11P << "; Idle time: " << idleTime << std::endl;

			if (idleTime > possibleNextEvent) {
				DBG_MAC << "Could have already send if we had it earlier" << std::endl;
				//we could have already sent. round up to next boundary
				simtime_t base = idleSince + DIFS;
				possibleNextEvent =  simTime() - simtime_t().setRaw((simTime() - base).raw() % SLOTLENGTH_11P.raw()) + SLOTLENGTH_11P;
			}
			else {
				//we are gonna send in the future
				DBG_MAC << "Sending in the future" << std::endl;
				possibleNextEvent =  idleSince + possibleNextEvent;
			}
			nextEvent == -1? nextEvent =  possibleNextEvent : nextEvent = std::min(nextEvent,possibleNextEvent);
		}
	}
	return nextEvent;
}

void qMAC::EDCA::stopContent(bool allowBackoff, bool generateTxOp) {
	//update all Queues

	DBG_MAC << "Stopping Contention at " << simTime().raw() << std::endl;

	simtime_t passedTime = simTime() - lastStart;

	DBG_MAC << "Channel was idle for " << passedTime << std::endl;

	lastStart = -1; //indicate that there was no last start

	for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
		if (iter->second.currentBackoff != 0 || iter->second.queue.size() != 0) {
			//check how many slots we already waited until the chan became busy

			int oldBackoff = iter->second.currentBackoff;

			std::string info;
			if (passedTime < iter->second.aifsn * SLOTLENGTH_11P + SIFS_11P) {
				//we didnt even make it one DIFS :(
				info.append(" No DIFS");
			}
			else {
				//decrease the backoff by one because we made it longer than one DIFS
				iter->second.currentBackoff--;

				//check how many slots we waited after the first DIFS
				int passedSlots = (int)((passedTime - SimTime(iter->second.aifsn * SLOTLENGTH_11P + SIFS_11P)) / SLOTLENGTH_11P);

				DBG_MAC << "Passed slots after DIFS: " << passedSlots << std::endl;


				if (iter->second.queue.size() == 0) {
					//this can be below 0 because of post transmit backoff -> backoff on empty queues will not generate macevents,
					//we dont want to generate a txOP for empty queues
					iter->second.currentBackoff -= std::min(iter->second.currentBackoff,passedSlots);
					info.append(" PostCommit Over");
				}
				else {
					iter->second.currentBackoff -= passedSlots;
					if (iter->second.currentBackoff <= -1) {
						if (generateTxOp) {
							iter->second.txOP = true; info.append(" TXOP");
						}
						//else: this packet couldnt be sent because there was too little time. we could have generated a txop, but the channel switched
						iter->second.currentBackoff = 0;
					}

				}
			}
			DBG_MAC << "Updating backoff for Queue " << iter->first << ": " << oldBackoff << " -> " << iter->second.currentBackoff << info <<std::endl;
		}
	}
}
void qMAC::EDCA::backoff(t_access_category ac) {
	myQueues[ac].currentBackoff = OWNER intuniform(0,myQueues[ac].cwCur);
	statsSlotsBackoff += myQueues[ac].currentBackoff;
	statsNumBackoff++;
	DBG_MAC << "Going into Backoff because channel was busy when new packet arrived from upperLayer" << std::endl;
}

void qMAC::EDCA::postTransmit(t_access_category ac) {
	delete myQueues[ac].queue.front();
	myQueues[ac].queue.pop();
	myQueues[ac].cwCur = myQueues[ac].cwMin;
	//myQueues[ac].cwCur = 3; //reset to min CW.
	//post transmit backoff
	myQueues[ac].currentBackoff = OWNER intuniform(0,myQueues[ac].cwCur);
	statsSlotsBackoff += myQueues[ac].currentBackoff;
	statsNumBackoff++;
	DBG_MAC << "Queue " << ac << " will go into post-transmit backoff for " << myQueues[ac].currentBackoff << " slots" << std::endl;
}

void qMAC::EDCA::cleanUp() {
	for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
		while (iter->second.queue.size() != 0) {
			delete iter->second.queue.front();
			iter->second.queue.pop();
		}
	}
	myQueues.clear();
}

void qMAC::EDCA::revokeTxOPs() {
	for (std::map<t_access_category, EDCAQueue>::iterator iter = myQueues.begin(); iter != myQueues.end(); iter++) {
		if (iter->second.txOP == true) {
			iter->second.txOP = false;
			iter->second.currentBackoff = 0;
		}
	}
}

void qMAC::channelBusySelf(bool generateTxOp) {

	//the channel turned busy because we're sending. we don't want our queues to go into backoff
	//internal contention is already handled in initiateTransmission

	if (!idleChannel) return;
	idleChannel = false;
	DBG_MAC << "Channel turned busy: Switch or Self-Send" << std::endl;

	lastBusy = simTime();

	//channel turned busy
	if (nextMacEvent->isScheduled() == true) {
		cancelEvent(nextMacEvent);
	}
	else {
		//the edca subsystem was not doing anything anyway.
	}
	myEDCA[activeChannel]->stopContent(false, generateTxOp);

	emit(sigChannelBusy, true);
}

void qMAC::channelBusy() {

	if (!idleChannel) return;

	//the channel turned busy because someone else is sending
	idleChannel = false;
	DBG_MAC << "Channel turned busy: External sender" << std::endl;
	lastBusy = simTime();

	//channel turned busy
	if (nextMacEvent->isScheduled() == true) {
		cancelEvent(nextMacEvent);
	}
	else {
		//the edca subsystem was not doing anything anyway.
	}
	myEDCA[activeChannel]->stopContent(true,false);

	emit(sigChannelBusy, true);
}

void qMAC::channelIdle(bool afterSwitch) {

	DBG_MAC << "Channel turned idle: Switch: " << afterSwitch << std::endl;

	if (nextMacEvent->isScheduled() == true) {
		//this rare case can happen when another node's time has such a big offset that the node sent a packet although we already changed the channel
		//the workaround is not trivial and requires a lot of changes to the phy and decider
		return;
		//throw cRuntimeError("channel turned idle but contention timer was scheduled!");
	}

	idleChannel = true;

	simtime_t delay = 0;

	//account for 1609.4 guards
	if (afterSwitch) {
		//	delay = GUARD_INTERVAL_11P;
	}
	if (useSCH) {
		delay += timeLeftTillGuardOver();
	}

	//channel turned idle! lets start contention!
	lastIdle = delay + simTime();
	statsTotalBusyTime += simTime() - lastBusy;

	//get next Event from current EDCA subsystem
	simtime_t nextEvent = myEDCA[activeChannel]->startContent(lastIdle,guardActive());
	if (nextEvent != -1) {
		if ((!useSCH) || (nextEvent < nextChannelSwitch->getArrivalTime())) {
			scheduleAt(nextEvent,nextMacEvent);
			DBG_MAC << "next Event is at " << nextMacEvent->getArrivalTime().raw() << std::endl;
		}
		else {
			DBG_MAC << "Too little time in this interval. will not schedule macEvent" << std::endl;
			statsNumTooLittleTime++;
			myEDCA[activeChannel]->revokeTxOPs();
		}
	}
	else {
		DBG_MAC << "I don't have any new events in this EDCA sub system" << std::endl;
	}
	emit(sigChannelBusy, false);

}

void qMAC::setParametersForBitrate(uint64_t bitrate) {
	for (unsigned int i = 0; i < NUM_BITRATES_80211P; i++) {
		if (bitrate == BITRATES_80211P[i]) {
			n_dbps = N_DBPS_80211P[i];
			return;
		}
	}
	throw cRuntimeError("Chosen Bitrate is not valid for 802.11p: Valid rates are: 3Mbps, 4.5Mbps, 6Mbps, 9Mbps, 12Mbps, 18Mbps, 24Mbps and 27Mbps. Please adjust your omnetpp.ini file accordingly.");
}


simtime_t qMAC::getFrameDuration(int payloadLengthBits, enum PHY_MCS mcs) const {
    simtime_t duration;
    if (mcs == MCS_DEFAULT) {
        // calculate frame duration according to Equation (17-29) of the IEEE 802.11-2007 standard
        duration = PHY_HDR_PREAMBLE_DURATION + PHY_HDR_PLCPSIGNAL_DURATION + T_SYM_80211P * ceil( (16 + payloadLengthBits + 6)/(n_dbps) );
    }
    else {
        uint32_t ndbps = getNDBPS(mcs);
        duration = PHY_HDR_PREAMBLE_DURATION + PHY_HDR_PLCPSIGNAL_DURATION + T_SYM_80211P * ceil( (16 + payloadLengthBits + 6)/(ndbps) );
    }

	return duration;
}
