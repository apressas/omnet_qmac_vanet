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

#ifndef EvalApp_H
#define EvalApp_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"

#include "veins/collector/collector.h"  //counts the up-to-date nodes!!!
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <map>

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::AnnotationManager;
using Veins::TraCIScenarioManager;


/**
 * Small IVC Demo using 11p
 */
class EvalApp : public BaseWaveApplLayer {
	public:
		virtual void initialize(int stage);
		virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);
		virtual void finish();
	protected:
		TraCIMobility* mobility;
		TraCICommandInterface* traci;
		TraCICommandInterface::Vehicle* traciVehicle;
		AnnotationManager* annotations;
		simtime_t lastDroveAt;
		bool sentMessage;
		bool isParking;
		bool sendWhileParking;
		static const simsignalwrap_t parkingStateChangedSignal;

		long rebroadSent;
		long rebroadRec;
		long originalRec;
		int originalMultiRec;
		long RxTrained;
		int RxMultiTrained;
		long ACK;

      long reduntantACK;
      int throughputA;
      int throughputB;
      int throughput=0;
      int ackowledgements;
      int unique;
      double rec_time;
      double rec_time1;
      double rec_timeVehicles;
      TraCIScenarioManager* manager;
      double PDR=0;
      int cars=50;
      double latency=0.0;

      double lifetime;
      double rec_timeTP;
      double delay=0.0;
      double delayunq=0.0;


      int unique_received = 0;
      //std::string ackowledged_msgId;
      std::vector<std::string> vACK;
      std::vector<std::string> vUnique;
      std::vector<int> Vvehicle;

      //Where will the results be saved
      std::string directory = "/Volumes/LocalDataHD/a/ap/ap431/RESULTS/";

      protected:
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);
		void sendMessage(std::string blockedRoadId);
		virtual void handlePositionUpdate(cObject* obj);
		virtual void handleParkingUpdate(cObject* obj);
		virtual void sendWSM(WaveShortMessage* wsm);
		virtual void csvWrite(std::string filename,int A);
};

#endif
