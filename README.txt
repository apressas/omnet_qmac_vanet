Simulation files for Q-Learning MAC +CCE implementation for Veins VANET framework.

VANET deployment with Q-Learning MAC protocol under OMNeT++.
Built on the Veins 4.4 framework, requires SUMO v0.25 to run (tested with OMNeT 4.4, 5, 5.1.1).

Features single-hop and multi-hop V2V communication on a highway formation.
The experiment scenarios and general parameters can be found and run at: 

experiments->single_hop highway
experiments->multi_hop highway

The WaveShortMessage found at src->veins->modules->messages has also been modified to include extra fields necessary for the protocols and measurements.

The Application Layer utilised can be found at src->veins->modules->application->EvalApp. 
Some modifications have also been done at src->veins->modules->application->ieee80211p->BaseWaveApplLayer files.

The studied MAC protocols and their parameters can be found at src->veins->modules->mac. 
* qMAC is a novel Q-Learning MAC protocol implementation (w/ decaying epsilon-greedy). 
* CCE_MAC is the novel Q-Learning MAC with the Collective Contention Estimation reward function.
* BEB is the Pseudo-Binary Exponential Backoff (WiFi-like) MAC layer.





