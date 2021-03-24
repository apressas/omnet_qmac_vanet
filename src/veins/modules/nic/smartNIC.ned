
package org.car2x.veins.modules.nic;

import org.car2x.veins.modules.mac.qMAC.qMAC;
import org.car2x.veins.modules.phy.PhyLayer80211p;

module qNIC like INic80211p
{
    gates:
        input upperLayerIn; // to upper layers
        output upperLayerOut; // from upper layers
        output upperControlOut; // control information
        input upperControlIn; // control information
        input radioIn; // radioIn gate for sendDirect

    submodules:
        phy80211p: PhyLayer80211p {
            @display("p=69,218;i=block/process_s");
        }

        smac: qMAC {
            @display("p=70,113");
        }
    connections:
        radioIn --> phy80211p.radioIn;

        smac.lowerControlOut --> phy80211p.upperControlIn;
        smac.lowerLayerOut --> phy80211p.upperLayerIn;
        phy80211p.upperLayerOut --> smac.lowerLayerIn;
        phy80211p.upperControlOut --> smac.lowerControlIn;
        
        smac.upperControlIn <-- upperControlIn;
        smac.upperLayerIn <-- upperLayerIn;

        smac.upperLayerOut --> upperLayerOut;
        smac.upperControlOut --> upperControlOut;
}
