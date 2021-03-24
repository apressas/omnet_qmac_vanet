package multihop_highway;

import org.car2x.veins.nodes.Scenario;
import org.car2x.veins.collector.Collector;

network net extends Scenario
{
    @display("bgb=20037.508,2098.0574");
    submodules:
        Collector: Collector {
            @display("p=254.31,1790.7662");
        }
}
