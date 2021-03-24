//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __VEINS_BASE_COLLECTOR_H_
#define __VEINS_BASE_COLLECTOR_H_

#include <omnetpp.h>
#include <vector>

using namespace omnetpp;

/**
 * TODO - Generated class
 */
class Collector : public cSimpleModule
{
  public:
    static std::vector<int> vNodes;
  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);

/*
    virtual IntVector& getVNodes() {return vNodes;}
    //virtual const IntVector& getVNodes() const {return const_cast<Collector*>(this)->getVNodes();}
    virtual void setVNodes(const IntVector& vNodes) {this->vNodes = vNodes;}
    */
};

/*
IntVector& Collector::getVNodes()
{
    return this->vNodes;
}

void Collector::setVNodes(const IntVector& vNodes)
{
    this->vNodes = vNodes;
}
*/
#endif
