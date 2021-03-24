//
// ObstacleControl - models obstacles that block radio transmissions
// Copyright (C) 2006 Christoph Sommer <christoph.sommer@informatik.uni-erlangen.de>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//


#ifndef OBSTACLE_OBSTACLE_H
#define OBSTACLE_OBSTACLE_H

#include <vector>
#include "veins/base/utils/Coord.h"
#include "veins/modules/world/annotations/AnnotationManager.h"

/**
 * stores information about an Obstacle for ObstacleControl
 */
namespace Veins {
class Obstacle {
	public:
		typedef std::vector<Coord> Coords;

		Obstacle(std::string id, std::string type, double attenuationPerCut, double attenuationPerMeter);

		void setShape(Coords shape);
		const Coords& getShape() const;
		const Coord getBboxP1() const;
		const Coord getBboxP2() const;

		std::string getType() const;
		std::string getId() const;

		double calculateAttenuation(const Coord& senderPos, const Coord& receiverPos) const;

		AnnotationManager::Annotation* visualRepresentation;

	protected:
		std::string id;
		std::string type;
		double attenuationPerCut; /**< in dB. attenuation per exterior border of obstacle */
		double attenuationPerMeter; /**< in dB / m. to account for attenuation caused by interior of obstacle */
		Coords coords;
		Coord bboxP1;
		Coord bboxP2;
};
}

#endif
