//
// Copyright (C) 2011 Stefan Joerer <stefan.joerer@uibk.ac.at>
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

#ifndef ANALOGUEMODEL_TWORAYINTERFERENCEMODEL_H
#define ANALOGUEMODEL_TWORAYINTERFERENCEMODEL_H

#include "veins/base/phyLayer/AnalogueModel.h"
#include "veins/base/modules/BaseWorldUtility.h"
#include "veins/base/phyLayer/MappingBase.h"

using Veins::AirFrame;

/**
 * @brief
 * Extended version of Two-Ray Ground path loss model.
 *
 * See the Veins website <a href="http://veins.car2x.org/"> for a tutorial, documentation, and publications </a>.
 *
 * An in-depth description of the model is available at:
 * Christoph Sommer and Falko Dressler, "Using the Right Two-Ray Model? A Measurement based Evaluation of PHY Models in VANETs," Proceedings of 17th ACM International Conference on Mobile Computing and Networking (MobiCom 2011), Poster Session, Las Vegas, NV, September 2011.
 *
 * @author Stefan Joerer
 *
 * @ingroup analogueModels
 */
class TwoRayInterferenceModel: public AnalogueModel {

	public:
		TwoRayInterferenceModel(double dielectricConstant, bool debug) :
			epsilon_r(dielectricConstant),
			debug(debug) {}

		virtual ~TwoRayInterferenceModel() {}

	virtual void filterSignal(AirFrame *frame, const Coord& sendersPos, const Coord& receiverPos);


	protected:

		class Mapping: public SimpleConstMapping {
			protected:
				double gamma;
				double d;
				double d_dir;
				double d_ref;
				double lambda;
				bool debug;
			public:
				Mapping(double gamma, double distance, double directDistance, double reflDistance, bool debug)
					: SimpleConstMapping(DimensionSet::timeFreqDomain()),
					gamma(gamma),
					d(distance),
					d_dir(directDistance),
					d_ref(reflDistance),
					debug(debug) {}

				virtual double getValue(const Argument& pos) const;

				ConstMapping* constClone() const {
					return new Mapping(*this);
				}
		};

		/** @brief stores the dielectric constant used for calculation */
		double epsilon_r;

		/** @brief Whether debug messages should be displayed. */
		bool debug;
};

#endif /* ANALOGUEMODEL_TWORAYINTERFERENCEMODEL_H */
