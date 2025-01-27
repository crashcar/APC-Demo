//
// Copyright (C) 2015 David Eckhoff <david.eckhoff@fau.de>
//                    Christoph Sommer <sommer@ccs-labs.org>
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

#ifndef ANALOGUEMODEL_NAKAGAMIFADING_H
#define ANALOGUEMODEL_NAKAGAMIFADING_H

#include "msg/AirFrame_serial.h"
#include "MIXIM_veins/nic/phy/AnalogueModel.h"
#include "global/BaseWorldUtility.h"
#include "MIXIM_veins/nic/phy/MappingBase.h"

/**
 * @brief
 * A simple model to account for fast fading using the Nakagami Distribution.
 *
 * See the Veins website <a href="http://veins.car2x.org/"> for a tutorial, documentation, and publications </a>.
 *
 * An in-depth description of the model is available at:
 * Todo: add paper
 *
 
 *
 * @ingroup analogueModels
 */
class NakagamiFading: public AnalogueModel {

	public:
		NakagamiFading(bool constM, double m, bool debug) :
		    constM(constM),
		    m(m),
			debug(debug) {}

		virtual ~NakagamiFading() {}

	virtual void filterSignal(AirFrame *frame, const Coord& sendersPos, const Coord& receiverPos);


	protected:

		/** @brief Whether to use a constant m or a m based on distance */
		bool constM;

		/** @brief The value of the coefficient m */
		double m;

		/** @brief Whether debug messages should be displayed. */
		bool debug;
};

#endif /* ANALOGUEMODEL_NAKAGAMIFADING_H */
