//
// AnnotationDummy - workaround to visualize annotations
// Copyright (C) 2010 Christoph Sommer <christoph.sommer@informatik.uni-erlangen.de>
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

#ifndef WORLD_ANNOTATION_ANNOTATIONDUMMY_H
#define WORLD_ANNOTATION_ANNOTATIONDUMMY_H

#include <omnetpp.h>

/**
 * AnnotationDummy is just a workaround to visualize annotations
 *
 
 */
namespace Veins {

class AnnotationDummy : public omnetpp::cSimpleModule
{
	public:
		~AnnotationDummy();

	protected:
};
}

#endif
