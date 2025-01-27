#ifndef PER_MODEL_H
#define PER_MODEL_H

#include <cassert>

#include "global/MiXiMDefs.h"
#include "MIXIM_veins/nic/phy/AnalogueModel.h"

/**
 * @brief This class applies a parameterized packet error rate
 * to incoming packets. This allows the user to easily
 * study the robustness of its system to packet loss.
 *
 * @ingroup analogueModels
 *
 
 */
class MIXIM_API PERModel : public AnalogueModel {
protected:
	double packetErrorRate;
public:
	/** @brief The PERModel constructor takes as argument the packet error rate to apply (must be between 0 and 1). */
	PERModel(double per): packetErrorRate(per) { assert(per <= 1 && per >= 0);}

	virtual void filterSignal(AirFrame *, const Coord&, const Coord&);

};

#endif
