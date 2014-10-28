#ifndef ED_SIMULATOR_OBJECT_H_
#define ED_SIMULATOR_OBJECT_H_

#include "types.h"

#include <geolib/datatypes.h>

namespace sim
{

class UpdateRequest
{

public:

    UpdateRequest() {}

    bool empty() const { return true; }

};

}

#endif
