//
// Created by lasagnaphil on 21. 2. 17..
//

#ifndef ARTSIM_XML_H
#define ARTSIM_XML_H

#include <artsim/artsim.h>

namespace artsim {

ArticulatedBody load_from_xml(const char* filename, std::vector<uint32_t>& contact_indices);

}

#endif //ARTSIM_XML_H

