//
// Created by lasagnaphil on 21. 2. 17..
//

#ifndef ARTSIM_XML_H
#define ARTSIM_XML_H

#include <artsim/artsim.h>
#include <tinyxml2.h>

namespace artsim {

bool load_from_xml_legacy(tinyxml2::XMLElement* root_el, OUT ArticulatedBodySpec& art);
tinyxml2::XMLError load_from_xml_legacy(const char* filename, OUT ArticulatedBodySpec& art);

bool load_from_xml(tinyxml2::XMLElement* art_elem, const char* current_dir, OUT ArticulatedBodySpec& spec);
tinyxml2::XMLError load_from_xml(const char* filename, OUT ArticulatedBodySpec& spec);

tinyxml2::XMLElement* save_to_xml(tinyxml2::XMLDocument& doc, ArticulatedBodySpec& spec);
tinyxml2::XMLError save_to_xml(const char* filename, ArticulatedBodySpec& art);

}

#endif //ARTSIM_XML_H

