//
// Created by lasagnaphil on 21. 2. 17..
//

#ifndef ARTSIM_XML_H
#define ARTSIM_XML_H

#include <artsim/artsim.h>
#include <pugixml.hpp>

namespace artsim {

bool load_from_xml_legacy(pugi::xml_node root_el, OUT ArticulatedBodySpec& art);
pugi::xml_parse_result load_from_xml_legacy(const char* filename, OUT ArticulatedBodySpec& art);

bool load_from_xml(pugi::xml_node art_elem, const char* current_dir, OUT ArticulatedBodySpec& spec);
pugi::xml_parse_result load_from_xml(const char* filename, OUT ArticulatedBodySpec& spec);

pugi::xml_node save_to_xml(pugi::xml_document& doc, ArticulatedBodySpec& spec);
void save_to_xml(const char* filename, ArticulatedBodySpec& art);

}

#endif //ARTSIM_XML_H

