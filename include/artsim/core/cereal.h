//
// Created by lasagnaphil on 21. 10. 1..
//

#ifndef ARTSIM_CORE_CEREAL_H
#define ARTSIM_CORE_CEREAL_H

// Macro for Cereal serialization to make life easier
#define SERIALIZE_FIELDS(...) template <class Archive> void serialize(Archive& ar) { \
    ar(__VA_ARGS__); \
}

#endif //ARTSIM_CORE_CEREAL_H
