#ifndef LANdMARK_H__
#define LANdMARK_H__

#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace emcl {

class Landmark
{
public:
    struct object {
        std::string name;
        double x;
        double y;
    };
    std::vector<object> map_;

    struct detect_object {
        std::string name;
        double yaw;
        double distance;
    };
    std::vector<detect_object> detect_objects_;

    bool readMapFile(const std::string filename);
};

}

#endif
