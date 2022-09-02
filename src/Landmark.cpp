#include "emcl/Landmark.h"

struct pos { double x, y, z; };

namespace YAML {

template<>
struct convert<struct pos> {
  static bool decode(const Node& node, struct pos& p) {
    p.x = node[0].as<double>();
    p.y = node[1].as<double>();
    p.z = node[2].as<double>();
    return true;
  }
};

}

namespace emcl {

bool Landmark::readMapFile(const std::string filename)
{
    map_.clear();
    YAML::Node node = YAML::LoadFile(filename);
    YAML::Node landmarks = node["landmark"];
    if (landmarks.IsSequence()) {
        for(const auto landmark: landmarks) {
            struct object obj;
            obj.name = landmark["name"].as<std::string>();
            struct pos v = landmark["pose"].as<struct pos>();
            obj.x = v.x;
            obj.x = v.y;
            map_.push_back(obj);
        }
    }
}

}
