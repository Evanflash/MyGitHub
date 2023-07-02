#include "LaserMapping.hpp"

#include <string>
#include "rclcpp/rclcpp.hpp"

namespace ALOAM{

class LaserMappingNode : public rclcpp::Node{
public:
    LaserMappingNode(std::string name)
        : Node(name){}

};

} // namespace ALOAM