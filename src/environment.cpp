#include <environment.h>


std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* Environment::getAllBinParts(){
    return &all_binParts;
}
std::map<std::string, std::vector<geometry_msgs::Pose>>* Environment::getSortedBinParts(){
    return &sorted_all_binParts;

}
std::vector<std::map<std::string, std::vector<OrderPart*>>>* Environment::getAgv1OrderParts(){
     return &agv1_orderParts;
}
std::vector<std::map<std::string, std::vector<OrderPart*>>>* Environment::getAgv2OrderParts(){
     return &agv2_orderParts;
}
std::map<std::string, std::vector<geometry_msgs::Pose>>* Environment::getTray1Parts(){
    return &tray1_parts;
}

std::map<std::string, std::vector<geometry_msgs::Pose>>* Environment::getTray2Parts(){
     return &tray2_parts;
}