#include <environment.h>


std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* Environment::getAllBinParts(){
    return &all_binParts;
}
std::map<std::string, std::vector<geometry_msgs::Pose>>* Environment::getSortedBinParts(){
    return &sorted_all_binParts;

}
std::map<std::string, std::vector<OrderPart*>>* Environment::getAllOrderParts(){
     return &all_orderParts;
}

std::vector<OrderPart*>* Environment::getDeliveredParts(){
    return &delivered_parts;
}

std::map<std::string, std::vector<OrderPart*>>* Environment::getTray1Parts(){
    return &tray1_parts;
}

std::map<std::string, std::vector<OrderPart*>>* Environment::getTray2Parts(){
     return &tray2_parts;
}