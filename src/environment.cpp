#include <environment.h>


Environment::Environment() : all_binCamera_called(false), pickup_location_added(false){

};

Environment::~Environment(){};


std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* Environment::getAllBinParts(){
    return &all_binParts;
}
std::map<std::string, std::vector<geometry_msgs::Pose>>* Environment::getSortedBinParts(){
    return &sorted_all_binParts;

}
std::vector<std::map<std::string, std::vector<OrderPart*>>>* getArm1PreOrderParts(){
    return &arm1_pre_orderParts;
}

std::vector<std::map<std::string, std::vector<OrderPart*>>>* getArm2PreOrderParts(){
    return &arm2_pre_orderParts;
}

std::vector<std::map<std::string, std::vector<OrderPart*>>>* Environment::getArm1OrderParts(){
     return &agv1_orderParts;
}
std::vector<std::map<std::string, std::vector<OrderPart*>>>* Environment::getArm2OrderParts(){
     return &agv2_orderParts;
}
std::map<std::string, std::vector<geometry_msgs::Pose>>* Environment::getTray1Parts(){
    return &tray1_parts;
}

std::map<std::string, std::vector<geometry_msgs::Pose>>* Environment::getTray2Parts(){
     return &tray2_parts;
}

OrderPart* Environment::getImmediateGoal(){
    return &immediate_goal;
}

bool Environment::isAllBinCameraCalled(){
    return all_binCamera_called;
}

void Environment::setPickLocationAdded(){
    pickup_location_added = true;
}