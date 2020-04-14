#ifndef GROUP6_RWA4_ENVIRONMENT_H
#define GROUP6_RWA4_ENVIRONMENT_H

#include <list>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <osrf_gear/Order.h>
#include <order_part.h>


class Environment{

    private:
    std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>> all_binParts;
    std::map<std::string, std::vector<geometry_msgs::Pose>> sorted_all_binParts;
    std::vector<std::map<std::string, std::vector<OrderPart*>>> arm1_pre_orderParts; // to be pre-excuted by arm 1
    std::vector<std::map<std::string, std::vector<OrderPart*>>> arm2_pre_orderParts; // to be pre-excuted by arm 2
    std::vector<std::map<std::string, std::vector<OrderPart*>>> arm1_orderParts; // to be excuted by arm 1
    std::vector<std::map<std::string, std::vector<OrderPart*>>> arm2_orderParts; // to be excuted by arm 2
    std::map<std::string, std::vector<OrderPart*>> agv_trash;
    // OrderPart* immediate_goal;
    std::map<std::string, std::vector<geometry_msgs::Pose>> tray1_parts;
    std::map<std::string, std::vector<geometry_msgs::Pose>> tray2_parts;
    bool all_binCamera_called;
    bool pickup_location_added;

    public:
    Environment();
    ~Environment();

    void setAllBinParts();
    void sortAllBinParts();
    void setAllOrderParts();
    void setReceivedOrders();
    void setPickLocationAdded();

    std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* getAllBinParts();
    std::map<std::string, std::vector<geometry_msgs::Pose>>* getSortedBinParts();
    std::vector<std::map<std::string, std::vector<OrderPart*>>>* getArm1PreOrderParts();
    std::vector<std::map<std::string, std::vector<OrderPart*>>>* getArm2PreOrderParts();
    std::vector<std::map<std::string, std::vector<OrderPart*>>>* getArm1OrderParts();
    std::vector<std::map<std::string, std::vector<OrderPart*>>>* getArm2OrderParts();
    std::map<std::string, std::vector<geometry_msgs::Pose>>* getTray1Parts();
    std::map<std::string, std::vector<geometry_msgs::Pose>>* getTray2Parts();
    bool isAllBinCameraCalled();

    OrderPart* getImmediateGoal();

};

#endif //GROUP6_RWA4_ENVIRONMENT_H