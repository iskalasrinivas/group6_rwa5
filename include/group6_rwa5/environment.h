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
    std::vector<std::map<std::string, std::vector<OrderPart*>>> agv1_orderParts; // to be excuted by arm 1
    std::vector<std::map<std::string, std::vector<OrderPart*>>> agv2_orderParts; // to be excuted by arm 2
    std::map<std::string, std::vector<OrderPart*>> agv_trash;
    OrderPart* immediate_goal;
    std::map<std::string, std::vector<geometry_msgs::Pose>> tray1_parts;
    std::map<std::string, std::vector<geometry_msgs::Pose>> tray2_parts;

    public:

    void setAllBinParts();
    void sortAllBinParts();
    void setAllOrderParts();
    void setReceivedOrders();

    std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* getAllBinParts();
    std::map<std::string, std::vector<geometry_msgs::Pose>>* getSortedBinParts();
    std::vector<std::map<std::string, std::vector<OrderPart*>>>* getAgv1OrderParts();
    std::vector<std::map<std::string, std::vector<OrderPart*>>>* getAgv2OrderParts();
    std::map<std::string, std::vector<geometry_msgs::Pose>>* getTray1Parts();
    std::map<std::string, std::vector<geometry_msgs::Pose>>* getTray2Parts();

    OrderPart* getImmediateGoal();

};

#endif //GROUP6_RWA4_ENVIRONMENT_H