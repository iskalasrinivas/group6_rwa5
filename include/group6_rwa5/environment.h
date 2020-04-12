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
    std::vector<std::map<std::string, std::vector<OrderPart*>>> all_orderParts;
    std::map<std::string, std::vector<OrderPart*>> agv_trash;
    OrderPart* immediate_goal;
    std::map<std::string, std::vector<OrderPart*>> tray1_parts;
    std::map<std::string, std::vector<OrderPart*>> tray2_parts;

    public:

    void setAllBinParts();
    void sortAllBinParts();
    void setAllOrderParts();
    void setReceivedOrders();

    std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* getAllBinParts();
    std::map<std::string, std::vector<geometry_msgs::Pose>>* getSortedBinParts();
    std::map<std::string, std::vector<OrderPart*>>* getAllOrderParts();
    std::vector<OrderPart*>* getDeliveredParts();
    std::map<std::string, std::vector<OrderPart*>>* getTray1Parts();
    std::map<std::string, std::vector<OrderPart*>>* getTray2Parts();
};