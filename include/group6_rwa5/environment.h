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

    std::vector<std::map<std::string, std::vector<OrderPart*> > > arm1_pre_orderParts; // to be pre-excuted by arm 1
    std::vector<std::map<std::string, std::vector<OrderPart*> > > arm2_pre_orderParts; // to be pre-excuted by arm 2
    std::vector<std::map<std::string, std::vector<OrderPart*> > > arm1_orderParts; // to be excuted by arm 1
    std::vector<std::map<std::string, std::vector<OrderPart*> > > arm2_orderParts; // to be excuted by arm 2
    std::map<std::string, std::vector<OrderPart*>> agv_trash;
    std::map<std::string, bool>is_faulty;
    // OrderPart* immediate_goal;
    std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>> tray_parts;
    geometry_msgs::Pose trash_bin_pose_;

    //to check whether the callback function is called or not
    std::map<std::string, bool> bin_cam_bool_map_;
	std::map<std::string, bool> tray_cam_bool_map_;
    std::map<std::string, bool> quality_cam_bool_map_;
    std::map<std::string, bool> quality_cam_see_map_;
    bool all_binCamera_called;
    bool all_trayCamera_called;
    bool all_qualityCamera_called;
    bool binCameraRequired;
    bool trayCameraRequired;
    bool order_manager_status;

    public:
    Environment();
    ~Environment();

    void setAllBinParts();
    void sortAllBinParts();
    void setAllOrderParts();
    void setReceivedOrders();
    void setorderManagerStatus(bool);

    std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose> > >* getAllBinParts();
    std::map<std::string, std::vector<geometry_msgs::Pose> >* getSortedBinParts();
    std::vector<std::map<std::string, std::vector<OrderPart* > > >* getArm1PreOrderParts();
    std::vector<std::map<std::string, std::vector<OrderPart* > > >* getArm2PreOrderParts();
    std::vector<std::map<std::string, std::vector<OrderPart* > > >* getArm1OrderParts();
    std::vector<std::map<std::string, std::vector<OrderPart* > > >* getArm2OrderParts();
    std::map<std::string, std::vector<geometry_msgs::Pose> >* getTray1Parts();
    std::map<std::string, std::vector<geometry_msgs::Pose> >* getTray2Parts();
    std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose> > >* getAllTrayParts();

    geometry_msgs::Pose getTrashBinPose();
    
    bool getOrderManagerStatus() const;

    std::map<std::string, bool>* getBinCamBoolMap();
    std::map<std::string, bool>* getTrayCamBoolMap();
    std::map<std::string, bool>* getQualityCamBoolMap();
    std::map<std::string, bool>* getQualityCamerasPartfaulty();

    bool isQualityCamera1Partfaulty();
    bool isQualityCamera2Partfaulty();    

    // void sorted_all_binParts(const bool&);
    void setAllBinCameraCalled(const bool& );
    void setAllTrayCameraCalled(const bool&);

    void setTrayCameraRequired(const bool& );
    void setBinCameraRequired(const bool&);

    bool isAllBinCameraCalled();
    bool isAllTrayCameraCalled();

    bool isBinCameraRequired();
    bool isTrayCameraRequired();

    void resetBinCamBoolmap();
    void resetTrayCamBoolmap();
    void resetQualityCamBoolmap();

    void setSeeQualityCamera1(bool);
    void setSeeQualityCamera2(bool);
    bool seeQualityCamera(std::string);

    bool isQuality1Called();
    bool isQuality2Called();

};

#endif //GROUP6_RWA4_ENVIRONMENT_H
