#include <environment.h>


Environment::Environment() :
all_binCamera_called(false),
all_trayCamera_called(false),
trayCameraRequired(false),
order_manager_status(false),
binCameraRequired(false){

// common trash pose
    trash_bin_pose_.position.x = -0.20;
	trash_bin_pose_.position.y = 0.0;
	trash_bin_pose_.position.z = 0.95;
    trash_bin_pose_.orientation.w  = 0;
    trash_bin_pose_.orientation.x  = 0;
    trash_bin_pose_.orientation.y  = 0;
    trash_bin_pose_.orientation.z  = 0;

};

Environment::~Environment(){};


std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* Environment::getAllBinParts(){
	return &all_binParts;
}

std::map<std::string, std::vector<geometry_msgs::Pose>>* Environment::getSortedBinParts(){
	return &sorted_all_binParts;
}

std::vector<std::map<std::string, std::vector<OrderPart*>>>* Environment::getArm1PreOrderParts(){
	return &arm1_pre_orderParts;
}

std::vector<std::map<std::string, std::vector<OrderPart*>>>* Environment::getArm2PreOrderParts(){
	return &arm2_pre_orderParts;
}

std::vector<std::map<std::string, std::vector<OrderPart*>>>* Environment::getArm1OrderParts(){
	return &arm1_orderParts;
}
std::vector<std::map<std::string, std::vector<OrderPart*>>>* Environment::getArm2OrderParts(){
	return &arm2_orderParts;
}
std::map<std::string, std::vector<geometry_msgs::Pose>>* Environment::getTray1Parts(){
	return &tray_parts["logical_camera_1"];
}

std::map<std::string, std::vector<geometry_msgs::Pose>>* Environment::getTray2Parts(){
     return &tray_parts["logical_camera_8"];
}

std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* Environment::getAllTrayParts(){
	return &tray_parts;
}

geometry_msgs::Pose Environment::getTrashBinPose(){
     return trash_bin_pose_;
}

void Environment::setorderManagerStatus(bool status) {
	order_manager_status = status;
}

bool Environment::getOrderManagerStatus() const {
	return order_manager_status;
}


bool Environment::isAllBinCameraCalled(){
	return all_binCamera_called;
}

bool Environment::isAllTrayCameraCalled(){
	return all_trayCamera_called;
}

bool Environment::isBinCameraRequired(){
	return binCameraRequired;
}

bool Environment::isTrayCameraRequired(){
	return trayCameraRequired;
}

void Environment::setBinCameraRequired(const bool& cond) {
	binCameraRequired = cond;
}

void Environment::setTrayCameraRequired(const bool& cond) {
	binCameraRequired = cond;
}

std::map<std::string, bool>* Environment::getBinCamBoolMap() {
	return &bin_cam_bool_map_;
}

std::map<std::string, bool>* Environment::getTrayCamBoolMap() {
	return &tray_cam_bool_map_;
 }

std::map<std::string, bool>* Environment::getQualityCamBoolMap() {
	return &quality_cam_bool_map_;
 }

void Environment::setAllBinCameraCalled(const bool& cond){
	all_binCamera_called = cond;
}

void Environment::setAllTrayCameraCalled(const bool& cond){
	all_trayCamera_called = cond;
}

void Environment::resetBinCamBoolmap() {
	for(auto it : bin_cam_bool_map_) {
		it.second = false;
	}
}

void Environment::resetTrayCamBoolmap() {
	for(auto it : tray_cam_bool_map_) {
		it.second = false;
	}
}

void Environment::resetQualityCamBoolmap() {
	for(auto it : quality_cam_bool_map_) {
		it.second = false;
	}
}

std::map<std::string, bool>* Environment::getQualityCamerasPartfaulty(){
	return &is_faulty;
}

bool Environment::isQualityCamera1Partfaulty() {
	return is_faulty["quality_control_sensor_1"];
}
bool Environment::isQualityCamera2Partfaulty(){
	return is_faulty["quality_control_sensor_2"];
}

void Environment::setSeeQualityCamera1(bool status) {
	quality_cam_see_map_["quality_control_sensor_1"] = status;
}

void Environment::setSeeQualityCamera2(bool status) {
	quality_cam_see_map_["quality_control_sensor_2"] = status;
}

bool Environment::seeQualityCamera(std::string cam_name) {
	return quality_cam_see_map_[cam_name];
}

bool Environment::isQuality1Called() {
	return all_qualityCamera_called["quality_control_sensor_1"];
}

bool Environment::isQuality2Called() {
	return all_qualityCamera_called["quality_control_sensor_2"];
}