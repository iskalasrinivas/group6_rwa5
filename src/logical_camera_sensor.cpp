#include <logical_camera_sensor.h>

LogicalCameraSensor::LogicalCameraSensor(std::string topic, Environment * env, bool bc, bool tc):
async_spinner(0), environment_(env), bincam_(bc) ,traycam_(tc), transform_(topic){
	async_spinner.start();
	getCameraName(topic);
	if (bincam_) {
		auto bincamboolmap_ = environment_->getBinCamBoolMap();
		(*bincamboolmap_)[cam_name]= false;
	}
	if (traycam_) {
		auto traycamboolmap_ = environment_->getTrayCamBoolMap();
		(*traycamboolmap_)[cam_name]= false;
	}
	logical_subscriber_ = logical_nh_.subscribe(topic, 10 ,
			&LogicalCameraSensor::logicalCameraCallback, this);
}




LogicalCameraSensor::~LogicalCameraSensor(){}

std::string LogicalCameraSensor::getCameraName(std::string topic_) {
	std::stringstream ss(topic_);
	const char delim = '/';
	std::string p_;
	while (std::getline(ss, p_, delim)) {
	}
	cam_name = p_;

	return cam_name;
}

void LogicalCameraSensor::SortAllBinParts() {

	auto sorted_all_binParts = environment_->getSortedBinParts();
	auto all_binParts = environment_->getAllBinParts();
	sorted_all_binParts->clear();
	for(auto cam_id : *all_binParts) {

		for(auto map_parts : cam_id.second) {
			auto part_type = map_parts.first;
			auto vec_parts = map_parts.second;
			if(sorted_all_binParts->count(part_type)) {
				(*sorted_all_binParts)[part_type].insert((*sorted_all_binParts)[part_type].end(), vec_parts.begin(), vec_parts.end() );
			} else {
				(*sorted_all_binParts)[part_type] = vec_parts;
			}
		}
	}
}

void LogicalCameraSensor::logicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
	
	if(environment_->isBinCameraRequired() or environment_->isTrayCameraRequired()) {
		ROS_INFO_STREAM("calling Logical camera");
		auto bincammap_ = environment_->getBinCamBoolMap();
		auto traycammap_ = environment_->getTrayCamBoolMap();

		// auto cam_name = transform_.getCameraName();
		auto sensor_pose = image_msg->pose;
		transform_.setParentPose(sensor_pose);
		
		std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* currentPartsPtr;
		if (bincammap_->count(cam_name)) {
			currentPartsPtr = environment_->getAllBinParts(); //std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>
		} else if (traycammap_->count(cam_name)) {
			currentPartsPtr = environment_->getAllTrayParts();
		}

		if(currentPartsPtr->count(cam_name) == 1) {
			(*currentPartsPtr)[cam_name].clear();
		}

		for (auto it = image_msg->models.begin(); it != image_msg->models.end();++it) {
			transform_.setChildPose(it->pose);
			transform_.setWorldTransform();
			auto partType = it->type;
			geometry_msgs::Pose pose = transform_.getChildWorldPose();
			if( (*currentPartsPtr)[cam_name].count(partType) ){
				(*currentPartsPtr)[cam_name][partType].push_back(pose);
			}
			else {
				(*currentPartsPtr)[cam_name][partType] = std::vector<geometry_msgs::Pose>{(pose)};
			}
		}

		if(bincammap_->count(cam_name)) {
			SortAllBinParts();
		}

		if(bincam_) {
			auto bincambool_ = environment_->getBinCamBoolMap();
			(*bincambool_)[cam_name]= true;
			auto bincamsize_ = bincambool_->size();
			int count =0;
			for (auto it = bincambool_->begin();it != bincambool_->end(); ++it) {
				if((*it).second == true) {
					count +=1;
				}
			}
			if(count == bincamsize_) {
				environment_->setAllBinCameraCalled(true);
				environment_->setBinCameraRequired(false);
				environment_->resetBinCamBoolmap();
			}
		}

		if(traycam_) {
			auto traycambool_ = environment_->getTrayCamBoolMap();
			(*traycambool_)[cam_name]= true;
			auto traycamsize_ = traycambool_->size();
			int count = 0;
			for (auto it = traycambool_->begin();it != traycambool_->end(); ++it) {
				if((*it).second == true) {
					count +=1;
				}
			}
			if(count == traycamsize_) {
				environment_->setAllTrayCameraCalled(true);
				environment_->setTrayCameraRequired(false);
				environment_->resetTrayCamBoolmap();
			}
		}
	}
}
