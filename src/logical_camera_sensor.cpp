#include <logical_camera_sensor.h>

LogicalCameraSensor::LogicalCameraSensor(std::string topic, Environment * env, bool blc, bool bc, bool tc,  bool trigcam):
async_spinner(0), environment_(env), beltcam_(blc), bincam_(bc) ,traycam_(tc), triggercam_(trigcam), transform_(topic), isBinPartsSorted(false){
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

	if (beltcam_) {
		auto beltcamboolmap_ = environment_->getBeltCamBoolMap();
		(*beltcamboolmap_)[cam_name]= false;
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

void LogicalCameraSensor::setBinPartsSorted(){
	isBinPartsSorted = true;
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
	setBinPartsSorted();
}

void LogicalCameraSensor::logicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
	if( beltcam_ == true) {
		beltLogicalCameraCallback(image_msg);
	} else if(triggercam_ == true) {
		beltTriggerLogicalCameraCallback(image_msg);
	} else {
		staticLogicalCameraCallback(image_msg);
	}
}

//TODO
void LogicalCameraSensor::beltTriggerLogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
	auto Arm1COP = environment_->getArm1ConveyorOrderParts();
	auto Arm2COP = environment_->getArm2ConveyorOrderParts();
	
	if(!image_msg->models.empty()) {
		for (auto it = image_msg->models.begin(); it != image_msg->models.end();++it) {
			if(Arm1COP->count(it->type)) {
				environment_->setConveyor1Trigger(true);
				// if count(arm1) > count(arms2) --> assign arm1
			} else if(Arm2COP->count(it->type)){
				environment_->setConveyor2Trigger(true);
			}
		}
	}
}

//TODO
void LogicalCameraSensor::beltLogicalCameraCallback (const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
	auto sensor_pose = image_msg->pose;
	transform_.setParentPose(sensor_pose);
	auto Arm1COP = environment_->getArm1ConveyorOrderParts();
	auto Arm2COP = environment_->getArm2ConveyorOrderParts();
	auto arm1pickuplocation = environment_->getBeltArm1PickupLocation();
	auto arm2pickuplocation =  environment_->getBeltArm1PickupLocation();

	for (auto it = image_msg->models.begin(); it != image_msg->models.end();++it) {
		if(Arm1COP->count(it->type)) {
			transform_.setChildPose(it->pose);
			transform_.setWorldTransform();
			geometry_msgs::Pose pose = transform_.getChildWorldPose();
			if (arm1pickuplocation  == nullptr) {
				environment_->createBeltArm1PickupLocation(pose);	
			} else if(arm1pickuplocation->position.y > it->pose.position.y) {
				environment_->setBeltArm1PickupLocation(pose);
			}
			// break;
		} else if (Arm2COP->count(it->type)) {
			transform_.setChildPose(it->pose);
			transform_.setWorldTransform();
			geometry_msgs::Pose pose = transform_.getChildWorldPose();
			if (arm2pickuplocation  == nullptr) {
				environment_->createBeltArm2PickupLocation(pose);	
			} else if(arm2pickuplocation->position.y > it->pose.position.y) {
				environment_->setBeltArm2PickupLocation(pose);
			}
			// break;
		}

}

void LogicalCameraSensor::staticLogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {

	if(environment_->isBinCameraRequired() or environment_->isTrayCameraRequired()) {
		ROS_INFO_STREAM("calling Logical camera");
		auto bincammap_ = environment_->getBinCamBoolMap();
		auto traycammap_ = environment_->getTrayCamBoolMap(); //std::map<std::string, bool>*
		// auto beltcammap_ = environment_->getBeltCamBoolMap();

		// auto cam_name = transform_.getCameraName();
		auto sensor_pose = image_msg->pose;
		transform_.setParentPose(sensor_pose);

		std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* currentPartsPtr;
		if (bincammap_->count(cam_name)) {
			currentPartsPtr = environment_->getAllBinParts(); //std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>
		} else if (traycammap_->count(cam_name)) {
			currentPartsPtr = environment_->getAllTrayParts();
		}
		// } else if (beltcammap_-> count(cam_name)) {
		// conveyor_belt_trigger = true;
		// }

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
	if(bincam_) {
		if(image_msg->models.size() < 4) {
			if(image_msg->pose.y >= 0) {
				environment_->addToAvailableBinPosesArm1(cam_name, image_msg->pose);
			} else  if(image_msg->pose.y < 0) {
				environment_->addToAvailableBinPosesArm2(cam_name, image_msg->pose);
			}
		} else {
			if(image_msg->pose >= 0) {
				environment_->clearBinFromArm1(cam_name);
			} else  if(image_msg->pose < 0) {
				environment_->clearBinFromArm2(cam_name);
			}
		}
	}
	
}
