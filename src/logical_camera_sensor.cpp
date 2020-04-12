#include <logical_camera_sensor.h>




LogicalCameraSensor::LogicalCameraSensor(std::string topic, Environment * env): environment_(env){
     transform_(topic);
     logical_subscriber_ = logical_nh_.subscribe(topic, 10 ,
			&LogicalCameraSensor::logicalCameraCallback, this);
}

LogicalCameraSensor::~LogicalCameraSensor(){}

void AriacSensorManager::SortAllBinParts() {

    auto sorted_all_binParts = environment_->getSortedBinParts();
    auto all_binParts = environment_->getAllBinParts();
	sorted_all_binParts.clear();
	for(auto cam_id : all_binParts) {

		for(auto map_parts : cam_id.second) {
			auto part_type = map_parts.first;
			auto vec_parts = map_parts.second;
			if(sorted_all_binParts.count(part_type)) {
				sorted_all_binParts[part_type].insert(sorted_all_binParts[part_type].end(), vec_parts.begin(), vec_parts.end() );
			} else {
				sorted_all_binParts[part_type] = vec_parts;
			}
		}
	}

}



void LogicalCameraSensor::logicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {

    auto cam_name = transform_.getCameraName();
    auto sensor_pose = image_msg->pose; 

    transform_.setParentPose(sensor_pose);

    auto all_binParts = environment_->getAllBinParts();

    if(all_binParts.count(cam_name) == 1) {
		all_binParts[cam_name].clear();
	}

    for (auto it = image_msg->models.begin(); it != image_msg->models.end();++it) {
        transform_.setChildPose(it->pose);
        transform_.setWorldTransform();
        auto partType = it->type;
        geometry_msgs::Pose pose = transform_.getChildWorldPose();
        all_binParts[cam_name][partType].push_back(pose);
    }    

    SortAllBinParts();    
} //  OK