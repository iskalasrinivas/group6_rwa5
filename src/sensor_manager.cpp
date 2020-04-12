#include<sensor_manager.h>

SensorManager::SensorManager(Environment * env):
env_(env),
lcamera_bin1("/ariac/logical_camera_1", env_),
lcamera_bin2("/ariac/logical_camera_2", env_),
lcamera_bin3("/ariac/logical_camera_3", env_),
lcamera_bin4("/ariac/logical_camera_4", env_),
lcamera_agv1("/ariac/logical_camera_5", env_),
lcamera_agv2("/ariac/logical_camera_6", env_){
    
}

SensorManager::~SensorManager(){};