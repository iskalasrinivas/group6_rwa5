#include<sensor_manager.h>

SensorManager::SensorManager():
lcamera_bin1("/ariac/logical_camera_1"),
lcamera_bin2("/ariac/logical_camera_2"),
lcamera_bin3("/ariac/logical_camera_3"),
lcamera_bin4("/ariac/logical_camera_4"),
lcamera_agv1("/ariac/logical_camera_5"),
lcamera_agv2("/ariac/logical_camera_6"){
    
}

SensorManager::~SensorManager(){};