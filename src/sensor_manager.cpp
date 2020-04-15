#include<sensor_manager.h>

SensorManager::SensorManager(Environment * env):
async_spinner(0),
env_(env),
lcamera_agv1("/ariac/logical_camera_1", env_, false, true),
lcamera_bin1("/ariac/logical_camera_2", env_, true, false),
lcamera_bin2("/ariac/logical_camera_3", env_, true, false),
lcamera_common_bin("/ariac/logical_camera_4", env_, true, false),
lcamera_bin4("/ariac/logical_camera_5", env_, true, false),
lcamera_bin5("/ariac/logical_camera_6", env_, true, false),
lcamera_bin6("/ariac/logical_camera_7", env_, true, false),
lcamera_agv2("/ariac/logical_camera_8", env_, false, true),
qcamera_agv1("/ariac/quality_control_sensor_1", env_),
qcamera_agv2("/ariac/quality_control_sensor_2", env_) {
async_spinner.start();
}

SensorManager::~SensorManager(){};
