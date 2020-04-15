#ifndef GROUP6_RWA5_SENSORMANAGER_H_
#define GROUP6_RWA5_SENSORMANAGER_H_

#include <logical_camera_sensor.h>
#include <environment.h>
class SensorManager{

    private:
    ros::NodeHandle sensor_nh_;
    Environment * env_;
    LogicalCameraSensor lcamera_bin1;
    LogicalCameraSensor lcamera_bin2;
    LogicalCameraSensor lcamera_common_bin;
    LogicalCameraSensor lcamera_bin4;
    LogicalCameraSensor lcamera_bin5;
    LogicalCameraSensor lcamera_bin6;
    LogicalCameraSensor lcamera_agv1;
    LogicalCameraSensor lcamera_agv2;

    public:
   
    SensorManager(Environment *);
    ~SensorManager();



}; 

#endif // GROUP6_RWA5_SENSORMANAGER_H_