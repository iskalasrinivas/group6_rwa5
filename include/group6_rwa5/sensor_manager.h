#include <logical_camera_sensor.h>
class SensorManager{

    private:
    ros::NodeHandle sensor_nh_;
    LogicalCameraSensor lcamera_bin1;
    LogicalCameraSensor lcamera_bin2;
    LogicalCameraSensor lcamera_bin3;
    LogicalCameraSensor lcamera_bin4;
    LogicalCameraSensor lcamera_agv1;
    LogicalCameraSensor lcamera_agv2;

    public:
   
    SensorManager();
    ~SensorManager();



};