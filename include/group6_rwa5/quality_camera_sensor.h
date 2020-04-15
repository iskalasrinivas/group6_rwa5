/**
 * @file      include/logical_camera_sensor.h
 * @brief     Header file for Sensor
 * @author    Saurav Kumar
 * @author    Raja Srinivas
 * @author    Sanket Acharya
 * @author    Dinesh Kadirimangalam
 * @author    Preyash Parikh
 * @copyright 2020
 *
 **BSD 3-Clause License
 *
 *Copyright (c) 2020
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *  this software without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GROUP6_RWA5_QUALITY_CAMERA_SENSOR_H_
#define GROUP6_RWA5_QUALITY_CAMERA_SENSOR_H_

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <environment.h>

class QualityCameraSensor {

private:

ros::NodeHandle quality_nh_;
ros::AsyncSpinner async_spinner;
ros::Subscriber quality_subscriber_;
Environment * environment_;
std::string cam_name;
bool is_faulty;
public:
	QualityCameraSensor(std::string, Environment *);
	~QualityCameraSensor();
	std::string getCameraName(std::string);
	void qualityControlSensorCallback(const osrf_gear::LogicalCameraImage::ConstPtr &);
	bool isPartFaulty();
};

#endif // GROUP6_RWA5_QUALITY_CAMERA_SENSOR_H_
