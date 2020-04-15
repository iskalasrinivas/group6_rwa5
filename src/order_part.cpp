/**
 * @file      src/order_part.cpp
 * @brief     Header file for building map
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

#include <order_part.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

OrderPart::OrderPart(std::string agv_id, std::string part_type, geometry_msgs::Pose t_pose):
 tray_id(agv_id), part_type_(part_type), tray_pose_(t_pose), tfListener(tfBuffer) {
    
	ros::AsyncSpinner async_spinner(4);
	ROS_INFO_STREAM("New order object Created");
	async_spinner.start();
	worldTransformation();
	ros::Duration(1.0).sleep();
}

OrderPart::OrderPart(): tfListener(tfBuffer){}

OrderPart::~OrderPart() {}

void OrderPart::setPartType(std::string part_type) {
	part_type_ = part_type;
}

void OrderPart::setCurrentPose(geometry_msgs::Pose pose) {
	current_pose_ = pose;
}

void OrderPart::setEndPose(geometry_msgs::Pose pose){
	end_pose_ = pose;
}

void OrderPart::setMiddlePose(geometry_msgs::Pose pose){
	middle_pose_ = pose;
}

std::string OrderPart::getPartType() const {
	return part_type_;
}

geometry_msgs::Pose OrderPart::getEndPose() const {
	return end_pose_;
}

geometry_msgs::Pose OrderPart::getTrayPose() const {
	return tray_pose_;
}

geometry_msgs::Pose OrderPart::getCurrentPose() const {
	return current_pose_;
}

geometry_msgs::Pose OrderPart::getMiddlePose() const {
	return middle_pose_;
}

std::string OrderPart::getTrayId(){
	return tray_id;
}

void OrderPart::worldTransformation() {

	ros::Duration(2.0).sleep();
	try {

		if (tray_id == "agv_1" or tray_id == "any"){
			tray_id = "agv_1";
			tS_w_b = tfBuffer.lookupTransform("world", "kit_tray_1",ros::Time(0));
		}

		else if (tray_id == "agv_2" ) {
			tray_id = "agv_2";
			tS_w_b = tfBuffer.lookupTransform("world", "kit_tray_2",ros::Time(0));
		}
	
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("exception");
		ROS_WARN("%s", ex.what());
	}
	
	ros::Duration(0.2).sleep();

	try{
				tf2::doTransform(tray_pose_, end_pose_, tS_w_b);
			}
			catch (tf2::TransformException &ex) {
					ROS_WARN("exception while converting child frame pose to world frame");
			    	ROS_WARN("%s",ex.what());
			        ros::Duration(0.01).sleep();
			}
	ros::Duration(0.01).sleep();
	// ROS_INFO_STREAM("Order  tray frame : " << tray_pose_.position.x << "  " << tray_pose_.position.y << "  " <<tray_pose_.position.z);
	ROS_INFO_STREAM("Order End Pose: " << end_pose_.position.x << "  " << end_pose_.position.y << "  " <<end_pose_.position.z);

}



