/**
 * @file      src/executer.cpp
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
#include <environment.h>
#include <executer.h>
#include <robot_controller.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

Executer::Executer(Environment* env): env_(env), arm_1_has_been_zeroed_(false), arm_2_has_been_zeroed_(false), arm1_("arm1"), arm2_("arm2"){
	arm_1_joint_trajectory_publisher_ = execute_nh_.advertise<trajectory_msgs::JointTrajectory>( "/ariac/arm1/arm/command", 10);

	arm_2_joint_trajectory_publisher_ = execute_nh_.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm2/arm/command", 10);


}

Executer::~Executer(){

}




// Called when a new JointState message is received.

void Executer::arm_1_joint_state_callback(
		const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{
	ROS_INFO_STREAM_THROTTLE(10,
			"Joint States arm 1 (throttled to 0.1 Hz):\n" << *joint_state_msg);
	// ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
	arm_1_current_joint_states_ = *joint_state_msg;
	if (!arm_1_has_been_zeroed_) {
		arm_1_has_been_zeroed_ = true;
		ROS_INFO("Sending arm to zero joint positions...");
		send_arm_to_zero_state(arm_1_joint_trajectory_publisher_);
	}
}

void Executer::arm_2_joint_state_callback(
		const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{
	ROS_INFO_STREAM_THROTTLE(10,
			"Joint States arm 2 (throttled to 0.1 Hz):\n" << *joint_state_msg);
	// ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
	arm_2_current_joint_states_ = *joint_state_msg;
	if (!arm_2_has_been_zeroed_) {
		arm_2_has_been_zeroed_ = true;
		ROS_INFO("Sending arm 2 to zero joint positions...");
		send_arm_to_zero_state(arm_2_joint_trajectory_publisher_);
	}
}


/// Create a JointTrajectory with all positions set to zero, and command the arm.
void Executer::send_arm_to_zero_state(ros::Publisher & joint_trajectory_publisher) {
	// Create a message to send.
	trajectory_msgs::JointTrajectory msg;

	// Fill the names of the joints to be controlled.
	// Note that the vacuum_gripper_joint is not controllable.
	msg.joint_names.clear();
	msg.joint_names.push_back("shoulder_pan_joint");
	msg.joint_names.push_back("shoulder_lift_joint");
	msg.joint_names.push_back("elbow_joint");
	msg.joint_names.push_back("wrist_1_joint");
	msg.joint_names.push_back("wrist_2_joint");
	msg.joint_names.push_back("wrist_3_joint");
	msg.joint_names.push_back("linear_arm_actuator_joint");
	// Create one point in the trajectory.
	msg.points.resize(1);
	// Resize the vector to the same length as the joint names.
	// Values are initialized to 0.
	msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
	// How long to take getting to the point (floating point seconds).
	msg.points[0].time_from_start = ros::Duration(0.001);
	ROS_INFO_STREAM("Sending command:\n" << msg);
	joint_trajectory_publisher.publish(msg);
}

// void Executer::Executor() {


	
// 	if(!order_part.empty())
// 	{
// 		auto src_pose = order_part->getCurrentPose();
// 		auto goal_pose = order_part->getEndPose();
// 		control_.moveToTarget(src_pose); //ask saurav whether the planner has to use waypoints or not
// 		control_.GripperToggle(True);
// 		if(control_.isPartAttached()){
// 		   control_.moveToTarget(goal_pose);
// 		}
// 		control_.GripperToggle(False);

void  Executer::updatePickAndEndPose(OrderPart * oPart){
      Transformer transform_("/ariac/logical_camera_4");


}


void Executer::Execute() {
	auto arm1preOrderParts = env_->getArm1PreOrderParts;
	for((auto po1_vec_it = arm1preOrderParts.begin();po1_vec_it != arm1preOrderParts.end(); ++po1_vec_it) {
		
		for (auto po1_map_it = po1_vec_it->begin();po1_map_it != po1_vec_it->end(); ++po1_map_it) {

			for(auto po1_it = po1_map_it->second.begin();po1_it != po1_map_it->second.end(); ++po1_it) { // pol_it is basically iterator to std::vector<OrderPart*>
				arm2_.PickPart((*po1_it)->currentPose());
				arm2_.GoToQualityCameraFromBin();
				if(arm2_.isAtQualitySensor()){
					if(arm2_.is_faulty) {
						ROS_WARN_STREAM("Part is faulty");
						arm2_.bin_part_faulty =true;
						arm2_.dropInTrash();
						UpdatePickPose((*po1_it));
						--po1_it;
					} else {
						arm2_.deliverThePartinBin((*po1_it)->EndPose());
						// po1_map_it.erase(po1_it);
						// updatePickAndEndPose((*po1_it));
					}
				}
			}
		}
	}

	auto arm2preOrderParts = env_->getArm2PreOrderParts;
	for((auto po2_vec_it = arm1preOrderParts.begin();po2_vec_it != arm1preOrderParts.end(); ++po2_vec_it) {
	
		for (auto po2_map_it = po1_vec_it->begin();po2_map_it != po1_vec_it->end(); ++po2_map_it) {

			for(auto po2_it = po1_map_it->second.begin();po2_it != po1_map_it->second.end(); ++po2_it) {
				arm1_.PickPart((*po2_it)->currentPose());
				arm1_.GoToQualityCameraFromBin();
				if(arm1_.isAtQualitySensor()){
					if(arm1_.is_faulty) {
						ROS_WARN_STREAM("Part is faulty");
						arm1_.bin_part_faulty =true;
						arm1_.dropInTrash();
						UpdatePickPose((*po2_it));
						--po2_it;
					} else {
						arm1_.deliverThePartinBin((*po1_it));
						// po1_map_it.erase(po1_it);
						// updatePickAndEndPose((*po1_it));
					}
				}
			}
		}
	}

	auto arm1OrderParts = env_->getArm1OrderParts;
	for((auto o1_vec_it = arm1OrderParts.begin();o1_vec_it != arm1OrderParts.end(); ++o1_vec_it) {
		
		for (auto o1_map_it = po1_vec_it->begin();o1_map_it != po1_vec_it->end(); ++o1_map_it) {

			for(auto o1_it = po1_map_it->begin();o1_it != po1_map_it->end(); ++o1_it) {
				arm1_.pickPart((*o1_it)->currentPose());
				arm1_.GoToQualityCameraFromBin();
	            if(arm1_.isAtQualitySensor()) {
					if(is_faulty) {
						ROS_WARN_STREAM("Part is faulty");
						bin_part_faulty =true;
						dropInTrash();
					}
				OrderPart* part = (*it);
				updateFaultyPartPose(part);
				return;
			} else {
				ROS_INFO_STREAM("Part is not faulty");
				ROS_INFO_STREAM("Dropping in AGV");
				dropInAGV(end_pose);	
				
				if()
				removeItemFromOrderPart((*o1_it));
				deleteTheOrderPart((*o1_it));
			}
		}
	
	}

	auto arm2OrderParts = env_->getArm2OrderParts;
	for((auto o2_vec_it = arm1OrderParts.begin();o2_vec_it != arm1OrderParts.end(); ++o2_vec_it) {
	
		for (auto o2_map_it = po1_vec_it->begin();o2_map_it != po1_vec_it->end(); ++o2_map_it) {

			for(auto o2_it = po1_map_it->begin();o2_it != po1_map_it->end(); ++o2_it) {
				arm2_.deliverThePartinTray((*o2_it));
				removeItemFromOrderPart((*o2_it));
				deleteTheOrderPart((*o2_it));
			}
		}
}
}




void Executer::updateFaultyPartPose(AriacOrderPart* part){
	if(faulty_part_ != nullptr){
		for (auto cam_it : all_binParts) {
			if(cam_it.second.count(part->getPartType())) {
				part->setCurrentPose(cam_it.second[part->getPartType()].back());
				return;
			}
		}
	}
	faulty_part_=nullptr;
}