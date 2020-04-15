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

Executer::Executer(Environment* env): async_spinner(0), env_(env), arm_1_has_been_zeroed_(false), arm_2_has_been_zeroed_(false), arm1_("arm1"), arm2_("arm2"){

	// ros::AsyncSpinner async_spinner(0);
	async_spinner.start();

	execute_sub_ = execute_nh_.subscribe<std_msgs::Bool>("/ariac/execute_executer", 10 ,&Executer::executeCallBack, this);

	arm_1_joint_trajectory_publisher_ = execute_nh_.advertise<trajectory_msgs::JointTrajectory>( "/ariac/arm1/arm/command", 10);

	arm_2_joint_trajectory_publisher_ = execute_nh_.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm2/arm/command", 10);
}

Executer::~Executer()
{}

void Executer::executeCallBack(const std_msgs::Bool::ConstPtr& msg) {
	if(msg->data) {
		Execute();
	}
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

// void  Executer::updatePickAndEndPose(OrderPart * oPart){

// 	//set environment variable to true
// 	env_->setBinCameraRequired(true);
// 	ros::Duration(1.0).sleep();

// 	while(! env->isAllBinCalled() ) {
// 		ROS_INFO_STREAM("ALL BIN CAMERA IS NOT CALLED");
// 		ros::Duration(0.1).sleep();
// 	}

// 	auto binParts = env->getAllBinParts();

// 	bool flag = false;
// 	for(auto map_it = binParts->begin(); map_it!=binParts->end(); ++map_it) {
// 		auto part_type = map_it->first;
// 		if(oPart->getPartType() == part_type and map_it->second.size()) { // if part_type exists in bin parts
// 			oPart->setCurrentPose(map_it->second[0]);
// 			flag = true;
// 		}		
// 	}

// 	if (not flag) { // we need to pick up from conveyor belt here

// 	}

// 	return whether converyor or not
// }

void Executer::updatePickPose(OrderPart* order_) {
	//  we need to search for the part type in bin parts
	env_->setBinCameraRequired(true);
	
	while(! env_->isAllBinCameraCalled() ) {
		ROS_INFO_STREAM("ALL BIN CAMERA IS NOT CALLED...Waiting...");
		ros::Duration(0.1).sleep();
	}
	auto part_type = order_->getPartType();
	
	if (env_->isAllBinCameraCalled()) {
		auto allbinpart = env_->getSortedBinParts(); //std::map<std::string, std::vector<geometry_msgs::Pose> >*
		if (allbinpart->count(part_type)) {
			auto new_pose = (*allbinpart)[part_type].back();
			order_->setCurrentPose(new_pose);
			env_->setBinCameraRequired(false);
			env_->setAllBinCameraCalled(false);
		}
	}
}


void Executer::Execute() {

	ROS_INFO_STREAM("<<<<<<<In Execute modules");
	// Execute Pre-order tasks of arm1
	std::vector<std::map<std::string, std::vector<OrderPart*>>>* arm1preOrderParts = env_->getArm1PreOrderParts(); // std::vector<std::map<std::string, std::vector<OrderPart*>>>*
	for(auto po1_vec_it = arm1preOrderParts->begin(); po1_vec_it != arm1preOrderParts->end(); ++po1_vec_it) {
		
		for (auto po1_map_it = po1_vec_it->begin();po1_map_it != po1_vec_it->end(); ++po1_map_it) {
			
			for(auto po1_it = po1_map_it->second.begin();po1_it != po1_map_it->second.end(); ++po1_it) { // pol_it is basically iterator to std::vector<OrderPart*>
				ROS_INFO_STREAM("1");
				arm1_.pickPart((*po1_it)->getCurrentPose());
				ROS_INFO_STREAM("<<<<<<<arm1 going to quality bin");
				arm1_.GoToQualityCameraFromBin();
				env_->setSeeQualityCamera1(true);
				while(not env_->isQuality1Called() ) {
					ros::Duration(0.1).sleep();
					ROS_WARN_STREAM("Waiting for Quality Camera 1 to be called");
				}
				if(env_->isQualityCamera1Partfaulty()) {
					ROS_WARN_STREAM("Part is faulty");
					arm1_.dropInTrash();
					updatePickPose((*po1_it)); 
					--po1_it;
				} else {
					arm1_.deliverPart((*po1_it)->getEndPose());
				}
				env_->setSeeQualityCamera2(true);
			}
		}
	}
	arm1_.SendRobotHome();


	// Execute Pre-order tasks of arm2
	std::vector<std::map<std::string, std::vector<OrderPart*>>>* arm2preOrderParts = env_->getArm2PreOrderParts();
	for(auto po2_vec_it = arm2preOrderParts->begin(); po2_vec_it != arm2preOrderParts->end(); ++po2_vec_it) {
	
		for (auto po2_map_it = po2_vec_it->begin(); po2_map_it != po2_vec_it->end(); ++po2_map_it) {

			for(auto po2_it = po2_map_it->second.begin();po2_it != po2_map_it->second.end(); ++po2_it) {
				ROS_INFO_STREAM("2");
				arm2_.pickPart((*po2_it)->getCurrentPose());
				ROS_INFO_STREAM("<<<<<<<arm2 going to quality bin");
				arm2_.GoToQualityCameraFromBin();
				env_->setSeeQualityCamera2(true);
				while(not env_->isQuality2Called() ) {
					ros::Duration(0.1).sleep();
					ROS_WARN_STREAM("Waiting for Quality Camera 2 to be called");
				}
				if(env_->isQualityCamera2Partfaulty()) {
					ROS_WARN_STREAM("Part is faulty");
					arm2_.dropInTrash();
					updatePickPose((*po2_it));
					--po2_it;
				} else {
					arm2_.deliverPart((*po2_it)->getEndPose());
				}
				env_->setSeeQualityCamera2(false);
			}
		}
	}
	arm2_.SendRobotHome();

	auto arm1OrderParts = env_->getArm1OrderParts(); // std::vector<std::map<std::string, std::vector<OrderPart* > > >*
	for(auto o1_vec_it = arm1OrderParts->begin(); o1_vec_it != arm1OrderParts->end(); ++o1_vec_it) {
		
		for (auto o1_map_it = o1_vec_it->begin();o1_map_it != o1_vec_it->end(); ++o1_map_it) {

			for(auto o1_it = o1_map_it->second.begin();o1_it != o1_map_it->second.end(); ++o1_it) {
				ROS_INFO_STREAM("3");
				arm1_.pickPart((*o1_it)->getCurrentPose());
				arm1_.GoToQualityCameraFromBin();
				env_->setSeeQualityCamera1(true);
				while(not env_->isQuality1Called() ) {
					ros::Duration(0.1).sleep();
					ROS_WARN_STREAM("Waiting for Quality Camera 1 to be called");
				}
				if(env_->isQualityCamera1Partfaulty()) {
					ROS_WARN_STREAM("Part is faulty");
					arm1_.dropInTrash();
					updatePickPose((*o1_it));
					--o1_it;
				} else {
					ROS_INFO_STREAM("Part is not faulty");
					ROS_INFO_STREAM("Dropping in AGV");
					arm1_.deliverPart((*o1_it)->getEndPose());						
					// if()
					// removeItemFromOrderPart((*o1_it));
					// deleteTheOrderPart((*o1_it));
				}
			}
		}
	}
	
	auto arm2OrderParts = env_->getArm2OrderParts(); // std::vector<std::map<std::string, std::vector<OrderPart* > > >*
	
	for(auto o2_vec_it = arm2OrderParts->begin(); o2_vec_it != arm2OrderParts->end(); ++o2_vec_it) {
		
		for (auto o2_map_it = o2_vec_it->begin();o2_map_it != o2_vec_it->end(); ++o2_map_it) {

			for(auto o2_it = o2_map_it->second.begin();o2_it != o2_map_it->second.end(); ++o2_it) {
				ROS_INFO_STREAM("4");
				arm2_.pickPart((*o2_it)->getCurrentPose());
				arm2_.GoToQualityCameraFromBin();
				env_->setSeeQualityCamera2(true);
				while(not env_->isQuality2Called() ) {
					ros::Duration(0.1).sleep();
					ROS_WARN_STREAM("Waiting for Quality Camera 2 to be called");
				}
				if(env_->isQualityCamera1Partfaulty()) {
					ROS_WARN_STREAM("Part is faulty");
					arm2_.dropInTrash();
					updatePickPose((*o2_it));
					--o2_it;
				} else {
					ROS_INFO_STREAM("Part is not faulty");
					ROS_INFO_STREAM("Dropping in AGV");
					arm2_.deliverPart((*o2_it)->getEndPose());						
					// if()
					// removeItemFromOrderPart((*o1_it));
					// deleteTheOrderPart((*o1_it));
				}
			}
		}
	}
}


// void Executer::updateFaultyPartPose(OrderPart* part){
// 	if(faulty_part_ != nullptr){
// 		for (auto cam_it : all_binParts) {
// 			if(cam_it.second.count(part->getPartType())) {
// 				part->setCurrentPose(cam_it.second[part->getPartType()].back());
// 				return;
// 			}
// 		}
// 	}
// 	faulty_part_=nullptr;
// }
