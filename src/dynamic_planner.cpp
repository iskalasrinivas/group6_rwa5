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

#include <dynamic_planner.h>
#include <order_part.h>
#include <environment.h>
#include <robot_controller.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

DynamicPlanner::DynamicPlanner(Environment* env): async_spinner(0), env_(env), exe_(env){

	// ros::AsyncSpinner async_spinner(0); 
	async_spinner.start();

	dpllaner_sub_ = dpllaner_nh_.subscribe<std_msgs::Bool>("/ariac/execute_executer", 10 ,&DynamicPlanner::dynamicPlannerCallBack, this);

	
}

DynamicPlanner::~DynamicPlanner()
{}

void DynamicPlanner::dynamicPlannerCallBack(const std_msgs::Bool::ConstPtr& msg) {
	if(msg->data) {
		dynamicPlanning();
	}
}

void DynamicPlanner::updatePickPose(OrderPart* order_) {
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

// void DynamicPlanner::flipPart(OrderPart *order_)
// {
// 	if (!order_->getFlipPart())
// 	{
// 		// logic to flip the part
// 		auto target_pose = ; //define pose
// 		arm1_.GoToTarget(target_pose);
// 		arm1_.GripperToggle(false);
// 		ros::Duration(0.2).sleep();

// 		// after flipping the part set flip part = true
// 		order_->setFlipPart();
// 	}
// }

void DynamicPlanner::dynamicPlanning() {

	ROS_INFO_STREAM("<<<<<<<In Execute modules");
	// Execute Pre-order tasks of arm1
	auto arm1_ = exe_.getArm1Object();
	std::vector<std::map<std::string, std::vector<OrderPart*>>>* arm1preOrderParts = env_->getArm1PreOrderParts(); // std::vector<std::map<std::string, std::vector<OrderPart*>>>*
	for(auto po1_vec_it = arm1preOrderParts->begin(); po1_vec_it != arm1preOrderParts->end(); ++po1_vec_it) {
		
		for (auto po1_map_it = po1_vec_it->begin();po1_map_it != po1_vec_it->end(); ++po1_map_it) {
			
			for(auto po1_it = po1_map_it->second.begin();po1_it != po1_map_it->second.end(); ++po1_it) { // pol_it is basically iterator to std::vector<OrderPart*>
				ROS_INFO_STREAM("<<< Pre Order Arm1 >>>");
				arm1_->pickPart((*po1_it)->getCurrentPose());
				ROS_INFO_STREAM("<<<<<<<arm1 going to quality bin");
				arm1_->GoToQualityCameraFromBin();
				env_->setSeeQualityCamera1(true);
				while(not env_->isQuality1Called() ) {
					ros::Duration(0.1).sleep();
					ROS_WARN_STREAM("Waiting for Quality Camera 1 to be called");
				}
				if(env_->isQualityCamera1Partfaulty()) {
					ROS_WARN_STREAM("Part is faulty");
					arm1_->dropInTrash();
					updatePickPose((*po1_it)); 
					--po1_it;
				} else {
					arm1_->deliverPart((*po1_it)->getEndPose());
				}
				env_->setSeeQualityCamera2(true);
			}
		}
	}
	arm1_->SendRobotHome();


	// Execute Pre-order tasks of arm2
	auto arm2_ = exe_.getArm2Object();
	std::vector<std::map<std::string, std::vector<OrderPart*>>>* arm2preOrderParts = env_->getArm2PreOrderParts();
	for(auto po2_vec_it = arm2preOrderParts->begin(); po2_vec_it != arm2preOrderParts->end(); ++po2_vec_it) {
	
		for (auto po2_map_it = po2_vec_it->begin(); po2_map_it != po2_vec_it->end(); ++po2_map_it) {

			for(auto po2_it = po2_map_it->second.begin();po2_it != po2_map_it->second.end(); ++po2_it) {
				ROS_INFO_STREAM("<<< Pre Order Arm2 >>>");
				arm2_->pickPart((*po2_it)->getCurrentPose());
				ROS_INFO_STREAM("<<<<<<<arm2 going to quality bin");
				arm2_->GoToQualityCameraFromBin();
				env_->setSeeQualityCamera2(true);
				while(not env_->isQuality2Called() ) {
					ros::Duration(0.1).sleep();
					ROS_WARN_STREAM("Waiting for Quality Camera 2 to be called");
				}
				if(env_->isQualityCamera2Partfaulty()) {
					ROS_WARN_STREAM("Part is faulty");
					arm2_->dropInTrash();
					updatePickPose((*po2_it));
					--po2_it;
				} else {
					arm2_->deliverPart((*po2_it)->getEndPose());
				}
				env_->setSeeQualityCamera2(false);
			}
		}
	}
	arm2_->SendRobotHome();

	auto arm1OrderParts = env_->getArm1OrderParts(); // std::vector<std::map<std::string, std::vector<OrderPart* > > >*
	for(auto o1_vec_it = arm1OrderParts->begin(); o1_vec_it != arm1OrderParts->end(); ++o1_vec_it) {
		
		for (auto o1_map_it = o1_vec_it->begin();o1_map_it != o1_vec_it->end(); ++o1_map_it) {

			for(auto o1_it = o1_map_it->second.begin();o1_it != o1_map_it->second.end(); ++o1_it) {
				ROS_INFO_STREAM("<<< Order Arm1 >>>");
				arm1_->pickPart((*o1_it)->getCurrentPose());
				// arm1.flipPart((*o1_it));
				arm1_->GoToQualityCameraFromBin();
				env_->setSeeQualityCamera1(true);
				while(not env_->isQuality1Called() ) {
					ros::Duration(0.1).sleep();
					ROS_WARN_STREAM("Waiting for Quality Camera 1 to be called");
				}
				if(env_->isQualityCamera1Partfaulty()) {
					ROS_WARN_STREAM("Part is faulty");
					arm1_->dropInTrash();
					updatePickPose((*o1_it));
					--o1_it;
				} else {
					ROS_INFO_STREAM("Part is not faulty");
					ROS_INFO_STREAM("Dropping in AGV");
					arm1_->deliverPart((*o1_it)->getEndPose());						
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
				ROS_INFO_STREAM("<<< Order Arm2 >>>");
				arm2_->pickPart((*o2_it)->getCurrentPose());
				// arm1.flipPart((*o1_it));
				arm2_->GoToQualityCameraFromBin();
				env_->setSeeQualityCamera2(true);
				while(not env_->isQuality2Called() ) {
					ros::Duration(0.1).sleep();
					ROS_WARN_STREAM("Waiting for Quality Camera 2 to be called");
				}
				if(env_->isQualityCamera1Partfaulty()) {
					ROS_WARN_STREAM("Part is faulty");
					arm2_->dropInTrash();
					updatePickPose((*o2_it));
					--o2_it;
				} else {
					ROS_INFO_STREAM("Part is not faulty");
					ROS_INFO_STREAM("Dropping in AGV");
					arm2_->deliverPart((*o2_it)->getEndPose());						
					// if()
					// removeItemFromOrderPart((*o1_it));
					// deleteTheOrderPart((*o1_it));
				}
			}
		}
	}
}


// dinesh
bool DynamicPlanner::checkPoseTray1(){
	env_->setTrayCameraRequired(true);
	bool answer = false;
	if (!env_->getArm1OrderParts()->empty()) {
		auto agv1_FirstOrder = env_->getArm1OrderParts()->begin();
		auto r_tray1_parts = *(env_->getTray1Parts()); 
		
		for (auto traypart_map : r_tray1_parts) {
			auto part_type = traypart_map.first;
			auto part_vec = traypart_map.second;
			if (agv1_FirstOrder->count(part_type)) {
				for (auto O_it = (*agv1_FirstOrder)[part_type].begin(); O_it != (*agv1_FirstOrder)[part_type].end(); ++O_it) {
					for (auto t_it = part_vec.begin(); t_it != part_vec.end(); ++t_it) {
						if ((*O_it)->getEndPose() == *t_it) {
							return true;
						}
					}
				}
			}
		}
	}
	return answer;
}

// dinesh
bool DynamicPlanner::checkPoseTray2(){
	env_->setTrayCameraRequired(true);
	if (!env_->getArm2OrderParts()->empty()) {
		auto agv2_FirstOrder = env_->getArm2OrderParts()->begin();
		auto r_tray2_parts = *(env_->getTray2Parts()); 
		
		for (auto traypart_map : r_tray2_parts) {
			auto part_type = traypart_map.first;
			auto part_vec = traypart_map.second;
			if (agv2_FirstOrder->count(part_type)) {
				for (auto O_it = (*agv2_FirstOrder)[part_type].begin(); O_it != (*agv2_FirstOrder)[part_type].end(); ++O_it) {
					for (auto t_it = part_vec.begin(); t_it != part_vec.end(); ++t_it) {
						if ((*O_it)->getEndPose() == *t_it) {
							return true;
						}
					}
				}
			}
		}
	}
}


// raja

bool DynamicPlanner::inVicinity(const geometry_msgs::Pose& world_part_pose, RobotController* arm_) {
	double threshold_z = 0.1;
	double threshold_y = 0.35;
	return (arm_->getHomeCartPose().position.z-
			world_part_pose.position.z < threshold_z &&
			arm_->getHomeCartPose().position.y-
			world_part_pose.position.y < threshold_y);
}

void DynamicPlanner::pickPartFromBelt(std::string arm, geometry_msgs::Pose world_part_pose, double y){
    RobotController* arm_;
    if(arm == "arm1"){
		 arm_ = exe_.getArm1Object();
	}
	else if(arm == "arm2"){
		 arm_ = exe_.getArm2Object();
	}
	if (!arm_->isPartAttached()) {
        world_part_pose.position.z += 0.02;
		world_part_pose.position.y -= y; 
		arm_->GoToTarget(world_part_pose);
		if (inVicinity(world_part_pose, arm_)) {
			ROS_WARN_STREAM("Gripper toggled");
			arm_->GripperToggle(true);
			while (!arm_->isPartAttached()) {
				ROS_WARN_STREAM("Part not attached");
				world_part_pose.position.z += 0.004;
				world_part_pose.position.y -= 2*y;
				arm_->GoToTarget(world_part_pose);
				world_part_pose.position.z -= 0.004;
				world_part_pose.position.y -= 2*y;
				arm_->GoToTarget(world_part_pose);
			}
			ROS_INFO_STREAM("Part attached");
			world_part_pose.position.z += 0.2;
			world_part_pose.position.y -= y;
			arm_->GoToTarget(world_part_pose);
		}
	} 	
}
