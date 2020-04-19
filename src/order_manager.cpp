/**
 * @file      src/order_manager.cpp
 * @brief     Source file for order manager
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

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <boost/optional.hpp>
#include <order_manager.h>
#include <environment.h>
#include <std_msgs/Bool.h>

OrderManager::OrderManager(Environment *env) :  async_spinner(4), environment(env) {
	async_spinner.start();
	order_ = NULL;
	order_subscriber_ = order_manager_nh_.subscribe("/ariac/orders", 10,
			&OrderManager::OrderCallback, this);
	ROS_INFO_STREAM("order manager is called. "<< boost::this_thread::get_id());	

	execute_planner = order_manager_nh_.advertise<std_msgs::Bool>("/ariac/execute_planner", 1000);

	// while (!environment->isAllBinCameraCalled())
	// {
	// 	ROS_INFO_STREAM("All Bin cameras are not called" << std::endl);
	// }
	// ROS_INFO_STREAM("Bin camera Called!! Starting segregating Orders." << std::endl);
	// ros::Duration(0.1).sleep();
	// updatePickupLocation();
}

OrderManager::~OrderManager() {}

void OrderManager::OrderCallback(const osrf_gear::Order::ConstPtr &order_msg) {
	ROS_WARN(">>>>> OrderCallback");
	setOrderParts(order_msg); //  Independent of bin
	setArmForAnyParts(); // raja
	updateAllOrder(); // check whether tray camera is called
	updatePickupLocation();// check whether bin camera is called
}
// once the part are categorized in agv1 and agv2 we need to compare with parts
// available in agv1 and agv2 resp and then remove the unnecessary part from the tray
// displace the parts in tray itself and add the new parts from bin.
void OrderManager::updateAllOrder() {
	environment->setTrayCameraRequired(true);
	ros::Duration(1.0).sleep();
	
	while (!environment->isAllTrayCameraCalled())
	{
		ROS_WARN_STREAM("All Tray cameras are not called" << std::endl);
		ros::Duration(0.1).sleep();
	}
	ROS_INFO_STREAM("All Tray camera Called!!" << std::endl);
	ros::Duration(0.1).sleep();

	// UPDATE for AGV1
	if (!environment->getArm1OrderParts()->empty()) {
		auto agv1_FirstOrder = environment->getArm1OrderParts()->begin();
		auto r_tray1_parts = *(environment->getTray1Parts()); // std::map<std::string, std::vector<geometry_msgs::Pose>>*
		// agv1_FirstOrder.second
		ROS_INFO_STREAM("Parts in tray1 " << r_tray1_parts.size());
		for (auto traypart_map : r_tray1_parts) { // for part on tray
			ROS_INFO_STREAM("There are parts in tray1");
			auto part_type = traypart_map.first;
			auto part_vec = traypart_map.second;
			if (agv1_FirstOrder->count(part_type)) { // If part on tray is part of order, keep part on tray if end-pose matches
				for (auto O_it = (*agv1_FirstOrder)[part_type].begin(); O_it != (*agv1_FirstOrder)[part_type].end(); ++O_it) {
					for (auto t_it = part_vec.begin(); t_it != part_vec.end(); ++t_it) {
						if ((*O_it)->getEndPose() == *t_it) {
							part_vec.erase(t_it);
							(*agv1_FirstOrder)[part_type].erase(O_it);
						}
					}
				}
			}
		}
		//upto here all the parts whose end pose are same are removed from the order and removed from tray's remove-part-list
		std::map<std::string, std::vector<OrderPart *>> displaceParts1;
		// Here, update the pose of parts left on tray i.e.segregate them into two sections 1. trash_parts 2. tray_parts_with_different_pose
		for (auto tray_part_it = r_tray1_parts.begin(); tray_part_it != r_tray1_parts.end(); ++tray_part_it) {
			ROS_INFO_STREAM("Displace : There are parts in tray1");
			auto part_type = (*tray_part_it).first;
			auto part_vec = (*tray_part_it).second;

			if (agv1_FirstOrder->count(part_type)) { // add it to tray_parts_with_differnt_pose
				auto displace_it = part_vec.begin();
				auto order_it = (*agv1_FirstOrder)[part_type].begin();
				for (order_it = (*agv1_FirstOrder)[part_type].begin(); order_it != (*agv1_FirstOrder)[part_type].end(); ++order_it, ++displace_it) {
					// for (auto displace_it = part_vec.begin(); displace_it != part_vec.end(); ++displace_it) {
					(*order_it)->setCurrentPose(*displace_it);
					if(displaceParts1.count(part_type)) {
						displaceParts1[part_type].emplace_back(*order_it);
					}
					else {
						vector<OrderPart*>dis_vec{(*order_it)};
						displaceParts1[part_type] = dis_vec;
					}
					part_vec.erase(displace_it);
					(*agv1_FirstOrder)[part_type].erase(order_it);
				}
			}
		}

		std::map<std::string, std::vector<OrderPart*>> trashParts1 = getTrashParts(r_tray1_parts);
		auto  it = environment->getArm1OrderParts()->begin();
		if(trashParts1.size() !=0) {
			environment->getArm1OrderParts()->insert(it, displaceParts1);
		}
		if(trashParts1.size() !=0) {
			environment->getArm1OrderParts()->insert(it, trashParts1);
		}
	}
	
	
	// UPDATE FOR AGV2

	if (!environment->getArm2OrderParts()->empty()) {
		auto agv2_FirstOrder = environment->getArm2OrderParts()->begin();
		auto r_tray2_parts = *(environment->getTray2Parts()); // std::map<std::string, std::vector<geometry_msgs::Pose>>*

		for (auto traypart_map : r_tray2_parts) { // for part on tray
			auto part_type = traypart_map.first;
			auto part_vec = traypart_map.second;
			if (agv2_FirstOrder->count(part_type)) { // If part on tray is part of order, keep part on tray if end-pose matches
				for (auto O_it = (*agv2_FirstOrder)[part_type].begin(); O_it != (*agv2_FirstOrder)[part_type].end(); ++O_it) {
					for (auto t_it = part_vec.begin(); t_it != part_vec.end(); ++t_it) {
						if ((*O_it)->getEndPose() == *t_it) {
							part_vec.erase(t_it);
							(*agv2_FirstOrder)[part_type].erase(O_it);
						}
					}
				}
			}
		}

		//upto here all the parts whose end pose are same are removed from the order and removed from tray's remove-part-list
		std::map<std::string, std::vector<OrderPart *>> displaceParts2;
		// Here, update the pose of parts left on tray i.e.segregate them into two sections 1. trash_parts 2. tray_parts_with_different_pose
		for (auto tray_part_it = r_tray2_parts.begin(); tray_part_it != r_tray2_parts.end(); ++tray_part_it) {
			auto part_type = (*tray_part_it).first;
			auto part_vec = (*tray_part_it).second;

			if (agv2_FirstOrder->count(part_type)) { // add it to tray_parts_with_differnt_pose
				auto displace_it = part_vec.begin();
				auto order_it = (*agv2_FirstOrder)[part_type].begin();
				for (order_it = (*agv2_FirstOrder)[part_type].begin(); order_it != (*agv2_FirstOrder)[part_type].end(); ++order_it, ++displace_it) {
					// for (auto displace_it = part_vec.begin(); displace_it != part_vec.end(); ++displace_it) {
					(*order_it)->setCurrentPose(*displace_it);
					if(displaceParts2.count(part_type)) {
						displaceParts2[part_type].emplace_back(*order_it);
					}
					else {
						vector<OrderPart*>dis_vec{(*order_it)};
						displaceParts2[part_type] = dis_vec;
					}
					part_vec.erase(displace_it);
					(*agv2_FirstOrder)[part_type].erase(order_it);
				}
			}
		}

		std::map<std::string, std::vector<OrderPart*>> trashParts2 = getTrashParts(r_tray2_parts);
		auto it = environment->getArm2OrderParts()->begin();
		if(trashParts2.size() !=0) {
			environment->getArm2OrderParts()->insert(it, displaceParts2);
		}
		if(trashParts2.size() !=0) {
			environment->getArm2OrderParts()->insert(it, trashParts2);
		}
	}
}

// this is used to convert pose to OrderPart
std::map<std::string, std::vector<OrderPart *>> OrderManager::getTrashParts(std::map<std::string, std::vector<geometry_msgs::Pose>> r_tray1_parts) {
	std::map<std::string, std::vector<OrderPart *>> return_var;

	for (auto it = r_tray1_parts.begin(); it != r_tray1_parts.end(); ++it)
	{
		ROS_INFO_STREAM("getTrashParts : There are parts in tray1");
		auto part_type = it->first;

		return_var[part_type] = std::vector<OrderPart *>();

		for (auto pose_it = it->second.begin(); pose_it != it->second.end(); ++pose_it)
		{

			OrderPart *trash_part = new OrderPart();
			trash_part->setPartType(part_type);
			trash_part->setCurrentPose(*pose_it);
			trash_part->setEndPose(environment->getTrashBinPose());
			return_var[part_type].emplace_back(trash_part);	
		}
	}
	return return_var;
}


void OrderManager::setOrderParts(const osrf_gear::Order::ConstPtr &order_msg) {
	ROS_INFO_STREAM("<<<<Reading order>>>>>" << std::endl);
	auto order_id = order_msg->order_id;
	auto shipments = order_msg->shipments;
	auto agv1_OrderParts = environment->getArm1OrderParts();
	auto agv2_OrderParts = environment->getArm2OrderParts();
	agv1_OrderParts->clear();
	agv2_OrderParts->clear();
	for (const auto &shipment : shipments) {
		std::map<std::string, std::vector<OrderPart *>> shipment_Parts;
		auto shipment_type = shipment.shipment_type;
		auto agv_id = shipment.agv_id;
		auto products = shipment.products;
		for (const auto &product : products) {
			std::string part_type = product.type;
			if (shipment_Parts.count(part_type)) {
				OrderPart *order_part = new OrderPart(agv_id, part_type, product.pose);
				shipment_Parts[part_type].push_back(order_part);
			} else {
				OrderPart *order_part = new OrderPart(agv_id, part_type, product.pose);
				std::vector<OrderPart *> vec;
				vec.push_back(order_part);
				shipment_Parts[part_type] = vec;
			}
		}
		if (agv_id == "agv_1") {
			agv1_OrderParts->push_back(shipment_Parts);
			
		} else if (agv_id == "agv_2") {
			agv2_OrderParts->push_back(shipment_Parts);
		} else {
			agv1_OrderParts->push_back(shipment_Parts);
			agv2_OrderParts->push_back(shipment_Parts);
			agv1_any.push_back(agv1_OrderParts->end()-1);
			agv2_any.push_back(agv2_OrderParts->end()-1);
		}
	}
}

// Once we have order, update currentpose based on poses from binparts, if it is part of BIN OFCOURSE
void OrderManager::updatePickupLocation() {

	environment->setBinCameraRequired(true);
	ros::Duration(1.0).sleep();

	while (!environment->isAllBinCameraCalled()) {
//		ROS_INFO_STREAM("All Bin cameras are not called" << std::endl);
		ros::Duration(0.1).sleep();
	}
//	ROS_INFO_STREAM("updatePickupLocation : All Bin cameras are called");
	auto sorted_all_binParts = *(environment->getSortedBinParts());
//	ROS_INFO_STREAM("updatePickupLocation : TYpe of Parts:" << sorted_all_binParts.size() );
    auto conveyor_arm1_parts = environment->getArm1ConveyorOrderParts();
	auto conveyor_arm2_parts = environment->getArm2ConveyorOrderParts();

	// process arm1 order parts and assign poses and mark them as assigned by deleting them!
	for (auto orderPartsVec : (*environment->getArm1OrderParts())) {
		for (auto orderPart : orderPartsVec) {
			auto part_type = orderPart.first;
			//		    ROS_INFO_STREAM( "order Part type :"<< part_type);
			auto oVecPart = orderPart.second;
			
			if (sorted_all_binParts.count(part_type)) {
				auto bin_vec = sorted_all_binParts[part_type];
				//			    ROS_INFO_STREAM( "order PArt type"<< bin_vec.front());
				if (bin_vec.size() >= orderPart.second.size()) {
					auto opart_it = oVecPart.begin();
					auto bin_part_pose = bin_vec.begin();

					for (opart_it = oVecPart.begin(), bin_part_pose = bin_vec.begin(); opart_it != oVecPart.end(); ++opart_it, ++bin_part_pose) {
						(*opart_it)->setCurrentPose(*bin_part_pose);
						ROS_INFO_STREAM("Order Initial Pose : "<< bin_part_pose->position.x << " " << bin_part_pose->position.y << " " << bin_part_pose->position.z);
						bin_vec.erase(bin_part_pose);
					}
				} else {
					//some of the part are available
					// choose parts available on bin from bin anbd the rest from conveyor belt
					auto opart_it = oVecPart.begin();
					auto bin_part_pose = bin_vec.begin();
					std::vector<OrderPart*> vec;
					for (opart_it = oVecPart.begin(), bin_part_pose = bin_vec.begin(); bin_part_pose != bin_vec.end(); ++opart_it, ++bin_part_pose) {
						(*opart_it)->setCurrentPose(*bin_part_pose);
						bin_vec.erase(bin_part_pose);
					
					}
					while(opart_it != oVecPart.end()){
					  if((*conveyor_arm1_parts).count(part_type)){
						   (*conveyor_arm1_parts)[part_type].push_back(*opart_it);
					  }
					  else{
						  vec.push_back(*opart_it);
						  (*conveyor_arm1_parts).insert({part_type, vec});
					  }
					}	
				}
			} else {
   				//none  of the part are available
				// choose everythinhg from conveyor belt
				(*conveyor_arm1_parts).insert({part_type, oVecPart}); //raja
			}
		}
	}
	
	// process arm2 order parts and assign poses and mark them as assigned by deleting them!
	for (const auto &orderPartsVec : (*environment->getArm2OrderParts())) {
		ROS_INFO_STREAM("Updating Part pick location for arm2");
		for (const auto &orderPart : orderPartsVec) {
			auto part_type = orderPart.first;
			//		    ROS_INFO_STREAM( "order Part type :"<< part_type);
			auto oVecPart = orderPart.second;
			
			if (sorted_all_binParts.count(part_type)) {
				auto bin_vec = sorted_all_binParts[part_type];
				//			    ROS_INFO_STREAM( "order PArt type"<< bin_vec.front());
				if (bin_vec.size() >= orderPart.second.size()) {
					auto opart_it = oVecPart.begin();
					auto bin_part = bin_vec.begin();

					for (opart_it = oVecPart.begin(), bin_part = bin_vec.begin(); opart_it != oVecPart.end(); ++opart_it, ++bin_part) {
						(*opart_it)->setCurrentPose(*bin_part);
						bin_vec.erase(bin_part);
					}
				} else {
					//some of the part are available
					//some of the part are available
					// choose parts available on bin from bin anbd the rest from conveyor belt
					auto opart_it = oVecPart.begin();
					auto bin_part_pose = bin_vec.begin();
					std::vector<OrderPart*> vec;
					for (opart_it = oVecPart.begin(), bin_part_pose = bin_vec.begin(); bin_part_pose != bin_vec.end(); ++opart_it, ++bin_part_pose) {
						(*opart_it)->setCurrentPose(*bin_part_pose);
						bin_vec.erase(bin_part_pose);
					
					}
					while(opart_it != oVecPart.end()){
					  if((*conveyor_arm2_parts).count(part_type)){
						   (*conveyor_arm2_parts)[part_type].push_back(*opart_it);
					  }
					  else{
						  vec.push_back(*opart_it);
						  (*conveyor_arm2_parts).insert({part_type, vec});
					  }
					}
				}
			} else {
   				//none  of the part are available
				(*conveyor_arm2_parts).insert({part_type, oVecPart}); //raja
			}
		}
	}
	

	environment->setAllBinCameraCalled(false);
	environment->setorderManagerStatus(true);
	std_msgs::Bool msg;
	msg.data = true;
	for(size_t i=0; i<1; ++i){
		execute_planner.publish(msg);
	}
	ROS_INFO_STREAM("!!! Order Manager has completed it's processing !!!");
}

void OrderManager::setArmForAnyParts() {
	auto agv1_score = 0;
	auto agv2_score = 0;
	auto sorted_all_binParts = environment->getSortedBinParts();
	auto tray1_Parts = environment->getTray1Parts();
	auto tray2_Parts = environment->getTray2Parts();
	auto agv1_OrderParts = environment->getArm1OrderParts();
	auto agv2_OrderParts = environment->getArm2OrderParts();

	// we are not ignoring tray parts 
    for (auto  it_agv1 = agv1_any.begin(), it_agv2 = agv2_any.begin(); it_agv1 != agv1_any.end(), it_agv2 != agv2_any.end(); ++it_agv1, ++it_agv2){
		  auto iterator_agv1_parts = *it_agv1;
		  auto iterator_agv2_parts = *it_agv2;
		for (auto orderPartsVec : *iterator_agv1_parts){
			for(auto orderPart : *orderPartsVec){
				auto part_type = orderPart.first;
				auto part_vec = orderPart.second;
				auto o_it = part_vec.begin()
				std::map<std::string, std::vector<geometry_msgs::Pose> >*::iterator tray1_it, tray2_it;					
                
				if(tray1_Parts->count(part_type)){
					auto tray1_poses = (*tray1_Parts)[part_type];
					auto tray1_it = tray1_Parts->begin();
				}
				if(tray2_Parts->count(part_type)){
					auto tray2_poses = (*tray2_Parts)[part_type];
					auto tray2_it = tray2_Parts->begin();
				}
				if(sorted_all_binParts->count(part_type)){
                     auto vec_bin_poses = (*sorted_all_binParts)[part_type];
					 auto b_it = vec_bin_poses.begin(); 
					 
					for(o_it = part_vec.begin(); o_it !=  part_vec.end(), b_it != vec_bin_poses.end(); ++o_it, ++b_it) {
						if(tray1_it != tray1_Parts->end()) {
							agv1_score++;
							agv1_score++;
							++tray1_it;	
						}
						if(tray2_it != tray2_Parts->end()) {
							agv2_score++;
							agv2_score++;
							++tray2_it;
						}
						if(b_it.position.y > -1.5 and b_it.position.y < 1.5){
							agv1_score++;
							agv2_score++;
						} else if(b_it.position.y >= 1.5) {
							agv1_score++;
						} else if(b_it.position.y <= -1.5){
							agv2_score++;
						}
					}
				}
			}
		}

	    if(agv1_score >= agv2_score) {
			//we need agv1 for any
			agv2_OrderParts->erase(*it_agv2);
			auto shipment = *it_agv1;
			for(auto orderPart :  shipment) {
				auto part_type = orderPart.first;
				auto part_vec = orderPart.second;
				for(auto o_it = part_vec.begin(); o_it !=  part_vec.end(); ++o_it) {
					(*o_it)->setTrayId("agv_1");
					(*o_it)->worldTransformation();
			    }   
		    }

			
	    } else {
			//we need agv2 for any
			agv1_OrderParts->erase(*it_agv1);
			auto shipment = *it_agv2;
			for(auto orderPart :  shipment) {
				auto part_type = orderPart.first;
				auto part_vec = orderPart.second;
				for(auto o_it = part_vec.begin(); o_it !=  part_vec.end(); ++o_it) {
					(*o_it)->setTrayId("agv_2");
					(*o_it)->worldTransformation();
			}
	    }
		agv1_score = 0;
		agv2_score = 0;

	   }

	}
}
// vec(*shipment) 

// shipment (piston rod 2 and gasket 3) compare the score based  
// all parts are availabe to arm1 , gasket is not availanbe to arm2
// compare with tray parts and bin parts
// arm1 cost : 2, arm2 : 3
// delete the shipment from the agv2_order.
//  
//