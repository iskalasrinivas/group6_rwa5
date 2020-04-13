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

OrderManager::OrderManager(Environment *env): environment(env){
	ros::AsyncSpinner async_spinner(4);
	async_spinner.start();
    order_= NULL;
	order_subscriber_ = order_manager_nh_.subscribe("/ariac/orders", 10,
			&OrderManager::OrderCallback, this);
}

OrderManager::~OrderManager() {}


void OrderManager::OrderCallback
(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_WARN(">>>>> OrderCallback");
    setOrderParts(order_msg);
        
}
// once the part are categorized in agv1 and agv2 we need to compare with parts
// available in agv1 and agv2 resp and then remove the unnecessary part from the tray
// displace the parts in tray itself and add the new parts from bin.
void OrderManager::updateAllOrder(){
    if(!environment->getAgv1OrderParts().empty()){
        auto agv1_FirstOrder = *(environment->getAgv1OrderParts().begin());
        auto r_tray1_parts = environment->getTray1Parts();
        for(auto traypart_map : r_tray1_parts){
          auto part_type = traypart_map->first;
          auto part_vec = traypart_map->second;
          if(agv1_FirstOrder[part_type].count) {
            for(auto O_it = agv1_FirstOrder[part_type].begin(); O_it != agv1_FirstOrder[part_type].end(), ++O_it) {
              for(auto t_it = part_vec.begin(); t_it != part_vec.end(), ++t_it) {
                  if(O_it->getEndPose == t_it->pose) {
                      //remove this item from r_tray1_parts
                  }
              }
            }
            
          }
          
          }

        }
    }
    if(!environment->getAgv2OrderParts().empty()){
    auto agv2_FirstOrder = *(environment->getAgv2OrderParts().begin());
    auto tray2_parts = environment->getTray2Parts();
    for(auto part_map : tray1_parts){
        auto part_type = part_map->first;
        auto part_vec = part_map->second;
        if(part_vec.size() > agv1_OrderParts.){

        }
    }
    
   
}

void OrderManager::setOrderParts(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_INFO_STREAM("reading order." << std::endl);

        auto order_id = order_msg->order_id;
        auto shipments = order_msg->shipments;
        auto agv1_OrderParts = environment->getAgv1OrderParts();
        auto agv2_OrderParts = environment->getAgv2OrderParts();
        for (const auto &shipment : shipments)
        {
            std::map<std::string, std::vector<OrderPart*>> shipment_Parts;
            auto shipment_type = shipment.shipment_type;
            auto agv_id = shipment.agv_id;
            auto products = shipment.products;

            for (const auto &product : products)
            {
                std::string part_type = product.type;
                if (shipment_Parts.count(part_type))
                {
                    OrderPart *order_part = new OrderPart(agv_id, part_type, product.pose);

                    shipment_Parts[part_type].push_back(order_part);
                }
                else
                {
                    OrderPart *order_part = new OrderPart(agv_id, part_type, product.pose);
                    std::vector<OrderPart *> vec;
                    vec.push_back(order_part);
                    shipment_Parts[part_type] = vec;
                }
            }
            if(agv_id == "agv_1" || agv_id == "any"){
                agv1_OrderParts->push_back(shipment_Parts);
            }
            else {
                agv2_OrderParts->push_back(shipment_Parts);
            }
            
        }
}
