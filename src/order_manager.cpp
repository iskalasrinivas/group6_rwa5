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

OrderManager::OrderManager(Environment *env) : environment(env), order_segregated(false)
{
    ros::AsyncSpinner async_spinner(4);
    async_spinner.start();
    order_ = NULL;
    order_subscriber_ = order_manager_nh_.subscribe("/ariac/orders", 10,
                                                    &OrderManager::OrderCallback, this);
    while (!environment->isAllBinCameraCalled())
    {
        ROS_INFO_STREAM("Bin camera is not called" << std::endl);
    }
    ROS_INFO_STREAM("Bin camera Called!! Starting segregating Orders." << std::endl);
    ros::Duration(1).sleep();
    updateAllOrderPickupLocation();
}

OrderManager::~OrderManager() {}

void OrderManager::OrderCallback(const osrf_gear::Order::ConstPtr &order_msg)
{
    ROS_WARN(">>>>> OrderCallback");
    setOrderParts(order_msg);
    updateAllOrder();
}
// once the part are categorized in agv1 and agv2 we need to compare with parts
// available in agv1 and agv2 resp and then remove the unnecessary part from the tray
// displace the parts in tray itself and add the new parts from bin.
void OrderManager::updateAllOrder()
{

    // UPDATE for AGV1
    if (!environment->getArm1OrderParts()->empty())
    {
        auto agv1_FirstOrder = environment->getArm1OrderParts()->begin();
        auto r_tray1_parts = *(environment->getTray1Parts()); // std::map<std::string, std::vector<geometry_msgs::Pose>>*
        // agv1_FirstOrder.second
        for (auto traypart_map : r_tray1_parts)
        { // for part on tray
            auto part_type = traypart_map.first;
            auto part_vec = traypart_map.second;
            if (agv1_FirstOrder->count(part_type))
            { // If part on tray is part of order, keep part on tray if end-pose matches
                for (auto O_it = (*agv1_FirstOrder)[part_type].begin(); O_it != (*agv1_FirstOrder)[part_type].end(); ++O_it)
                {
                    for (auto t_it = part_vec.begin(); t_it != part_vec.end(); ++t_it)
                    {
                        if ((*O_it)->getEndPose() == *t_it)
                        {
                            part_vec.erase(t_it);
                            (*agv1_FirstOrder)[part_type].erase(O_it);
                        }
                    }
                }
            }
        }
        //upto here all the parts whose end pose are same are removed from the order and removed from tray's remove-part-list
        std::map<std::string, std::vector<OrderPart *>> displaceParts;
        // Here, update the pose of parts left on tray i.e.segregate them into two sections 1. trash_parts 2. tray_parts_with_different_pose
        for (auto tray_part_it = r_tray1_parts.begin(); tray_part_it != r_tray1_parts.end(); ++tray_part_it)
        {
            auto part_type = (*tray_part_it).first;
            auto part_vec = (*tray_part_it).second;

            if (agv1_FirstOrder->count(part_type))
            { // add it to tray_parts_with_differnt_pose
                for (auto order_it = (*agv1_FirstOrder)[part_type].begin(); order_it != (*agv1_FirstOrder)[part_type].end(); ++order_it)
                {
                    for (auto displace_it = part_vec.begin(); displace_it != part_vec.end(); ++displace_it)
                    {
                        (*order_it)->setCurrentPose(*displace_it);
    {
        auto agv2_FirstOrder = environment->getArm2OrderParts()->begin();
        auto r_tray2_parts = *(environment->getTray2Parts());
        // agv2_FirstOrder.second
        for (auto traypart_map : r_tray2_parts)
        { // for part on tray
            auto part_type = traypart_map.first;
            auto part_vec = traypart_map.second;
            if (agv2_FirstOrder->count(part_type))
            { // If part on tray is part of order, keep part on tray if end-pose matches
                for (auto O_it = (*agv2_FirstOrder)[part_type].begin(); O_it != (*agv2_FirstOrder)[part_type].end(); ++O_it)
                {
                    for (auto t_it = part_vec.begin(); t_it != part_vec.end(); ++t_it)
                    {
                        if ((*O_it)->getEndPose() == *t_it)
                        {
                            part_vec.erase(t_it);
                            (*agv2_FirstOrder)[part_type].erase(O_it);
                        }
                    }
                }
            }
        }
        //upto here all the parts whose end pose are same are removed from the order and removed from tray's remove-part-list
        std::map<std::string, std::vector<OrderPart *>> displaceParts;
        // Here, update the pose of parts left on tray i.e.segregate them into two sections 1. trash_parts 2. tray_parts_with_different_pose
        for (auto tray_part_it = r_tray2_parts.begin(); tray_part_it != r_tray2_parts.end(); ++tray_part_it)
        {
            auto part_type = (*tray_part_it).first;
            auto part_vec = (*tray_part_it).second;

            if (agv2_FirstOrder->count(part_type))
            { // add it to tray_parts_with_differnt_pose
                for (auto order_it = (*agv2_FirstOrder)[part_type].begin(); order_it != (*agv2_FirstOrder)[part_type].end(); ++order_it)
                {
                    for (auto displace_it = part_vec.begin(); displace_it != part_vec.end(); ++displace_it)
                    {
                        (*order_it)->setCurrentPose(*displace_it);
                        if (displaceParts.count(part_type))
                        {
                            displaceParts[part_type].emplace_back(*order_it);
                        }
                        else
                        {
                            std::vector<OrderPart *> order_vec{*order_it};
                            displaceParts[part_type] = order_vec;
                            ;
                        }
                        part_vec.erase(displace_it);
                        (*agv2_FirstOrder)[part_type].erase(order_it);
                    }
                }
            }
        }

        std::map<std::string, std::vector<OrderPart *>> trashParts = getTrashParts(r_tray2_parts);
        auto it = environment->getArm2OrderParts()->begin();
        environment->getArm2OrderParts()->insert(it, displaceParts);
        environment->getArm2OrderParts()->insert(it, trashParts);
    }
}

std::map<std::string, std::vector<OrderPart *>> OrderManager::getTrashParts(std::map<std::string, std::vector<geometry_msgs::Pose>> r_tray1_parts)
{

    std::map<std::string, std::vector<OrderPart *>> return_var;

    for (auto it = r_tray1_parts.begin(); it != r_tray1_parts.end(); ++it)
    {

        auto part_type = it->first;

        return_var[part_type] = std::vector<OrderPart *>();

        for (auto pose_it = it->second.begin(); pose_it != it->second.end(); ++pose_it)
        {

            OrderPart *trash_part = new OrderPart();
            trash_part->setPartType(part_type);
            trash_part->setCurrentPose(*pose_it);
            trash_part->setEndPose(*end_pose); // TO-DO --- DEFINE END_POSE of trash bin
            return_var[part_type].emplace_back(trash_part);
        }
    }
    return return_var;
}

void OrderManager::setOrderParts(const osrf_gear::Order::ConstPtr &order_msg)
{
    ROS_INFO_STREAM("reading order." << std::endl);

    auto order_id = order_msg->order_id;
    auto shipments = order_msg->shipments;
    auto agv1_OrderParts = environment->getArm1OrderParts();
    auto agv2_OrderParts = environment->getArm2OrderParts();
    for (const auto &shipment : shipments)
    {
        std::map<std::string, std::vector<OrderPart *>> shipment_Parts;
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
        if (agv_id == "agv_1" || agv_id == "any")
        {
            agv1_OrderParts->push_back(shipment_Parts);
        }
        else
        {
            agv2_OrderParts->push_back(shipment_Parts);
        }
    }
}
void updateAllOrderPickupLocation() {
    updatePickupLocation(env_->getArm1OrderParts());.
    updatePickupLocation(env_->getArm2OrderParts());

}

void OrderManager::updatePickupLocation(std::vector<std::map<std::string, std::vector<OrderPart *>>> *orderParts)
{

    for (const auto &orderPartsVec : orderParts)
    {
        for (const auto &orderPart : orderPartsVec)
        {
            auto part_type = orderPart.first;
            //		    ROS_INFO_STREAM( "order Part type :"<< part_type);
            auto oVecPart = orderPart.second;
            auto sorted_all_binParts = environment->getSortedBinParts;
            if (sorted_all_binParts->count(part_type))
            {
                auto bin_vec = (*sorted_all_binParts)[part_type];
                //			    ROS_INFO_STREAM( "order PArt type"<< bin_vec.front());
                if (bin_vec.size() >= orderPart.second.size())
                {
                    auto opart_it = oVecPart.begin();
                    auto bin_part = bin_vec.begin();

                    for (opart_it = oVecPart.begin(), bin_part = bin_vec.begin(); opart_it != oVecPart.end(); ++opart_it, ++bin_part)
                    {
                        (*opart_it)->setCurrentPose(*bin_part);
                        //					ROS_WARN_STREAM( "order Type and Current Pose from bin"<<(*opart_it)->getPartType() << " " << (*opart_it)->getCurrentPose());
                    }
                    //				setCurrentPose(oVecPart, (*sorted_all_binParts)[part_type]);
                    // bin_order_parts.insert({part_type, oVecPart});
                }
            }

            //     else {
            //         ROS_INFO_STREAM("Sufficient Parts not available on bin lesser parts:have to be coded !!");
            //     }
        }
    }

    env_->setPickLocationAdded(true);
}