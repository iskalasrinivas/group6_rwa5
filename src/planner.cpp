/**
 * @file      include/competition.h
 * @brief     Header file for competition
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

#include <planner.h>

Planner::Planner(Environment *env) : env_(env), ordermanager_(env_), arm1_("arm1"),arm2_("arm2"){
    // Initialize the common pose index 
    common_pose_ind = 0;
    
    // Hardcode several common pose locations
    common_pose_[0].position.x = -0.5;
    common_pose_[0].position.y = 0.583010;
    common_pose_[0].position.z = 0.724102;
    common_pose_[0].orientation.x = 0;
    common_pose_[0].orientation.y = 0;
    common_pose_[0].orientation.z = 0;
    common_pose_[0].orientation.w = 0;

    common_pose_[1].position.x = -0.5;
    common_pose_[1].position.y = 0.283010;
    common_pose_[1].position.z = 0.724102;
    common_pose_[1].orientation.x = 0;
    common_pose_[1].orientation.y = 0;
    common_pose_[1].orientation.z = 0;
    common_pose_[1].orientation.w = 0;    
    
    common_pose_[2].position.x = -0.2;
    common_pose_[2].position.y = 0.283010;
    common_pose_[2].position.z = 0.724102;
    common_pose_[2].orientation.x = 0;
    common_pose_[2].orientation.y = 0;
    common_pose_[2].orientation.z = 0;
    common_pose_[2].orientation.w = 0;

    common_pose_[3].position.x = -0.2;
    common_pose_[3].position.y = 0.583010;
    common_pose_[3].position.z = 0.724102;
    common_pose_[3].orientation.x = 0;
    common_pose_[3].orientation.y = 0;
    common_pose_[3].orientation.z = 0;
    common_pose_[3].orientation.w = 0;


    // std::vector<geometry::Pose>
    // for ()
    // common_pose[0]
}

Planner::~Planner()
{}


void Planner::plan() {
    // auto sorted_BinParts = environment->getSortedBinParts();  std::vector<std::map<std::string, std::vector<OrderPart*>>>
    auto agv1_OrderParts = env_->getArm1OrderParts(); //  remove from tray, displace, bin to tray p1(mp X -> ep)
    auto agv2_OrderParts = env_->getArm2OrderParts(); // p1(sp -> mp)

    /// Checking if anything needs to be added to arm2 to support arm1

    std::map<std::string, std::vector<OrderPart *>> new_shipment_to_agv2;

    for (auto ship_it = agv1_OrderParts->begin(); ship_it != agv1_OrderParts->end(); ++ship_it) {
        
        auto part_type = (*ship_it).first;

        for (auto ord_it = (*ship_it).second.begin(); ord_it != (*ship_it).second.end(); ++ord_it) {

            if ((*ord_it)->getCurrentPose().position.y < -1.5)
            { // order part is not reachable To-DO ----- make sure value is right
                // add a copy of this part to agv2 order parts
                OrderPart* part = new OrderPart();
                part->setPartType((*ord_it)->getPartType());
                part->setCurrentPose((*ord_it)->getCurrentPose());
                // TO-DO ----> set common bin pose so that arm1 can pick it from there.
                // add it to the beginning of agv2 as a new shipment
                if( common_pose_ind <= 3) {
                    part->setEndPose(common_pose_[common_pose_ind]);
                    (*ord_it)->setCurrentPose(common_pose_[common_pose_ind]);
                    if(common_pose_ind==3) common_pose_ind=0;
                    else ++common_pose_ind;
                }
                
                if(new_shipment_to_agv2.count(part_type)) {
                    new_shipment_to_agv2[part_type].emplace_back(part);
                }
                else {
                    new_shipment_to_agv2[part_type] = std::vector<OrderPart*>({part});
                }
            }
        }
    }

    if(new_shipment_to_agv2.size()) {
        env_->getArm2PreOrderParts()->clear();
        env_->getArm2PreOrderParts()->emplace_back(new_shipment_to_agv2);
    }


    /// Checking if anything needs to be added to arm1 to support arm2
    std::map<std::string, std::vector<OrderPart *>> new_shipment_to_agv1;

    for (auto ship_it = agv2_OrderParts->begin(); ship_it != agv2_OrderParts->end(); ++ship_it)
    {

        auto part_type = ship_it->first;

        for (auto ord_it = ship_it->second.begin(); ord_it != ship_it->second.end(); ++ord_it)
        {

            if ((*ord_it)->getCurrentPose().position.y < 0.5) { // order part is not reachable  To-DO ----- make sure value is right
                // add a copy of this part to agv2 order parts
                OrderPart *part = new OrderPart();
                part->setPartType((*ord_it)->getPartType());
                part->setCurrentPose((*ord_it)->getCurrentPose());
                // TO-DO ---- set ommon bin pose so that arm1 can pick it from there.
                // add it to the beginning of agv2 as a new shipment
                if( common_pose_ind <= 3) {
                    part->setEndPose(common_pose_[common_pose_ind]);
                    (*ord_it)->setCurrentPose(common_pose_[common_pose_ind]);
                    if(common_pose_ind==3) common_pose_ind=0;
                    else ++common_pose_ind;
                }
                if (new_shipment_to_agv1[part_type].count()) {
                    new_shipment_to_agv1[part_type].emplace_back(part);
                }
                else {
                    new_shipment_to_agv1[part_type] = std::vector<OrderPart *>({part});
                }
            }
        }
    }

    if (new_shipment_to_agv1.size())
    {
        env_->getArm1PreOrderParts()->clear();
        env_->getArm1PreOrderParts()->emplace_back(new_shipment_to_agv1);
    }

        // for (const auto &part : agv1_OrderParts)
        // {
        //     auto part_type = part->first;
        //     auto vec_poses = (*sorted_BinParts)[part_type];
        //     for (auto pose : vec_poses)
        // for (const auto &part : agv1_OrderParts)
        // {
        //     auto part_type = part->first;
        //     auto vec_poses = (*sorted_BinParts)[part_type];
        //     for (auto pose : vec_poses)
        //     {
        //         if (pose.position.y < -1.5)
        //             {
        //                 arm2_Vector = part->second;
        //             }
        //         else
        //         {
        //             arm1_Vector = part->second;
        //         }
        //     }
        // }
    //     for (const auto &part : agv2_OrderParts)
    //     {
    //         auto part_type = part->first;
    //     auto vec_poses = (*sorted_BinParts)part_type];
    //     for (const auto &part : agv1_OrderParts)
    //     {
    //         if (pose.position.y > 1.5)
    //             {
    //                 arm1_Vector = part->second;
    //             }
    //         else
    //         {
    //             arm2_Vector = part->second;
    //         }
//     //     }
//     // }
//     // target();
//     }
// 
// void Planner::target(){
//     for (auto part : arm1_Vector)
//     {
//         if(part.getTrayId() == "agv1")
//         {
//             arm1_.moveToTarget(part.getTrayPose());
        } else
//         {
//             arm1_.moveToTarget(common_pose_);
//         }
//     }
// 
//     for (auto part : arm2_Vector)
//     {
//         if (part.getTrayId() == "agv2")
//         {
//             arm2_.moveToTarget(part.getTrayPose());
//         }
//         else
        {
            arm2_.moveToTarget(common_pose_);
        }
    }
}
        {
            arm2_.moveToTarget(common_pose_);
        }
    }
}
        {
            arm2_.moveToTarget(common_pose_);
        }
    }
}
        {
            arm2_.moveToTarget(common_pose_);
        }
    }
}
        {
            arm2_.moveToTarget(common_pose_);
        }
    }
}
        {
            arm2_.moveToTarget(common_pose_);
        }
    }
}
        {
            arm2_.moveToTarget(common_pose_);
        }
    }
}
        {
            arm2_.moveToTarget(common_pose_);
        }
    }
}
        {
            arm2_.moveToTarget(common_pose_);
        }
    }
}
        {
            arm2_.moveToTarget(common_pose_);
        }
    }
}
        {
            arm2_.moveToTarget(common_pose_);
        }
    }
}
        {
            arm2_.moveToTarget(common_pose_);
        }
    }
}
        {
            arm2_.moveToTarget(common_pose_);
        }
    }
}
        {
            arm2_.moveToTarget(common_pose_);
        }
    }
}
        {
            arm2_.moveToTarget(common_pose_);
        }
    }
}

    

    // find Unreachable Parts for agv1 from the agv1 order

    // push these parts to the vector of the order parts of arm2

    // execute to deliver these parts to the common location

    // find Unreachable Parts for agv2 from the agv2 order

    // push these parts to the vector of the order parts of arm1

    // execute to deliver these parts to the common location
    // logic for execute :
    // iterate thorugh arm1_Vector
    // if (part.agv id == agv 1)
    //  goToTarget(end_Pose)
    // if( part.agv id == agv 2)
    // goToTarget(common_pose)

    //    iterate through arm2_Vector
    //    if(part.agv id == agv 2)
    //     goToTarget(end_Pose)
    //    if(part.agv id == agv 1)
    //     goToTarget(common_Pose)
