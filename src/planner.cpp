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

Planner::Planner(Environment * env): env_(env), ordermanager_(env_){};
Planner::~Planner(){};


Planner::plan(){
    std::vector<OrderPart> arm2_Vector;
    std::vector<OrderPart> arm1_Vector;
    auto sorted_BinParts = environment->getSortedBinParts();
    auto agv1_OrderParts = environment->getAgv1OrderParts();
    auto agv2_OrderParts = environment->getAgv2OrderParts();
    for (const auto &part : agv1_OrderParts){
        if(part->first == )  )
        if (part.getTrayPose().position.y)>1.5){
                arm2_Vector.push_back(part);
        } else
        {
            arm1_Vector.push_back(part);
        }
    }
    for (const auto &part : agv2_OrderParts)
    {
        if (part.getTrayPose().position.y)<0.5)
            {
                arm1_Vector.push_back(part);
                arm2_Vector.push_back(part);
            }
        else
        {
            arm2_Vector.push_back(part);
        }
    }
}
// find Unreachable Parts for agv1 from the agv1 order

// push these parts to the vector of the order parts of arm2

// execute to deliver these parts to the common location

// find Unreachable Parts for agv2 from the agv2 order

// push these parts to the vector of the order parts of arm1

// execute to deliver these parts to the common location




