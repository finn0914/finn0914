////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2023/12 Kingwaytech Inc.
// Auther :Morteza Aliyari
// Email: mortezaliyari@gmail.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////


#include "station_tcp_connection/station_tcp_connection_core.hpp"

RGBDTCPRec::RGBDTCPRec():Node("stationtcpcon_node")
{
  Init();
  RCLCPP_INFO(this->get_logger(),"initialized!");
}

RGBDTCPRec::~RGBDTCPRec(){
}
 /**
     * @Functinality:
        1.read parameters
        2.run a thread to collect  and publish the topics
     * @Inputs:
     * @Outputs:
    */
void RGBDTCPRec::Init(){
    try{
        
    }
    catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }

}
