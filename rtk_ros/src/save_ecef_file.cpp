/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Goncalo Cabrita and Jorge Fraga on 14/06/2013
*********************************************************************/

#include <ros/ros.h>
#include <rtk_msgs/ECEFCoordinates.h>

#define TIMEOUT 10.0

rtk_msgs::ECEFCoordinates ecef_msg;

void callback(const rtk_msgs::ECEFCoordinates::ConstPtr& msg)
{
    ecef_msg = *msg;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "save_ecef_file");

	ros::NodeHandle n;

    std::string file_path = "ecef_base_station.yaml";

    ROS_INFO("Opening %s...", file_path.c_str());
    FILE * yaml_file;
    yaml_file = fopen(file_path.c_str() , "w");
    if(yaml_file == NULL)
    {
        ROS_FATAL("Error opnening %s!", file_path.c_str());
        ROS_BREAK();
    }
    ROS_INFO("Done.");

    ROS_INFO("Subscribing to ECEF topic...");
    ros::Subscriber sub = n.subscribe("base_station/gps/ecef", 10, callback);
    ROS_INFO("Done.");

    ros::Time start_time = ros::Time::now();
    bool waiting_for_message = true;

    ROS_INFO("Waiting for ECEF message...");
    ros::Rate r(5.0);
    while(ros::ok() && waiting_for_message)
	{
        if(ros::Time::now() - ecef_msg.header.stamp < ros::Duration(TIMEOUT))
        {
            fprintf(yaml_file, "base_position:\n  x: %lf\n  y: %lf\n  z: %lf\n\n  covariance: [", ecef_msg.position.x, ecef_msg.position.y, ecef_msg.position.z);
            for(int i=0 ; i<ecef_msg.position_covariance.size() ; i++)
            {
                fprintf(yaml_file, "%lf%c", ecef_msg.position_covariance[i], (i==ecef_msg.position_covariance.size()-1 ? ']' : ','));
            }
            waiting_for_message = false;
        }

        if(ros::Time::now() - start_time > ros::Duration(TIMEOUT))
        {
            ROS_FATAL("Timeout waiting for ECEF message. Are you sure the base station node is online?");
            ROS_BREAK();
        }

        ros::spinOnce();
        r.sleep();
	}				

    fclose(yaml_file);
    ROS_INFO("File saved. Goodbye!");

	return 0;
}

// EOF

