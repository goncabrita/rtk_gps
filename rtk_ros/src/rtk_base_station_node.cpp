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
#include <cereal_port/CerealPort.h>
#include <std_msgs/ByteMultiArray.h>
#include <cstdlib>
#include <zeromq_cpp/zmq.hpp>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "rtk_base_station");

    ROS_INFO("RTKlib for ROS Base Station Edition");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    /*get ROS_IP or ROS_HOSTNAME*/
    char* basestationaddress;
    basestationaddress = getenv ("ROS_HOSTNAME");
    if (basestationaddress==NULL){
        basestationaddress = getenv ("ROS_IP");

        if (basestationaddress==NULL){
            ROS_FATAL("RTK -- Failed to get basestation hostname/ip !!!");
            ROS_BREAK();
        }
    }

    ROS_INFO("RTK -- Basestation Hostname/IP: %s" ,basestationaddress) ;

    /*node parameters*/
    std::string port;
    int socket_port; // zmq socket port
    int baudrate;
    pn.param<std::string>("port", port, "/dev/ttyACM0");
    pn.param<int>("socket_port", socket_port, 45000);
    pn.param("baudrate", baudrate, 115200);

    std::stringstream tempstream;
    tempstream << "tcp://" << basestationaddress<<":"<<socket_port;
    n.setParam("/basestation_address", tempstream.str());


    /*zeromq*/
    zmq::context_t zContext(1);
    zmq::socket_t zBroadcastQueue(zContext, ZMQ_PUB);
    zBroadcastQueue.bind(tempstream.str().c_str());

    cereal::CerealPort serial_gps;

    try{ serial_gps.open(port.c_str(), baudrate); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("RTK -- Failed to open the serial port!!!");
        ROS_BREAK();
    }

    char buffer[350];
    int count;

    buffer[108]=0;
    buffer[0]='l';
    buffer[1]='s';
    buffer[2]='e';
    buffer[3]=1;



    //ros::Publisher pub = n.advertise<std_msgs::ByteMultiArray>("base_station/gps/raw_data", 100);

    ROS_INFO("RTK -- Streaming data...");

    while(ros::ok())
    {
        try{
         count = serial_gps.read(buffer, 300, 200); }
        catch(cereal::TimeoutException& e)
        {
            //ROS_WARN("RTK -- Failed to get data from GPS!");
        }

        /*std_msgs::ByteMultiArray raw_msg;

        for(int i=0 ; i<count ; i++)
        {
            raw_msg.data.push_back(buffer[i]);
        }

        pub.publish(raw_msg);*/

        zmq::message_t data((void*)buffer,count,NULL,NULL); //zero copy
        zBroadcastQueue.send(data);

        ros::Duration(0.1).sleep();
    }

    return 0;
}

// EOF

