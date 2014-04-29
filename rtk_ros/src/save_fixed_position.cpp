/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
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
* Author: Goncalo Cabrita on 21/04/2014
*********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <angles/angles.h>

#include <rtk_ros/UTMConverter.h>

#define MAX_MEDIAN_VECTOR_SIZE  4048

std::vector<double> lat_median_vector;
std::vector<double> long_median_vector;
std::vector<double> alt_median_vector;

double median(std::vector<double> *values, double new_value=0.0)
{
  double median;
  size_t size = values->size();

  if(size < MAX_MEDIAN_VECTOR_SIZE)
  {
      values->push_back(new_value);
      if(values->size() == 1) return new_value;
  }
  else
  {
      std::sort(values->begin(), values->end());
      if(new_value > *values->begin() && new_value < *values->end())
      {
          if(size % 2 == 0)
          {
              median = (values->at(size / 2 - 1) + values->at(size / 2)) / 2;
          }
          else
          {
              median = values->at(size / 2);
          }

          if(new_value < median)
          {
              values->erase(values->begin());
          }
          else if(new_value > median)
          {
              values->erase(values->end());
          }
          else
          {
              values->erase(values->begin());
              values->erase(values->end());
          }
          values->push_back(new_value);
      }
  }

  std::sort(values->begin(), values->end());
  if(size % 2 == 0)
  {
      median = (values->at(size / 2 - 1) + values->at(size / 2)) / 2;
  }
  else
  {
      median = values->at(size / 2);
  }

  return median;
}

void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    median(&lat_median_vector, msg->latitude);
    median(&long_median_vector, msg->longitude);
    median(&alt_median_vector, msg->altitude);
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "save_fixed_position");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    ros::Subscriber sub = n.subscribe("gps/fix", 50, fixCallback);

    double duration;
    pn.param("duration", duration, 60.0);

    ros::Time start_time = ros::Time::now();

    ros::Rate r(10.0);
    while(ros::ok() && ros::Time::now() - start_time < ros::Duration())
    {
        r.sleep();
        ros::spinOnce();
    }

    sensor_msgs::NavSatFix fix;
    fix.latitude = median(&lat_median_vector);
    fix.longitude = median(&long_median_vector);
    fix.altitude = median(&alt_median_vector);

    UTMCoordinates utm;
    UTMConverter::latitudeAndLongitudeToUTMCoordinates(fix, utm);

    printf("fix:\n");
    printf("   latitude: %lf\n", fix.latitude);
    printf("   longitude: %lf\n", fix.longitude);
    printf("   altitude: %lf\n", fix.altitude);
    printf("\n");
    printf("utm:\n");
    printf("   easting: %lf\n", utm.easting);
    printf("   northing: %lf\n", utm.northing);
    printf("   altitude: %lf\n", fix.altitude);

    return 0;
}
