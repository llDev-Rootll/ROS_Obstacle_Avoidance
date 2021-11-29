/**
 * Copyright (c) 2021 Arunava Basu
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef INCLUDE_OA_H_
#define INCLUDE_OA_H_

#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"


class Listener {
 public:
    /**
    * @brief Constructor
    * 
    * @param n : node handle
    */
     explicit Listener(ros::NodeHandle n);
     /**
      * @brief keeps track of number of turns the robot has turned
      * 
      */
     int count;
 private:
     /**
      * @brief ros node handle
      * 
      */
     ros::NodeHandle node_h;
     /**
      * @brief ros velocity publisher object
      * 
      */
     ros::Publisher vel_pub;
     /**
      * @brief ros subscriber object
      * 
      */
     ros::Subscriber sub;
     /**
      * @brief laser scan callback
      * 
      * @param msg : laser scan message
      */
     void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
     float threshold;
};
#endif  // OA_H_
