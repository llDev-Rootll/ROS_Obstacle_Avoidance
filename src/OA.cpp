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
#include <vector>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "OA.h"

/**
 * @brief Construct a new Listener:: Listener object 
 *and initialize the global nodel handle, publisher and 
 *subcriber objects. Also initialize the count and distance threshold
 * 
 * @param n = node handle 
 */
Listener::Listener(ros::NodeHandle n) {
  node_h = n;
  vel_pub = node_h.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  sub = node_h.subscribe("scan", 100, &Listener::scanCallback, this);
  ros::spinOnce();
  count = 0;
  threshold = 0.8;
}



/**
 * @brief A callback function to publish corresponding Twist messages according to distance
 * 
 * @param received laser scan object
 */
void Listener::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  std::vector<float> laser_scan = msg->ranges;
  geometry_msgs::Twist cmd_velocity;
  int range = 18;
  for ( int i = 0; i <= range; i++ ) {
      if ( laser_scan[i] >= threshold ||
      laser_scan[359-i] >= threshold ) {
        threshold = 0.8;
        cmd_velocity.linear.x = 0.2;
        cmd_velocity.angular.z = 0;
        vel_pub.publish(cmd_velocity);
        count = 0;
      } else {
        if ( count > 100 ) {
          cmd_velocity.linear.x = 0;
          cmd_velocity.angular.z = -0.8;
          vel_pub.publish(cmd_velocity);
          threshold = 2.5;
        } else {
          cmd_velocity.linear.x = 0;
          cmd_velocity.angular.z = 0.8;
          vel_pub.publish(cmd_velocity);
          count = count + 1;
        }
      }
  }

  std::cout << "Command Velocity:" << cmd_velocity << std::endl;
}
/**
 * @brief main function which creates a node handles 
 * 
 * @return int 
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  Listener node_handle(n);
  ros::spin();

  return 0;
}
