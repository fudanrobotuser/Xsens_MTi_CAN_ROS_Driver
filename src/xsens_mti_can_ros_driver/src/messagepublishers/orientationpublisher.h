
//  Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef ORIENTATIONPUBLISHER_H
#define ORIENTATIONPUBLISHER_H

#include "packetcallback.h"
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ros/ros.h>

struct OrientationPublisher : public PacketCallback
{
    ros::Publisher pub;
    ros::Publisher pub2;
    std::string frame_id = DEFAULT_FRAME_ID;


    OrientationPublisher(ros::NodeHandle &node)
    {
        int pub_queue_size = 5;
        ros::param::get("~publisher_queue_size", pub_queue_size);
        pub = node.advertise<geometry_msgs::QuaternionStamped>("filter/quaternion", pub_queue_size);
        pub2 = node.advertise<geometry_msgs::QuaternionStamped>("filter/mashihao", pub_queue_size);
        ros::param::get("~frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, ros::Time timestamp)
    {
        if (packet.containsXsQuaternion)
        {
            geometry_msgs::QuaternionStamped msg;
            geometry_msgs::Vector3Stamped euler_msg;

            msg.header.stamp = timestamp;
            msg.header.frame_id = frame_id;

            XsQuaternion q = packet.quaternion;

            msg.quaternion.w = q.q0;
            msg.quaternion.x = q.q1;
            msg.quaternion.y = q.q2;
            msg.quaternion.z = q.q3;

            // ����Ԫ��ת��Ϊtf2::Quaternion
            tf2::Quaternion tf_quat(q.q1, q.q2, q.q3, q.q0);
            tf2::Matrix3x3 tf_mat(tf_quat);

            // ����Ԫ��ת��Ϊŷ����
            double roll, pitch, yaw;
            tf_mat.getRPY(roll, pitch, yaw);

            // ���ŷ������Ϣ
            euler_msg.vector.x = roll;
            euler_msg.vector.y = pitch;
            euler_msg.vector.z = yaw;

            // ����ŷ������Ϣ
            pub2.publish(euler_msg);

            pub.publish(msg);
            
            // ROS_INFO("Published euler angles: roll=%f, pitch=%f, yaw=%f", roll, pitch, yaw);
            // ROS_INFO("Published euler angles: roll=%f, pitch=%f, yaw=%f", packet.euler.roll, packet.euler.pitch, packet.euler.yaw);
        }
    }
};

#endif
