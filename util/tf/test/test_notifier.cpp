/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Josh Faust */

#include <tf/message_notifier.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/PositionStamped.h>
#include <boost/bind.hpp>

#include <gtest/gtest.h>

using namespace tf;

ros::node* g_node = NULL;
TransformListener* g_tf = NULL;
TransformBroadcaster* g_broadcaster = NULL;

class Notification
{
public:
	Notification()
	: count_(0)
	{

	}

	void notify(const MessageNotifier<std_msgs::PositionStamped>::MessagePtr& message)
	{
		++count_;
	}

	int count_;
};

template<class T>
class Counter
{
public:
	Counter(const std::string& topic)
	: count_(0)
	, topic_(topic)
	{
		g_node->subscribe(topic_, message_, &Counter::callback, this, 0);
	}

	~Counter()
	{
		g_node->unsubscribe(topic_, &Counter::callback, this);
	}

	void callback()
	{
		++count_;
	}

	T message_;

	int count_;
	std::string topic_;
};

TEST(MessageNotifier, noTransforms)
{
	Notification n;
	Counter<std_msgs::PositionStamped> c("test_message");
	MessageNotifier<std_msgs::PositionStamped>* notifier = new MessageNotifier<std_msgs::PositionStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n, _1), "test_message", "frame1", 1);

	ros::Duration(0.1).sleep();

	std_msgs::PositionStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "frame2";
	g_node->publish("test_message", msg);

	while (c.count_ < 1)
	{
		ros::Duration(0.01).sleep();
	}

	EXPECT_EQ(n.count_, 0);

	delete notifier;
}

TEST(MessageNotifier, preexistingTransforms)
{
	Notification n;
	Counter<std_msgs::PositionStamped> c("test_message");
	Counter<rosTF::TransformArray> c2("TransformArray"); /// \todo Switch this to tf_message once rosTF goes away completely
	MessageNotifier<std_msgs::PositionStamped>* notifier = new MessageNotifier<std_msgs::PositionStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n, _1), "test_message", "frame1", 1);

	ros::Duration(0.1).sleep();

	ros::Time stamp = ros::Time::now();

	g_broadcaster->sendTransform(btTransform(btQuaternion(0,0,0), btVector3(1,2,3)), stamp, "frame1", "frame2");

	while (c2.count_ < 1)
	{
		ros::Duration(0.01).sleep();
	}

	std_msgs::PositionStamped msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = "frame2";
	g_node->publish("test_message", msg);

	while (c.count_ < 1)
	{
		ros::Duration(0.01).sleep();
	}

	ros::Duration(0.1).sleep();

	EXPECT_EQ(n.count_, 1);

	delete notifier;
}

TEST(MessageNotifier, postTransforms)
{
	Notification n;
	Counter<std_msgs::PositionStamped> c("test_message");
	MessageNotifier<std_msgs::PositionStamped>* notifier = new MessageNotifier<std_msgs::PositionStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n, _1), "test_message", "frame3", 1);

	ros::Duration(0.1).sleep();

	std_msgs::PositionStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "frame4";
	g_node->publish("test_message", msg);

	while (c.count_ < 1)
	{
		ros::Duration(0.01).sleep();
	}

	g_broadcaster->sendTransform(btTransform(btQuaternion(0,0,0), btVector3(1,2,3)), msg.header.stamp, "frame3", "frame4");

	ros::Duration(0.1).sleep();

	EXPECT_EQ(n.count_, 1);

	delete notifier;
}

TEST(MessageNotifier, queueSize)
{
	Notification n;
	Counter<std_msgs::PositionStamped> c("test_message");
	MessageNotifier<std_msgs::PositionStamped>* notifier = new MessageNotifier<std_msgs::PositionStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n, _1), "test_message", "frame5", 10);

	ros::Duration(0.1).sleep();

	for (int i = 0; i < 20; ++i)
	{
		std_msgs::PositionStamped msg;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "frame6";
		g_node->publish("test_message", msg);
	}

	while (c.count_ < 20)
	{
		ros::Duration(0.01).sleep();
	}

	g_broadcaster->sendTransform(btTransform(btQuaternion(0,0,0), btVector3(1,2,3)), ros::Time::now(), "frame5", "frame6");

  ros::Duration(0.1).sleep();

	EXPECT_EQ(n.count_, 10);

	delete notifier;
}

TEST(MessageNotifier, setTopic)
{
	Notification n;
	Counter<std_msgs::PositionStamped> c("test_message2");
	MessageNotifier<std_msgs::PositionStamped>* notifier = new MessageNotifier<std_msgs::PositionStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n, _1), "test_message", "frame7", 1);
	notifier->setTopic("test_message2");

	ros::Duration(0.1).sleep();

	ros::Time stamp = ros::Time::now();

	g_broadcaster->sendTransform(btTransform(btQuaternion(0,0,0), btVector3(1,2,3)), stamp, "frame7", "frame8");

	ros::Duration(0.1).sleep();

	std_msgs::PositionStamped msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = "frame8";
	g_node->publish("test_message2", msg);

	while (c.count_ < 1)
	{
		ros::Duration(0.01).sleep();
	}

	ros::Duration(0.1).sleep();

	EXPECT_EQ(n.count_, 1);

	delete notifier;
}

TEST(MessageNotifier, setTargetFrame)
{
	Notification n;
	Counter<std_msgs::PositionStamped> c("test_message");
	MessageNotifier<std_msgs::PositionStamped>* notifier = new MessageNotifier<std_msgs::PositionStamped>(g_tf, g_node, boost::bind(&Notification::notify, &n, _1), "test_message", "frame9", 1);
	notifier->setTargetFrame("frame1000");

	ros::Duration(0.1).sleep();

	ros::Time stamp = ros::Time::now();

	g_broadcaster->sendTransform(btTransform(btQuaternion(0,0,0), btVector3(1,2,3)), stamp, "frame9", "frame10");

	ros::Duration(0.1).sleep();

	std_msgs::PositionStamped msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = "frame10";
	g_node->publish("test_message", msg);

	while (c.count_ < 1)
	{
		ros::Duration(0.01).sleep();
	}

	ros::Duration(0.1).sleep();

	EXPECT_EQ(n.count_, 0);

	delete notifier;
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv);
	g_node = new ros::node("test_notifier");
	g_node->advertise<std_msgs::PositionStamped>("test_message", 0);
	g_node->advertise<std_msgs::PositionStamped>("test_message2", 0);

	g_tf = new TransformListener(*g_node);
	g_broadcaster = new TransformBroadcaster(*g_node);

	int ret = RUN_ALL_TESTS();

	delete g_broadcaster;
	delete g_tf;

	ros::fini();
	delete g_node;

	return ret;
}
