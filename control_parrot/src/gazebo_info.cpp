
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>
#include <sstream>

ros::Publisher pose_pub;

/////////////////////////////////////////////////
// Function is called every time a message is received.
void posesStampedCallback(ConstPosesStampedPtr &posesStamped)
{
  // imprime por pantalla todo en modo debug
  //std::cout << posesStamped->DebugString();

  ::google::protobuf::int32 sec = posesStamped->time().sec();
  ::google::protobuf::int32 nsec = posesStamped->time().nsec();
  // std::cout << "Read time: sec: " << sec << " nsec: " << nsec << std::endl;

    //cojo pose(0) porque sera donde esta guardada pose de mi drone. pose(1)-> posicion del cuerpo, que no da info. Si tuviese otro drone, seria pose(2).
    const ::gazebo::msgs::Pose &pose = posesStamped->pose(0);
    std::string name = pose.name();

      const ::gazebo::msgs::Vector3d &position = pose.position();
      const ::gazebo::msgs::Quaternion &orientation = pose.orientation();

      double x = position.x();
      double y = position.y();
      double z = position.z();
      double tx = orientation.x();
      double ty = orientation.y();
      double tz = orientation.z();
      double tw = orientation.w();

      // std::cout << "Read position: x: " << x
          // << " y: " << y << " z: " << z << std::endl;

      // std::cout << "Read orientation: x: " << tx
          // << " y: " << ty << " z: " << tz << " w: " << tw << std::endl;

      geometry_msgs::PoseStamped msg;
      msg.pose.position.x = x;
      msg.pose.position.y = y;
      msg.pose.position.z = z;
      msg.pose.orientation.x = tx;
      msg.pose.orientation.y = ty;
      msg.pose.orientation.z = tz;
      msg.pose.orientation.w = tw;
      msg.header.stamp.sec = int(sec);
      msg.header.stamp.nsec = int(nsec);

      pose_pub.publish(msg);

}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  //ros node name
  ros::init(_argc, _argv, "gazebo_info");

  ros::NodeHandle n("~");

  std::string num_drone;
  n.param<std::string>("param_num", num_drone, "0");

  std::cout << num_drone << std::endl;

  //std::string num_str = std::to_string(num_drone);

  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/drone_gazebo" + num_drone + "/pose", 1000);



  // Load gazebo
   gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo pose info topic
  gazebo::transport::SubscriberPtr sub =
  node->Subscribe("~/pose/info", posesStampedCallback);

  // Busy wait loop...replace with your own code as needed.
  while (ros::ok()){
    gazebo::common::Time::MSleep(10);

  }
  // Make sure to shut everything down.
  // gazebo::client::shutdown();
  gazebo::shutdown();
}
