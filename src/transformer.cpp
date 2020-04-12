#include <string>
#include <sstream>
#include <transformer.h>


Transformer::Transformer(const std::string & topic): topic_(topic),tfListener(tf_buffer){
    getcameraname()
    child_ = parent_ + "_child";
}

Transformer::~Transformer(){}

void Transformer::getCameraName() {
    stringstream ss(topic_);
    const char delim = '/';
    std::string p_;
    while (std::getline(ss, p_, delim)) {	
	}
	parent_ = p_;
}


void Transformer::setParentPose(const geometry_msgs::Pose & sensor_pose) {
    auto current_time = ros::Time::now();

	transformStamped1.header.stamp = current_time;
	transformStamped1.header.frame_id = "world";
	transformStamped1.child_frame_id = parent;

	transformStamped2.header.stamp = current_time;
	transformStamped2.header.frame_id = parent;
	transformStamped2.child_frame_id = child;

	setPose(sensor_pose,transformStamped1);

    br_w_s.sendTransform(transformStamped1);
	ros::Duration(0.01).sleep();
}

void Transformer::setChildPose(const geometry_msgs::Pose & child_pose) {
    setPose(child_pose,transformStamped2);
    br_s_c.sendTransform(transformStamped2);
	ros::Duration(0.01).sleep();

}

void Transformer::setWorldTransform(){

	try{
		transformStamped3 = tfBuffer.lookupTransform("world", child,
				ros::Time(0));
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("exception");
		ROS_WARN("%s",ex.what());
		ros::Duration(0.01).sleep();
	}
}

geometry_msgs::Pose Transformer::getChildWorldPose(){
	geometry_msgs::Pose pose;
	setPose(transformStamped3, pose);
	return pose;
}


void Transformer::setPose(const geometry_msgs::Pose pose, geometry_msgs::TransformStamped &transformStamped ){
	transformStamped.transform.translation.x = pose.position.x;
	transformStamped.transform.translation.y = pose.position.y;
	transformStamped.transform.translation.z = pose.position.z;
	transformStamped.transform.rotation.x = pose.orientation.x;
	transformStamped.transform.rotation.y = pose.orientation.y;
	transformStamped.transform.rotation.z = pose.orientation.z;
	transformStamped.transform.rotation.w = pose.orientation.w;
}

void Transformer::setPose(geometry_msgs::TransformStamped * transformStamped, geometry_msgs::Pose &pose) {
	pose.position.x = transformStamped->transform.translation.x;
	pose.position.y = transformStamped->transform.translation.y;
	pose.position.z = transformStamped->transform.translation.z;
	pose.orientation.x = transformStamped->transform.rotation.x;
	pose.orientation.y = transformStamped->transform.rotation.y;
	pose.orientation.z = transformStamped->transform.rotation.z;
	pose.orientation.w = transformStamped->transform.rotation.w;
}
    
    