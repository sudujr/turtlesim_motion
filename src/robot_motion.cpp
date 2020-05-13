#include "robot_motion.h"

int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "turtlesim_motion");
	ros::NodeHandle n;
	motion m(n);
	
	
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;
	double rotspeed, linear_speed;

	/** test your code here **/
	/*ROS_INFO("\n\n\n******START TESTING************\n");
	cout<<"enter speed: ";
	cin>>speed;
	cout<<"enter distance: ";
	cin>>distance;
	cout<<"forward?: ";
	cin>>isForward;
	m.move(speed, distance, isForward);

	cout<<"enter angular velocity (degree/sec): ";
	cin>>angular_speed;
	cout<<"enter desired angle (degrees): ";
	cin>>angle;
	cout<<"clockwise ?: ";
	cin>>clockwise;
	m.rotate(m.degrees2radians(angular_speed), m.degrees2radians(angle), clockwise);*/
	 
	/** set the robot to desired orientation **/
	/*m.setDesiredOrientation(m.degrees2radians(120));
	ros::Rate loop_rate(0.5);
	loop_rate.sleep();
	m.setDesiredOrientation(m.degrees2radians(-60));
	loop_rate.sleep();
	m.setDesiredOrientation(m.degrees2radians(0));*/


	/** move the turtlesim to desired position **/
	turtlesim::Pose goal_pose;
	goal_pose.x=1;
	goal_pose.y=1;
	goal_pose.theta=0;
	m.moveGoal(goal_pose, 0.01);

	//gridClean();
	

	cout << "Enter the speed for rotation :" << " ";
	cin >> rotspeed;
	cout << "Enter the speed for motion :" << " ";
	cin >> linear_speed;

	m.spiralmotion(rotspeed,linear_speed);



    ros::spin();

    return 0;
}

motion::motion(ros::NodeHandle &n){
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, &motion::poseCallback,this);
}
void motion::publishVelocity(geometry_msgs::Twist vel_msg){
	velocity_publisher.publish(vel_msg);
}

void motion::move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x =abs(speed);
	else
		vel_msg.linear.x =-abs(speed);
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do{
		//velocity_publisher.publish(vel_msg);
		publishVelocity(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);
	vel_msg.linear.x =0;
	publishVelocity(vel_msg);

}


void motion::rotate (double angular_speed, double relative_angle, bool clockwise){

	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (clockwise)
		vel_msg.angular.z =-abs(angular_speed);
	else
		vel_msg.angular.z =abs(angular_speed);

	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(10);
	do{
		publishVelocity(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle<relative_angle);

	vel_msg.angular.z =0;
	publishVelocity(vel_msg);

}

double motion::degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}


void motion::setDesiredOrientation (double desired_angle_radians){
	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	bool clockwise = ((relative_angle_radians<0)?true:false);
	//cout<<desired_angle_radians <<","<<turtlesim_pose.theta<<","<<relative_angle_radians<<","<<clockwise<<endl;
	rotate (degrees2radians(10), abs(relative_angle_radians), clockwise);

}

void motion::poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}

double motion::getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

void motion::moveGoal(turtlesim::Pose  goal_pose, double distance_tolerance){

	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(100);
	double E = 0.0;
	do{
		/****** Proportional Controller ******/
		//linear velocity in the x-axis
		double Kp=1.0;
		double Ki=0.02;
		//double v0 = 2.0;
		//double alpha = 0.5;
		double e = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		double E = E+e;
		//Kp = v0 * (exp(-alpha)*error*error)/(error*error);
		vel_msg.linear.x = (Kp*e);
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		//angular velocity in the z-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =4*(atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x-turtlesim_pose.x)-turtlesim_pose.theta);

		publishVelocity(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerance);
	cout<<"end move goal"<<endl;
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	publishVelocity(vel_msg);
}

void motion::spiralmotion(double &constant_speed, double &linear_speed){
	geometry_msgs::Twist vel_msg;
	
	ros::Rate loop(1);

	do{
		linear_speed=linear_speed+1.0;
		vel_msg.linear.x =linear_speed;
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		//set a random angular velocity in the y-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =constant_speed;//((vk)/(0.5+rk));

		cout<<"vel_msg.linear.x = "<<vel_msg.linear.x<<endl;
		cout<<"vel_msg.angular.z = "<<vel_msg.angular.z<<endl;
		publishVelocity(vel_msg);
		ros::spinOnce();

		loop.sleep();

	}while((turtlesim_pose.x<9.5)&&(turtlesim_pose.y<9.5));
	vel_msg.linear.x =0;
	publishVelocity(vel_msg);
}


