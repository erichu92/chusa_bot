#include <motor_controller/step_motor.h>
#include <math.h>

Step_Motor::Step_Motor(){
	bool init_result = init();
	ROS_ASSERT(init_result);
}

Step_Motor::~Step_Motor(){

}

bool Step_Motor::init(){
	publisher_ = nh_.advertise<chusa_msgs::Step>("step_motor", 1);
	subscriber_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &Step_Motor::msgCallback, this);
	return true;
}

void Step_Motor::msgCallback(const geometry_msgs::Twist msg_in){
	
	double v_l = msg_in.linear.x - 0.5 * ROBOT_WIDTH * msg_in.angular.z;
	double v_r = msg_in.linear.x + 0.5 * ROBOT_WIDTH * msg_in.angular.z;

	if(v_l < 0){
		this->msg.left_direction = false;
		if(v_l < -MAX_VEL){
			v_l = -MAX_VEL;
		}
	}
	else{
		this->msg.left_direction = true;
		if(v_l > MAX_VEL){
			v_l = MAX_VEL;
		}
	}

	if(v_r < 0){
		this->msg.right_direction = false;
		if(v_r < -MAX_VEL){
			v_r = -MAX_VEL;
		}
	}
	else{
		this->msg.right_direction = true;
		if(v_r > MAX_VEL){
			v_r = MAX_VEL;
		}
	}
	
	this->msg.left_tick = floor(v_l * 100);
	this->msg.right_tick = floor(v_r * 100);

	publisher_.publish(this->msg);
	this->msg.left_tick = 0;
	this->msg.right_tick = 0;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "motor_controller_node");
	Step_Motor step_motor_controller;
	
	ros::Rate r(10);
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}