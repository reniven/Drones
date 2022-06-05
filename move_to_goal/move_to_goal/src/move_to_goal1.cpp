#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <ball_detector/ballLocation.h>
#include <unistd.h>
#include <cstdlib>

mavros_msgs::State current_state;
ball_detector::ballLocation current_location;
geometry_msgs::PoseStamped pose;
int should_move_to_goal = 1;
int should_defend_goal = 0;
int pos_img_thresh = 10;
int neg_img_thresh = -10;
int radius = 0;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
	//std::cout << *msg << std::endl;
}

void get_location(const ball_detector::ballLocation::ConstPtr& msg){
	current_location = *msg;
	//std::cout << *msg << std::endl;
}

void get_location1(const ball_detector::ballLocation::ConstPtr& msg){
        std::cout << "hello" << std::endl;
	std::cout << *msg << std::endl;
}

void move_to_goal(ros::Publisher local_pos_pub){
//	std::cout << pose << std::endl;
	if (current_location.x > pos_img_thresh) {
		pose.pose.position.y += -0.01;	
	}
	if (current_location.x < neg_img_thresh) {
		pose.pose.position.y += 0.01;	
	}
	if (current_location.y > pos_img_thresh) {
		pose.pose.position.z += 0.01;	
	}
	if (current_location.y < neg_img_thresh) {
		pose.pose.position.z += -0.01;	
	}
	if (current_location.radius < (current_location.imageHeight / 2) - 20) {
		pose.pose.position.x += 0.01;		
	}
	if (current_location.radius > (current_location.imageHeight / 2) - 10) {
		pose.pose.position.x += -0.01;		
	}
	if (current_location.x < pos_img_thresh && current_location.x > neg_img_thresh && current_location.y < pos_img_thresh && current_location.y > neg_img_thresh && current_location.radius > (current_location.imageHeight / 2) - 20) {
		pose.pose.orientation.z = 3.1415;		
		local_pos_pub.publish(pose);
		//ros::Duration(5).sleep();
		sleep(2);
		should_move_to_goal = 0;	
		should_defend_goal = 1;
	}
	ros::spinOnce();

}

void defend_goal(ros::Publisher local_pos_pub){

	if (current_location.x > pos_img_thresh) {
		std::cout << "hello1" << std::endl;
                pose.pose.position.y += 0.01;
        }
        if (current_location.x < neg_img_thresh) {
		std::cout << "hello2" << std::endl;
                pose.pose.position.y += -0.01;
        }
        if (current_location.y > pos_img_thresh) {
		std::cout << "hello3" << std::endl;
                pose.pose.position.z += 0.01;
        }
        if (current_location.y < neg_img_thresh) {
		std::cout << "hello4" << std::endl;
                pose.pose.position.z += -0.01;
        }
	if(radius > current_location.radius){
		std::cout << "hello5" << std::endl;
		pose.pose.orientation.z = 0;
		local_pos_pub.publish(pose);
		//ros::Duration(5).sleep();
		sleep(2);
                should_move_to_goal = 1;
                should_defend_goal = 0;
	}
	ros::spinOnce();


}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    // Added Ball Subscriber
    ros::Subscriber ball_location_sum = nh.subscribe<ball_detector::ballLocation>("/yellowGoal", 1, get_location);
    //ros::Subscriber ball_location_yellow = nh.subscribe<ball_detector::ballLocation>("/yelloGoal", 1, get_location1);



    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
	

    int changeRad = 0; 
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;

	//send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

	while(ros::ok()){
 
		if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
			if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
				ROS_INFO("Offboard enabled");
			}
		last_request = ros::Time::now();
		} else {
			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
				if( arming_client.call(arm_cmd) && arm_cmd.response.success){
					ROS_INFO("Vehicle armed");
					//send a few setpoints before starting
					for(int i = 100; ros::ok() && i > 0; --i){
						local_pos_pub.publish(pose);
						ros::spinOnce();
						rate.sleep();
				 	}
				}
				last_request = ros::Time::now();
			}
        }
	if (should_move_to_goal) {
		move_to_goal(local_pos_pub);
	}
	
	if(should_defend_goal) {
		std::cout << "hello" << std::endl;
		if(changeRad == 0){
			radius = current_location.radius;
		}
		defend_goal(local_pos_pub);
		if(changeRad == 2){
			changeRad = 0;
		}
		else{
			changeRad++;
		}
	}
	local_pos_pub.publish(pose);	
        rate.sleep();
	}
	return 0;
}

