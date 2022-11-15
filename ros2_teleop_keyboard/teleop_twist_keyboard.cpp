#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <roscpp>


#include <sstream>
#include <termios.h>
#include <select.h>
#include<tty.h>
#include <tuple>
#include <string>

int getKey(){
    tty.setraw(sys.stdin.fileno());
	select.select([sys.stdin], [], [], 0)  ; 
	string key = sys.stdin.read(1);
	termios.tcsetattr(system.stdin, termios.TCSADRAIN, settings);
	return key;
}

std::tuple<float, float> vels(speed,turn){
	return {speed,turn},
}

int main(int argc, char **argv){

    settings = termios.tcgetattr(system.stdin);

    msg = /*
        Reading from the keyboard  and Publishing to Twist!
        ---------------------------
        Moving around:
           u    i    o
           j    k    l
           m    ,    .

        For Holonomic mode (strafing), hold down the shift key:
        ---------------------------
           U    I    O
           J    K    L
           M    <    >

        t : up (+z)
        b : down (-z)

        anything else : stop

        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%

        CTRL-C to quit
    */

    map<string, int[]> moveBindings = {
        'i',[1,0,0,0],
		'o',[1,0,0,-1],
		'j',[0,0,0,1],
		'l',[0,0,0,-1],
		'u',[1,0,0,1],
		',',[-1,0,0,0],
		'.',[-1,0,0,1],
		'm',[-1,0,0,-],
		'O',[1,-1,0,0],
		'I',[1,0,0,0],
		'J',[0,1,0,0],
		'L',[0,-1,0,0],
		'U',[1,1,0,0],
		'<',[-1,0,0,0],
		'>',[-1,-1,0,],
		'M',[-1,1,0,0],
		't',[0,0,1,0}],
		'b',[0,0,-1,0]};

    map<string, tuple<int>> speedBindings ={
		'q':[1.1,1.1],
		'z':[.9,.9],
		'w':[1.1,1],
		'x':[.9,1],
		'e':[1,1.1],
		'c':[1,.9]};

    ros::init(argc, argv, "teleop_twist_keyboard");
    ros::NodeHandle n;

    pub = ros::Publisher qos_profile_default = n.advertise<std_msgs::Twist>("cmd_vel");

    float speed = 0.5;
	float turn = 1.0;
	int x = 0;
	int y = 0;
	int z = 0;
	float th = 0;
	int status = 0;

    try{
        cout<<msg;
        cout<<vels(speed,turn);
        while(true){
            string key = getKey();
            if(moveBindings.contains(key)){
                x = moveBindings[key][0];
				y = moveBindings[key][1];
				z = moveBindings[key][2];
				th = moveBindings[key][3];
            }
            else if(speedBindingsBindings.contains(key)){
                speed = speed * speedBindings[key][0];
				turn = turn * speedBindings[key][1];

                cout<<vels(speed,turn);
                if (status == 14){
					cout<<msg;
                }    
				status = (status + 1) % 15;
            }
            else{
                int x = 0;
				int y = 0;
				int z = 0;
				float th = 0;
				if (key == '\x03'){
					break;
                }
            }

            twist = Twist();
			twist.linear.x = x*speed; 
            twist.linear.y = y*speed; 
            twist.linear.z = z*speed;
			twist.angular.x = 0.0; 
            twist.angular.y = 0.0; 
            twist.angular.z = th*turn;
			pub.publish(twist);
        }
    }
    catch(){
        cout<<e;
    }
    
    twist = Twist()
	twist.linear.x = 0.0; 
    twist.linear.y = 0.0; 
    twist.linear.z = 0.0;
	twist.angular.x = 0.0; 
    twist.angular.y = 0.0; 
    twist.angular.z = 0.0;
	pub.publish(twist);

	termios.tcsetattr(system.stdin, termios.TCSADRAIN, settings);
}