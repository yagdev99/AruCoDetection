#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <conio.h>
#include <iostream>

using namespace std;

void checkPub(ros::Publisher & pub){
    int c = 0;
    geometry_msgs::Twist msg;


    switch(c = getch()) {
        case 65:
            cout << "Up" << endl;//key up
            msg.linear.x  = 0.3;
            break;

        case 66:
            cout << "Down" << endl;   // key down
            msg.linear.x  = -0.3;
            break;

        case 67:
            cout << "Right" << endl;  // key right
            msg.angular.z = -0.5;
            break;

        case 68:
            cout << "Left" << endl;  // key left
            msg.angular.z = 0.5;
            break;

        case 32:
            cout << "Stopping" << endl;  // key Space bar
            msg.linear.x  = 0;
            msg.angular.z = 0;
            break;

        // case 27:
        //     cout << "ESC" << endl;  // key ESC
        //     ros::shutdown();
        //     break;

        // default:
        //     cout << "Invalid Keypress" << endl; 
        //     break;
    }

    pub.publish(msg);

    

}

int main(int argc, char ** argv){

    ros::init(argc,argv,"turtleControl");

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    
    while(ros::ok()){
        checkPub(pub);

        ros::spinOnce();
    }


}

