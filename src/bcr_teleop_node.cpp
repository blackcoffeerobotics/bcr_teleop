
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

char key(' ');

int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 1;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "bcr_teleop");
    ros::NodeHandle nh;

    // Init cmd_vel publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Create Twist message
    geometry_msgs::Twist twist;
    
    while (true)
    {

        // Get the pressed key
        key = getch();
        std::cout<< +key << std::endl;
        switch (key)
        {
        case 65: // Up arrow
            twist.linear.x += 0.05;
            break;
        case 66: // Down arrow
            twist.linear.x -= 0.05;
            break;
        case 67: // Right arrow
            twist.angular.z += 0.05;
            break;
        case 68: // Left arrow
            twist.angular.z -= 0.05;
            break;
        case 13: //Enter
            twist.linear.x = 0;
            twist.angular.z = 0;
            break;
        default:
            break;
        }
        if (key == '\x03') // if ctrl c was pressed
                  {
            break;
        }
        pub.publish(twist);
                ros::spinOnce();
    }
        twist.linear.x = 0;
        twist.angular.z = 0;
        pub.publish(twist);
    return 0;

}
