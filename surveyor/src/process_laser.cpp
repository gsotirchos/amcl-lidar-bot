#include "ros/ros.h"
#include "surveyor/DriveToTarget.h"
#include <sensor_msgs/LaserScan.h>


class CommandRobotClient {
  public:
    CommandRobotClient()
      : far_angle_{0},
        current_lin_x_{-1},
        current_ang_z_{-1}
    {
        // Define a client_ service capable of requesting services from command_robot
        client_ = n_.serviceClient<surveyor::DriveToTarget>("/surveyor/command_robot");

        // Subscribe to /scan topic to read the laser measurements inside the process_laser_callback function
        sub1_ = n_.subscribe("/scan", 10, &CommandRobotClient::process_laser_callback, this);
    }

    // This callback method continuously executes and reads the laser scan
    void process_laser_callback(const sensor_msgs::LaserScan lsr) {
        int scans = (lsr.angle_max - lsr.angle_min) / lsr.angle_increment + 1;
        float range_close = 0.2;  // meters
        bool obstacle_found = false;
        far_angle_ = scans / 2;

        // Loop through each scan angle and find the angle of the farthest range
        for (int i = 0; i < scans; i++) {
            if (preprocess_(lsr, lsr.ranges[i]) > range_close
                && preprocess_(lsr, lsr.ranges[i]) > preprocess_(lsr, lsr.ranges[far_angle_])
            ) {
                far_angle_ = i;
            } else if (preprocess_(lsr, lsr.ranges[i]) > lsr.range_min
                && preprocess_(lsr, lsr.ranges[i]) < range_close
            ) {
                obstacle_found = true;
            }
        }

        // Set desired velocities according to angle of farthest range angle, if found
        float lin_x;
        float ang_z;
        float rot;

        if (obstacle_found == false) {
            lin_x = 0.5;
            rot = 5.0;
        } else {
            lin_x = -0.8;
            rot = 10.0;
        }

        switch (far_angle_ / (scans / 3)) {
            case 0:  // right
                ang_z = -1 * rot;
                break;
            case 1:  // center
                ang_z = 0;
                break;
            case 2:  // left
                ang_z = rot;
                break;
        }

        // If new velocities differ from current then call the drive_robot service to set velocities
        if (lin_x != current_lin_x_ || ang_z != current_ang_z_) {
            ROS_INFO("Angle: %d/%d",far_angle_, scans);
            drive_robot_(lin_x, ang_z);
            current_lin_x_ = lin_x;
            current_ang_z_ = ang_z;
        }
    }

  private:
    ros::NodeHandle n_;
    ros::ServiceClient client_;
    ros::Subscriber sub1_;

    int far_angle_;
    float current_lin_x_;
    float current_ang_z_;

    // This method calls the command_robot service to drive the robot in the specified direction
    void drive_robot_(float lin_x, float ang_z) {
        // Request desired vlelocities
        surveyor::DriveToTarget srv;
        srv.request.linear_x = lin_x;
        srv.request.angular_z = ang_z;

        // Call the command_robot service and pass the requested velocities
        if (!client_.call(srv)) {
            ROS_ERROR("Failed to call service command_robot");
        }
    }

    // This method bounds measurements to range limits
    float preprocess_(const sensor_msgs::LaserScan & lsr, float data) {
        return std::min<float>(std::max<float>(data, lsr.range_min), lsr.range_max);
    }
};  // class CommandRobotClient


int main(int argc, char * * argv) {
    // Initialize the process_laser node and create a handle to it
    ros::init(argc, argv, "process_laser");

    // Create a an object of class CommandRobotClient
    CommandRobotClient CRCObject;

    // Handle ROS communication events
    ros::spin();

    return 0;
}
