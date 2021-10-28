//
// Created by redwan on 10/17/21.
//

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "traj_min_snap.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include "sensor_msgs/Joy.h"
//#define SIMULATION true

using namespace std;
using namespace Eigen;
typedef vector<pair<double, double>> WP;
typedef deque<tuple<double, double, double, double>> TRAJ;

double width, height, theta;
double max_vel, max_acc;
TRAJ trajectory;
ros::Publisher trajPub;
WP path;



VectorXd allocateTime(const MatrixXd &wayPs,
                      double vel,
                      double acc)
{
    int N = (int)(wayPs.cols()) - 1;
    VectorXd durations(N);
    if (N > 0)
    {

        Eigen::Vector3d p0, p1;
        double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
        for (int k = 0; k < N; k++)
        {
            p0 = wayPs.col(k);
            p1 = wayPs.col(k + 1);
            D = (p1 - p0).norm();

            acct = vel / acc;
            accd = (acc * acct * acct / 2);
            dcct = vel / acc;
            dccd = acc * dcct * dcct / 2;

            if (D < accd + dccd)
            {
                t1 = sqrt(acc * D) / acc;
                t2 = (acc * t1) / acc;
                dtxyz = t1 + t2;
            }
            else
            {
                t1 = acct;
                t2 = (D - accd - dccd) / vel;
                t3 = dcct;
                dtxyz = t1 + t2 + t3;
            }

            durations(k) = dtxyz;
        }
    }

    return durations;
}

MatrixXd path_to_waypoint_matrix(const WP& path)
{
    MatrixXd waypoints;
    int count = 0;
    waypoints.resize(3, path.size());

    for (auto p: path)
    {
        waypoints(0, count ) = p.first;
        waypoints(1, count ) = p.second;
        waypoints(2, count ) = 1.0;
        count++;
    }
    return waypoints;
}


TRAJ path_to_trajectory(const MatrixXd& waypoints, int num_points)
{
    VectorXd ts;
    Matrix3d iS, fS;
    Eigen::Matrix<double, 3, 4> iSS, fSS;

    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory minSnapTraj;
    iS.setZero();
    fS.setZero();

    iS.col(0) << waypoints.leftCols<1>();
    fS.col(0) << waypoints.rightCols<1>();
    ts = allocateTime(waypoints, max_vel, max_acc);

    iSS << iS, Eigen::MatrixXd::Zero(3, 1);
    fSS << fS, Eigen::MatrixXd::Zero(3, 1);

    snapOpt.reset(iSS, fSS, waypoints.cols() - 1);
    snapOpt.generate(waypoints.block(0, 1, 3, num_points - 1), ts);
    snapOpt.getTraj(minSnapTraj);



    //            populate trajectory
    TRAJ trajectory;
    float total_time = 0;
    for(auto traj:minSnapTraj)
    {
        float time = 0;
        while( time <= traj.getDuration())
        {
            auto pos = traj.getPos(time);
            trajectory.push_back(make_tuple(total_time, pos[0], pos[1], pos[2]));
            time += 0.1;
            total_time += 0.1;
        }
        ROS_INFO_STREAM("(" << total_time << ") [traj duration]: "<< time );
    }
    return trajectory;

}

void trajTimerCallback(const ros::TimerEvent& event)
{
    if (trajectory.size() < 1)
        return;
    double time, x, y, z;
    std::tie(time, x, y, z) = trajectory.front();
    ROS_DEBUG("[size = %d] time %lf x = %lf y = %lf z = %lf", (int)trajectory.size(), time, x, y, z );
    trajectory.pop_front();
    geometry_msgs::Point msg;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    trajPub.publish(msg);

}

void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if(msg->buttons[0])
    {
        MatrixXd pathMat = path_to_waypoint_matrix(path);
        ROS_INFO_STREAM("path size " << path.size());
        trajectory = path_to_trajectory(pathMat, path.size() - 1);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "bebop trajectory generator");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("bebop trajectory generator");

    nh.getParam("rect_width", width);
    nh.getParam("rect_height", height);
    nh.getParam("path_theta", theta);

    nh.getParam("max_vel", max_vel);
    nh.getParam("max_acc", max_acc);

    vector<double>X, Y;
    nh.getParam("wp_x", X);
    nh.getParam("wp_y", Y);
    for (int i = 0; i < X.size(); ++i) {
        path.emplace_back(make_pair(X[i], Y[i]));
    }

#ifdef SIMULATION
    MatrixXd pathMat = path_to_waypoint_matrix(path);
    ROS_INFO_STREAM("path size " << path.size());
    trajectory = path_to_trajectory(pathMat, path.size() - 1);
#endif

    trajPub = nh.advertise<geometry_msgs::Point>("/bebop/trajectory", 1);

    ros::Subscriber joy_sub = nh.subscribe("/joy", 1, joystickCallback);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), trajTimerCallback);
    ros::spin();


    return 0;
}