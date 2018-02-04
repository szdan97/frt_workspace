#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>
#include <Eigen/Core>

//Custom ROS message includes
#include <frt_custom_msgs/SlamMap.h>
#include <frt_custom_msgs/Measurements.h>
#include <frt_custom_msgs/SlamDataAssociation.h>
#include <frt_custom_msgs/Odometry.h>

ros::ServiceClient srvClient;
ros::Publisher publisher;
frt_custom_msgs::SlamMap map;
frt_custom_msgs::Odometry lastOdometry;

//SLAM state
Eigen::VectorXd mean(3);
Eigen::MatrixXd covariance(3, 3);


void cameraMeasurementCallback(const frt_custom_msgs::Measurements &measurements);
void odometryCallback(const frt_custom_msgs::Odometry& o) { lastOdometry = o; }
double normalizeAngle(double orig);

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf_slam");

    mean.setZero();
    covariance.setZero();

    ros::NodeHandle node;
    ros::Subscriber subscriber = node.subscribe("camera_measurements", 1000, cameraMeasurementCallback);
    node.subscribe("odometry", 1000, odometryCallback);

    publisher = node.advertise<frt_custom_msgs::SlamMap>("slam_map", 1000);

    srvClient = node.serviceClient<frt_custom_msgs::SlamDataAssociation>("data_association", true);

    return 0;
}

void cameraMeasurementCallback(const frt_custom_msgs::Measurements &measurements) {
    frt_custom_msgs::SlamDataAssociationRequest daReq;
    frt_custom_msgs::SlamDataAssociationResponse daRes;
    daReq.measurements = measurements;
    daReq.map = map;

    if(srvClient.call(daReq, daRes)) {
        auto assoc = daRes.associations;

        //prediction step
        double mx = mean(0);
        double my = mean(1);
        double mtheta = mean(2);

        //computing the new mean of the robot pose using the odometry motion model
        double mtheta_new = normalizeAngle(mtheta + lastOdometry.theta1);
        double mx_new = mx + lastOdometry.d * sin(mtheta);
        double my_new = my + lastOdometry.d * cos(mtheta);
        mtheta_new = normalizeAngle(mtheta_new + lastOdometry.theta2);
        mean(0) = mx_new;
        mean(1) = my_new;
        mean(2) = mtheta_new;

        //computing the jacobian of the motion
        Eigen::MatrixXd motionJacobian(mean.size());
        motionJacobian.setIdentity();
        motionJacobian(0, 2) = lastOdometry.d * cos(mtheta + lastOdometry.theta1);
        motionJacobian(1, 2) = -lastOdometry.d * sin(mtheta + lastOdometry.theta1);

        covariance = motionJacobian * covariance * motionJacobian.transpose(); //TODO: add motion noise matrix
    }
}

double normalizeAngle(double orig) {
    if(orig > M_PI)
        return orig - 2 * M_PI;
    if(orig < -M_PI)
        return orig + 2 * M_PI;

}
