#include <ros/ros.h>
#include <frt_custom_msgs/SlamMap.h>
#include <frt_custom_msgs/Measurements.h>
#include <frt_custom_msgs/SlamDataAssociation.h>

void cameraMeasurementCallback(const frt_custom_msgs::Measurements &measurements);
ros::ServiceClient srvClient;
ros::Publisher publisher;
frt_custom_msgs::SlamMap map;

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf_slam");

    ros::NodeHandle node;
    ros::Subscriber subscriber = node.subscribe("camera_measurements", 1000, cameraMeasurementCallback);

    publisher = node.advertise<frt_custom_msgs::SlamMap>("slam_map", 1000);

    srvClient = node.serviceClient<frt_custom_msgs::SlamDataAssociation>("data_association", true);
}

void cameraMeasurementCallback(const frt_custom_msgs::Measurements &measurements) {
    frt_custom_msgs::SlamDataAssociationRequestConstPtr daReq;
    frt_custom_msgs::SlamDataAssociationResponseConstPtr daRes;
    daReq->measurements = measurements;
    daReq->map = map;

    if(srvClient.call(daReq, daRes)) {

    }
}

