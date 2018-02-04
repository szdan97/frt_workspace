#include <ros/ros.h>
#include <frt_custom_msgs/SlamDataAssociation.h>
#include <frt_custom_msgs/SlamDataAssociationResponse.h>
#include <frt_custom_msgs/SlamDataAssociationRequest.h>
#include <math.h>
#include <Eigen/Core>
#include <boost/array.hpp>

using namespace frt_custom_msgs;

bool dataAssociationServiceCallback(SlamDataAssociationRequest &req, SlamDataAssociationResponse &res);

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_association");

    ros::NodeHandle node;

    ros::ServiceServer serviceServer =
            node.advertiseService<SlamDataAssociationRequest, SlamDataAssociationResponse>("data_associaton", dataAssociationServiceCallback);

    return 0;
}

/// Returns the square of the mahalanobis distance between a given point and a 2d probability density given by its mean and covariance
/// \param xp x coordinate of the point
/// \param yp y coordinate of the point
/// \param xm x coordinate of the mean
/// \param ym y coordinate of the mean
/// \param sigma covariance matrix
/// \return the square of the mahalanobis distance between the given point and probability density
double squaredMahalanobis(double xp, double yp, double xm, double ym, Eigen::Matrix2d sigma) {
    auto dx = xm - xp;
    auto dy = ym - yp;
    auto d = Eigen::Vector2d();
    d << dx, dy;
    return d.transpose() * sigma * d;
}

/// Returns the mahalanobis distance between a given point and a 2d probability density given by its mean and covariance
/// \param xp x coordinate of the point
/// \param yp y coordinate of the point
/// \param xm x coordinate of the mean
/// \param ym y coordinate of the mean
/// \param sigma covariance matrix
/// \return the mahalanobis distance between the given point and probability density
double mahalanobis(double xp, double yp, double xm, double ym, Eigen::Matrix2d sigma) {
    return sqrt(squaredMahalanobis(xp, yp, xm, ym, sigma));
}

/// Converts a boost array representing a 2x2 matrix to a 2x2 eigen matrix
/// \param orig the array to convert from
/// \return the 2x2 matrix represented by the given array
Eigen::Matrix2d msgToEigen(boost::array<double, 4> &orig) {
    Eigen::Matrix2d matrix;
    matrix << orig[0], orig[1], orig[2], orig[3];
    return matrix;
}

/// Callback function called when the service is called.
/// Returns the data associations determined by a local nnsf algorithm based on mahalanobis distance in the res param.
/// \param req input parameter of the service
/// \param res output parameter, result of the service
bool dataAssociationServiceCallback(SlamDataAssociationRequest &req, SlamDataAssociationResponse &res) {
    const double INIT_DIST = std::numeric_limits<double>::has_infinity ? std::numeric_limits<double>::infinity() : 10000000000;

    auto measurements = req.measurements.landmarks;
    auto map = req.map.map;
    bool used[map.size()] = {false};
    int associations[measurements.size()];

    for (int i = 0; i < measurements.size(); ++i) {
        auto measurement = measurements[i];
        double minSquaredDist = INIT_DIST;
        for (int j = 0; j < map.size(); ++j) {
            auto &landmark = map[j];
            if (!used[j] && measurement.color == landmark.landmark.color) {
                auto squaredDist = squaredMahalanobis(
                        measurement.x, measurement.y,
                        landmark.landmark.x, landmark.landmark.y,
                        msgToEigen(landmark.covariance)
                );
                if (squaredDist < minSquaredDist) {
                    minSquaredDist = squaredDist;
                    associations[i] = j;
                }
            }
        }
        if(associations[i] != -1)
            used[associations[i]] = true;
    }

    res.associations = std::vector<int>(associations, associations + measurements.size());
    return true;
}
