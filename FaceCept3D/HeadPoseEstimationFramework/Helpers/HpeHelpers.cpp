#include "HpeHelpers.h"



namespace hpe
{
    Eigen::Vector3f VectorToEulerAngles(Eigen::Vector3f v)
    {
        Eigen::Vector3f normed = v / v.norm();
        float x = normed(0);
        float y = normed(1);
        float z = normed(2);

        Eigen::Vector3f result;
        result(0) = std::atan2(z, x); // yaw
        result(1) = std::atan2(y, z); // tilt
        result(2) = std::atan(y / x); // roll
        return result;
    }

    Eigen::Vector3f EulerAnglesToVector(double yaw, double tilt, bool inDegrees)
    {
        if (inDegrees)
        {
            yaw = yaw / 180 * M_PI;
            tilt = tilt / 180 * M_PI;
        }

        Eigen::Vector3f newVector;
        double yawTan = std::tan(yaw);
        double tiltTan = std::tan(tilt);
        if (yawTan != 0)
        {
            newVector << 1 / yawTan, -tiltTan, 1;
        }
        else
        {
            newVector << 1000, -tiltTan, 1;
        }

        return newVector / newVector.norm();
    }

    cv::Point2f MeanPoint(std::vector<cv::Point2f> points)
    {
        cv::Point2f res(0, 0);
        for (auto pt = points.begin(); pt != points.end(); pt++)
        {
            res.x += pt->x;
            res.y += pt->y;
        }

        if (points.size() != 0)
        {
            res.x /= points.size();
            res.y /= points.size();
        }

        return res;
    }






}