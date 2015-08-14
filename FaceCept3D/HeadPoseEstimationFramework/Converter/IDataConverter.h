#ifndef IDATACONVERTER_H
#define IDATACONVERTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

#include "Interface/IInterface.h"

namespace hpe
{
    class IDataConverter : public IInterface
    {
        public:
            virtual pcl::PointCloud<pcl::PointXYZRGBA>::Ptr DepthRGBToPointCloud(cv::Mat &depth, cv::Mat &bgr) = 0;

    };
}

#endif // IDATACONVERTER_H
