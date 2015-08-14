#include "KinectDataConverter.h"

#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/math/special_functions/round.hpp>

namespace hpe
{
    typedef union
    {
        struct
        {
            unsigned char Blue;
            unsigned char Green;
            unsigned char Red;
            unsigned char Alpha;
        };
        float float_value;
        uint32_t long_value;
    } RGBValue;

    KinectDataConverter::KinectDataConverter()
    {
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr KinectDataConverter::DepthRGBToPointCloud(cv::Mat &depth, cv::Mat &bgr)
    {
        cv::Size depthImageSize = depth.size();

        float centerX, centerY;
        float scaleFactorX, scaleFactorY;

        ComputeCameraIntrinsics(depthImageSize, centerX, centerY, scaleFactorX, scaleFactorY);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBA>);
        result->width = depthImageSize.width;
        result->height = depthImageSize.height;
        result->is_dense = false;
        result->points.resize(depthImageSize.width * depthImageSize.height);

        for (int y = 0; y < depthImageSize.height; ++y)
        {
            for (int x = 0; x < depthImageSize.width; ++x)
            {
                pcl::PointXYZRGBA &pt = result->at(x, y);

                float depthValue = (float) depth.at<float>(y, x);

                if (depthValue == -1000.f || depthValue == 0.0f || boost::math::isnan(depthValue))
                {
                    pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
                }
                else
                {
                    //depthValue /= 1000;
                    pt.x = (static_cast<float>(x) - centerX) * scaleFactorX * depthValue;
                    pt.y = (centerY - static_cast<float>(y)) * scaleFactorY * depthValue;
                    pt.z = depthValue;

                    cv::Vec3b colorValue = bgr.at<cv::Vec3b>(y, x);
                    RGBValue color;
                    color.Red = colorValue[2];
                    color.Green = colorValue[1];
                    color.Blue = colorValue[0];
                    pt.rgba = color.long_value;
                }
            }
        }

        return result;
    }

    void KinectDataConverter::ComputeCameraIntrinsics(cv::Size &imageSize, float &centerX, float &centerY, float &scaleFactorX, float &scaleFactorY)
    {
        int dims[] = {imageSize.width, imageSize.height};

        // The 525 factor default is only true for VGA. If not, we should scale
        scaleFactorX = scaleFactorY = 1 / 525.f * 640.f / dims[0];
        centerX = ((float)dims[0] - 1.f) / 2.f;
        centerY = ((float)dims[1] - 1.f) / 2.f;
    }

    pcl::PointXYZ KinectDataConverter::ConvertOnePoint(float x, float y, float depthValue, cv::Size frameSize)
    {
        float centerX, centerY;
        float scaleFactorX, scaleFactorY;

        ComputeCameraIntrinsics(frameSize, centerX, centerY, scaleFactorX, scaleFactorY);

        pcl::PointXYZ pt;
        pt.x = (static_cast<float>(x) - centerX) * scaleFactorX * depthValue;
        pt.y = (centerY - static_cast<float>(y)) * scaleFactorY * depthValue;
        pt.z = depthValue;
        return pt;
    }

}
