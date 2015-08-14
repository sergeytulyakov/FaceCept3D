#include "KinectSDKConverterProcessor.h"

#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/math/special_functions/round.hpp>

#include <Windows.h>
#include <NuiApi.h>

#include <WindowsOnly/Grabber/KinectSDKGrabber.h>
#include <Exception/HPEException.h>
#include <DataObject/RawFrames.h>
#include <DataObject/CloudXYZRGBA.h>

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

    KinectSDKConverterProcessor::KinectSDKConverterProcessor() : m_cloudKey("Cloud")
    {
    }

    KinectSDKConverterProcessor::KinectSDKConverterProcessor(std::string key) : m_cloudKey(key)
    {
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr KinectSDKConverterProcessor::DepthRGBToPointCloud(cv::Mat &depth, cv::Mat &bgr)
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

                float depthValue = depth.at<float>(y, x);

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

                    int depthIndex = x + y * 640;

                    LONG actX = m_correspondencies[depthIndex * 2];
                    LONG actY = m_correspondencies[depthIndex * 2 + 1];

                    if (actX >= bgr.cols || actX < 0 || actY < 0 || actY >= bgr.rows)
                    {
                        pt.rgba = 0;
                    }
                    else
                    {
                        cv::Vec4b colorValue = bgr.at<cv::Vec4b>(actY, actX);
                        RGBValue color;
                        color.Red = colorValue[2];
                        color.Green = colorValue[1];
                        color.Blue = colorValue[0];
                        pt.rgba = color.long_value;
                    }
                }
            }
        }

        return result;
    }

    void KinectSDKConverterProcessor::ComputeCameraIntrinsics(cv::Size &imageSize, float &centerX, float &centerY, float &scaleFactorX, float &scaleFactorY)
    {
        int dims[] = {imageSize.width, imageSize.height};

        // The 525 factor default is only true for VGA. If not, we should scale
        scaleFactorX = scaleFactorY = 1 / 525.f * 640.f / dims[0];
        centerX = (static_cast<float>(dims[0]) - 1.f) / 2.f;
        centerY = (static_cast<float>(dims[1]) - 1.f) / 2.f;
    }

    void KinectSDKConverterProcessor::Process(IDataStorage::Ptr dataStorage)
    {
        std::shared_ptr<KinectCoordinatesMapper> mapperObject = std::dynamic_pointer_cast<KinectCoordinatesMapper>(dataStorage->Get("CoordinatesMapper"));
        if (mapperObject.get() == nullptr)
        {
            throw HPEException("KinectSDKConverterProcessor::Process - mapperObject.get() == nullptr");
        }
        m_correspondencies = mapperObject->correspondencies;

        IDataObject::Ptr object = dataStorage->Get("RawFrames");
        RawFrames::Ptr rawFrames = std::dynamic_pointer_cast<RawFrames>(object);
        if (rawFrames.get() == nullptr)
        {
            throw HPEException("KinectSDKConverterProcessor::Process - rawFrames.get() == nullptr");
        }

        CloudXYZRGBA::Ptr cloudObject(new CloudXYZRGBA);
        cloudObject->cloud = DepthRGBToPointCloud(rawFrames->depthFrame, rawFrames->colorFrame);
        cloudObject->cloud->width = cloudObject->cloud->width * cloudObject->cloud->height;
        cloudObject->cloud->height = 1;

        dataStorage->Set(m_cloudKey, cloudObject);
    }

}
