#include "DetectorProcessor.h"

#include <DataObject/RawFrames.h>
#include <DataObject/HeadPoseInfo.h>

#include <Converter/KinectDataConverter.h>

namespace hpe
{

    DetectorProcessor::DetectorProcessor(void)
        : m_detector("DetectorData")
    {
    }

    DetectorProcessor::DetectorProcessor(std::string dataDir)
        : m_detector(dataDir)
    {
    }

    DetectorProcessor::~DetectorProcessor(void)
    {
    }

    void DetectorProcessor::Process(hpe::IDataStorage::Ptr dataStorage)
    {
        hpe::RawFrames::Ptr frames = dataStorage->GetAndCastNotNull<hpe::RawFrames>("RawFrames", "DetectorProcessor::Process - can't find raw frames");

        hpe::headPoseInfo detectorInfo = m_detector.Detect(frames->depthFrame);

        hpe::HeadPoseInformation::Ptr info(new hpe::HeadPoseInformation);
        if (detectorInfo.x != 0 && detectorInfo.y != 0)
        {
            info->DepthFrameX = detectorInfo.x;
            info->DepthFrameY = detectorInfo.y;

            float depthValue = frames->depthFrame.at<float>(detectorInfo.y, detectorInfo.x);
            KinectDataConverter converter;
            pcl::PointXYZ pt = converter.ConvertOnePoint(detectorInfo.x, detectorInfo.y, depthValue, cv::Size(frames->depthFrame.cols, frames->depthFrame.rows));

            info->WorldX = pt.x;
            info->WorldY = pt.y;
            info->WorldZ = pt.z;

            if (detectorInfo.tilt != -100 && detectorInfo.yaw != -100)
            {
                info->Tilt = detectorInfo.tilt;
                info->Yaw = detectorInfo.yaw;
                info->HasHeadInfo = true;
            }
        }


        dataStorage->Set("HeadPoseInfo", info);
    }

}
