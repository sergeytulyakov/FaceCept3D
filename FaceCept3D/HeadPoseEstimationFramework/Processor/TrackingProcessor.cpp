#include "TrackingProcessor.h"

#include <pcl/common/common.h>
#include <pcl/io/io.h>
#include <pcl/search/kdtree.h>

#include <boost/algorithm/string.hpp>

#include <UI/PointPicker.h>
#include <UI/ShowCloud.h>
#include <Filter/Filters.h>
#include <Landmarks.h>
#include <Converter/KinectDataConverter.h>

#include <DataObject/CloudXYZRGBA.h>
#include <DataObject/LandmarksObject.h>
#include <DataObject/HeadPoseInfo.h>
#include <DataObject/RawFrames.h>

#include <Grabber/PCDGrabber.h>

#include <Helpers/HpeHelpers.h>
#include <boost/filesystem/operations.hpp>

namespace hpe
{
    TrackingProcessor::TrackingProcessor(Cloud::Ptr templateCloud)
        : m_template(new Cloud), m_first(true), m_leftEyeIdx(-1), m_rightEyeIdx(-1), m_noseIdx(-1), m_saveLandmarks(false)
    {
        pcl::copyPointCloud(*templateCloud, *m_template);
        m_template = Voxelize<PointType>(m_template, 0.0075);

        CloudMapper::DynamicICPParams dynamicIcpParams;
        dynamicIcpParams.push_back(CloudMapper::ICPParams(0.007, 0.005, 10e-6, 400, 10));
        m_mapper.SetDynamicICPParams(dynamicIcpParams);
        m_mapper.SetUsePointToPlaneMetric(true);

        m_tracker = std::shared_ptr<ICPTemplateTracker<PointType>>(new ICPTemplateTracker<PointType>(m_template));
    }

    TrackingProcessor::~TrackingProcessor(void)
    {
    }

    void TrackingProcessor::Process(IDataStorage::Ptr storage)
    {
        CloudXYZRGBA::Ptr cloudObject = storage->GetAndCastNotNull<CloudXYZRGBA>("Cloud", "TrackingProcessor::Process - Cloud is null");
        CloudXYZRGBA::Ptr updatedTemplateObject(new CloudXYZRGBA);
        updatedTemplateObject->cloud = Cloud::Ptr(new Cloud);

        HeadPoseInformation::Ptr info = storage->GetAndCast<HeadPoseInformation>("HeadPoseInfo");
        PCDGrabber::CloudFileInfo::Ptr fileinfo = storage->GetAndCast<PCDGrabber::CloudFileInfo>("FileInfo");

        if (m_first)
        {
            if (m_templateLandmarks.size() != 3)
            {
                m_templateLandmarks = GetLandmarks(m_template);
            }
            m_leftEyeIdx = m_templateLandmarks[1].index;
            m_rightEyeIdx = m_templateLandmarks[0].index;
            m_noseIdx = m_templateLandmarks[2].index;
            if (info.get() == nullptr)
            {
                Common<PointType>::Landmarks frameLandmarks;
                if (fileinfo.get() != nullptr)
                {
                    std::string landmarksFile = boost::replace_all_copy(fileinfo->Filename, ".pcd", ".bnd");
                    if (boost::filesystem::exists(landmarksFile))
                    {
                        frameLandmarks = LoadLandmarks<pcl::PointXYZRGBA>(landmarksFile);
                    }
                }
                if (frameLandmarks.size() != 3)
                {
                    frameLandmarks = GetLandmarks(cloudObject->cloud);
                    std::string landmarksFile = boost::replace_all_copy(fileinfo->Filename, ".pcd", ".bnd");
                    SaveLandmarks<pcl::PointXYZRGBA>(frameLandmarks, landmarksFile);
                }
                Eigen::Matrix4f transform = m_mapper.GetTransformHavingLandmarks(m_template, m_templateLandmarks, cloudObject->cloud, frameLandmarks);
                pcl::transformPointCloud(*m_template, *(updatedTemplateObject->cloud), transform);
                m_tracker->RoughFirstTimeDetection(cloudObject->cloud, transform);
            }
            else
            {
                Eigen::Vector3f zVector = GetNormal<PointType>(m_template, m_noseIdx);
                if (zVector(2) < 0)
                {
                    zVector = -zVector;
                }

                RawFrames::Ptr frames = storage->GetAndCastNotNull<RawFrames>("RawFrames", "TrackingProcessor::Process - RawFrames is null");

                KinectDataConverter converter;

                float depthValue = 0;
                int squareSize = 0;
                while (depthValue == 0)
                {
                    cv::Mat square = frames->depthFrame
                                     .rowRange(info->DepthFrameY - squareSize, info->DepthFrameY + squareSize + 1)
                                     .colRange(info->DepthFrameX - squareSize, info->DepthFrameX + squareSize + 1);
                    double min, max;
                    cv::minMaxIdx(square, &min, &max);
                    depthValue = max;
                    squareSize += 1;
                }
                pcl::PointXYZ pt = converter.ConvertOnePoint(info->DepthFrameX, info->DepthFrameY, depthValue, cv::Size(frames->depthFrame.cols, frames->depthFrame.rows));

                Eigen::Vector3f orientation = pt.getVector3fMap();
                orientation /= orientation.norm();

                Eigen::Quaternionf rotation;
                rotation.setFromTwoVectors(zVector, orientation);
                Eigen::Matrix4f transformRotation = Eigen::Matrix4f::Identity();
                transformRotation.block<3, 3>(0, 0) = rotation.toRotationMatrix();
                pcl::transformPointCloud(*m_template, *(updatedTemplateObject->cloud), transformRotation);

                pcl::PointXYZRGBA nose = updatedTemplateObject->cloud->points[m_templateLandmarks[2].index];

                Eigen::Matrix4f transformTranslation = Eigen::Matrix4f::Identity();
                transformTranslation.block<3, 1>(0, 3) = pt.getVector3fMap() - nose.getVector3fMap();

                pcl::transformPointCloud(*(updatedTemplateObject->cloud), *(updatedTemplateObject->cloud), transformTranslation);

                Eigen::Matrix4f transform = transformRotation * transformTranslation;

                Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
                m_tracker->SetTemplate(updatedTemplateObject->cloud);
                m_tracker->RoughFirstTimeDetection(cloudObject->cloud, identity);
                updatedTemplateObject->cloud = m_tracker->Update(cloudObject->cloud);
            }
            m_first = false;
        }
        else
        {
            updatedTemplateObject->cloud = m_tracker->Update(cloudObject->cloud);
        }

        Common<PointType>::Landmarks landmarks = ResampleLandmarks<PointType>(m_templateLandmarks, updatedTemplateObject->cloud);
        //landmarks.push_back(Common<PointType>::Landmark(m_leftEyeIdx, updatedTemplateObject->cloud->points[m_leftEyeIdx]));
        //landmarks.push_back(Common<PointType>::Landmark(m_rightEyeIdx, updatedTemplateObject->cloud->points[m_rightEyeIdx]));
        //landmarks.push_back(Common<PointType>::Landmark(m_noseIdx, updatedTemplateObject->cloud->points[m_noseIdx]));
        PointType pointOnNormal = updatedTemplateObject->cloud->points[m_noseIdx];
        pointOnNormal.getArray3fMap() = pointOnNormal.getVector3fMap() + GetNormal<PointType>(updatedTemplateObject->cloud, m_noseIdx);
        landmarks.push_back(Common<PointType>::Landmark(-1, pointOnNormal));

        if (fileinfo.get() != nullptr)
        {
            if (m_saveLandmarks)
            {
                std::string eyespath = boost::replace_all_copy(fileinfo->Filename, ".pcd", ".bnd");
                std::cout << eyespath << std::endl;
                SaveLandmarks<PointType>(landmarks, eyespath);
            }
        }

        LandmarksObject<PointType>::Ptr landmarksObject(new LandmarksObject<PointType>);
        landmarksObject->landmarks = landmarks;
        storage->Set("Landmarks", landmarksObject);
        storage->Set("UpdatedTemplate", updatedTemplateObject);

        Eigen::Vector3f headPose = VectorToEulerAngles(GetNormal<PointType>(updatedTemplateObject->cloud, m_noseIdx));
        m_headPoseReadySignal(headPose);
    }

    Common<TrackingProcessor::PointType>::Landmarks TrackingProcessor::GetLandmarks(Cloud::Ptr cloud)
    {
        PointPicker<PointType> picker;
        picker.SetCloud(cloud);
        return picker.Pick("3 points", 3);
    }

    void TrackingProcessor::SetTemplateLandmarksFromFile(std::string landmarksFile)
    {
        Common<pcl::PointXYZRGBA>::Landmarks l;
        if (boost::filesystem::exists(landmarksFile))
        {
            l = LoadLandmarks<pcl::PointXYZRGBA>(landmarksFile);
        }
        else
        {
            PointPicker<pcl::PointXYZRGBA> picker;
            picker.SetCloud(m_template);
            l = picker.Pick("eyes and nose", 3);
            SaveLandmarks<pcl::PointXYZRGBA>(l, landmarksFile);
        }
        m_templateLandmarks = l;
    }


}
