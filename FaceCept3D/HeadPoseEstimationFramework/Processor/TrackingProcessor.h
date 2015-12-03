#pragma once

#ifndef TRACKINGPROCESSOR_H
#define TRACKINGPROCESSOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Processor/IProcessor.h>
#include <Align/CloudMapper.h>
#include <Tracker/ICPTemplateTracker.h>
#include <Common.h>
#include <Landmarks.h>

#include <boost/signals2/signal.hpp>

namespace hpe
{

    class TrackingProcessor : public IProcessor
    {
        public:
            typedef pcl::PointXYZRGBA PointType;
            typedef pcl::PointCloud<PointType> Cloud;

            TrackingProcessor(Cloud::Ptr templateCloud);
            ~TrackingProcessor(void);

            void Process(IDataStorage::Ptr storage);

            typedef boost::signals2::signal<void (Eigen::Vector3f)> HeadPoseReadySignal;
            void SubscribeHeadposeReadySignal(const HeadPoseReadySignal::slot_type &slot)
            {
                m_headPoseReadySignal.connect(slot);
            }

            void SetTemplateLandmarks(Common<pcl::PointXYZRGBA>::Landmarks &landmarks)
            {
                m_templateLandmarks = ResampleLandmarksByPosition<PointType>(landmarks, m_template);
            }

            void SetSaveLandmakrs(bool value)
            {
                m_saveLandmarks = value;
            }

            void ResetTracker()
            {
                m_tracker->ResetTracking();
                m_first = true;
            }

            void SetTemplateLandmarksFromFile(std::string landmarksFile);

        private:
            Common<PointType>::Landmarks GetLandmarks(Cloud::Ptr cloud);

            int m_leftEyeIdx;
            int m_rightEyeIdx;
            int m_noseIdx;

            bool m_first;
            Cloud::Ptr m_template;
            CloudMapper m_mapper;
            std::shared_ptr<ICPTemplateTracker<PointType>> m_tracker;

            Common<pcl::PointXYZRGBA>::Landmarks m_templateLandmarks;

            bool m_saveLandmarks;

            HeadPoseReadySignal m_headPoseReadySignal;
    };

}

#endif
