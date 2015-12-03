#pragma once

#include <Features/Sampler/IPatchSampler.h>

#include <pcl/ModelCoefficients.h>

#ifndef CYLINDERSAMPLER_H
#define CYLINDERSAMPLER_H

namespace hpe
{
    class CylinderSampler : public IPatchSampler<pcl::PointXYZRGBNormal>
    {
        public:
            typedef std::pair<float, float> Range;
            typedef pcl::PointXYZRGBNormal PointType;
            typedef pcl::PointCloud<PointType> CloudType;

            CylinderSampler(Range phiRange, Range zRange, int topRows, int bottomRows, int sampleColumns);
            ~CylinderSampler(void);

            pcl::PointCloud<PointType>::Ptr SampleCloud(pcl::PointCloud<PointType>::Ptr &cloud);
            pcl::PointCloud<PointType>::Ptr SampleCloud(pcl::PointCloud<PointType>::Ptr &cloud, const Common<PointType>::Landmarks &landmarks, pcl::ModelCoefficients &coefficients);

            void SetVisualize(bool visualizeSampler);
            void SetEyeIndices(std::vector<int> &leftEyeIndices, std::vector<int> &rightEyeIndices);
            void SetSupportOptimizedSampling(bool support);


        private:
            pcl::ModelCoefficients ComputeCylinder(PointType p1, PointType p2);
            PointType MeanPoint(const Common<PointType>::Landmarks &landmarks, const std::vector<int> &indices);
            CloudType::Ptr GenerateSamplingCloud(pcl::ModelCoefficients cylinder, Eigen::Vector3f direction, Range phiRange, int phiSteps, std::vector<float> samplingYs);
            Eigen::Vector3f GetDirection(pcl::PointCloud<PointType>::Ptr &cloud, const Common<PointType>::Landmarks &landmarks);
            std::vector<int> m_leftEyeIndices;
            std::vector<int> m_rightEyeIndices;
            int m_noseTipIndex;

            bool m_visualize;
            bool m_supportOptimizedSampling;

            Range m_phiRange;
            Range m_zRange;
            int m_topRows;
            int m_bottomRows;
            int m_sampleColumns;
    };
}

#endif