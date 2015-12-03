#pragma once

#ifndef FEATURE_CALCULATOR_BASE_H
#define FEATURE_CALCULATOR_BASE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#ifndef EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#endif
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/normal_3d.h>

#include <Features/Sampler/IPatchSampler.h>
#include <Features/IFeature.h>
#include <Filter/Filters.h>
#include <Landmarks.h>

namespace hpe
{
    template<typename PointType, typename NormalType>
    class FeatureCalculatorBase
    {
        public:
            void SetLandmarks(const typename Common<PointType>::Landmarks &landmarks)
            {
                m_landmarks = landmarks;
            }
        protected:
            typedef PointType TPoint;
            typedef NormalType TNormal;
            typedef std::shared_ptr<IPatchSampler<PointType>> Sampler;
            typedef pcl::PointCloud<PointType> Cloud;
            typedef pcl::PointCloud<NormalType> NormalCloud;
            typedef pcl::search::KdTree<PointType> Search;

            FeatureCalculatorBase(Sampler sampler)
                : m_sampler(sampler)
            {

            }

            void Calculate(typename Cloud::Ptr &cloud)
            {
                pcl::ModelCoefficients coefficients;
                if (m_landmarks.size() != 0)
                {
                    m_featurePoints = m_sampler->SampleCloud(cloud, m_landmarks, coefficients);
                    m_landmarks.clear();
                }
                else
                {
                    m_featurePoints = m_sampler->SampleCloud(cloud, coefficients);
                }

                auto normalCloud = m_featurePoints;

                int numberOfSamples = m_featurePoints->size();
                m_featureNumber = 0;

                if (coefficients.values.size() == 0)
                {
                    PreCompute();
                }
                else
                {
                    PreCompute(cloud, coefficients);
                }

                for (int i = 0; i < numberOfSamples; i++)
                {
                    auto gridPoint = m_featurePoints->at(i);
                    PointType point;
                    NormalType normal;

                    if ((boost::math::isnan(gridPoint.x) || boost::math::isnan(gridPoint.y) || boost::math::isnan(gridPoint.z)) == false)
                    {
                        // TODO make both usages:
                        int samplePointIndex = i; //GetReferencePoint(gridPoint);
                        point = m_featurePoints->at(samplePointIndex);
                        normal = normalCloud->at(samplePointIndex);
                    }

                    ComputeFeature(cloud, point, normal);

                    m_featureNumber++;
                }

                m_featureNumber = 0;
            }

            int GetReferencePoint(PointType &point)
            {
                std::vector<int> indices;
                std::vector<float> distances;

                m_search->nearestKSearch(point, 1, indices, distances);

                return indices[0];
            }

            int GetFeatureNumber()
            {
                return m_featureNumber;
            }

            int GetFeatureCount()
            {
                return m_featurePoints->size();
            }

            virtual void ComputeFeature(typename Cloud::Ptr cloud, PointType &point, NormalType &normal) = 0;
            virtual void PreCompute() = 0;
            virtual void PreCompute(typename Cloud::Ptr cloud, pcl::ModelCoefficients &coefficients) {}

            typename Search::Ptr m_search;
            Sampler m_sampler;

        private:
            int m_featureNumber;
            typename Common<PointType>::Landmarks m_landmarks;
            typename Cloud::Ptr m_featurePoints;
    };

}

#endif //FEATURE_CALCULATOR_BASE_H

