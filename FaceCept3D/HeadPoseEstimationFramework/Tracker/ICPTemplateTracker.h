#ifndef ICPTEMPLATETRACKER_H
#define ICPTEMPLATETRACKER_H

#include "Common.h"
#include "Align/CloudMapper.h"
#include <DataFilter/IDataFilter.h>

#include <memory.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/common/time.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>

namespace hpe
{
//FIXME Now works only with pcl::PointXYZRGBA because CloudMapper is not a template
    template <typename TPoint>
    class ICPTemplateTracker
    {
        public:
            typedef pcl::PointCloud<TPoint> TCloud;

            ICPTemplateTracker(typename TCloud::Ptr &templateCloud)
                : m_templateCloud(templateCloud), m_canPredict(false), m_transformationFilter(nullptr)
            {
                m_mapper = std::shared_ptr<CloudMapper>(new CloudMapper());
                m_mapper->SetUseNormalShooting(false);
                m_mapper->SetUsePointToPlaneMetric(true);

                m_icpParams.push_back(CloudMapper::ICPParams(0.005, 0.005, 10e-6, 2000, 5));
                m_mapper->SetDynamicICPParams(m_icpParams);
                m_mapper->SetUsePointToPlaneMetric(true);
                m_followingTemplate = typename TCloud::Ptr(new TCloud);
            }

            void RoughFirstTimeDetection(typename TCloud::Ptr &frame, Eigen::Matrix4f &transformation)
            {
                pcl::transformPointCloud(*m_templateCloud, *m_followingTemplate, transformation);
                m_canPredict = true;
            }

            void SetTemplate(typename TCloud::Ptr &templateCloud)
            {
                pcl::copyPointCloud(*templateCloud, *m_templateCloud);
                m_canPredict = false;
            }

            void GetRelevantPoints(typename TCloud::Ptr &newFrame, std::vector<int> &points, float distance)
            {
                pcl::search::KdTree<TPoint> search;
                search.setInputCloud(newFrame);
                for (int i = 0; i < m_followingTemplate->size() ; i += 1)
                {
                    static const int neighbours = 1;
                    std::vector<int> pointIdx(neighbours);
                    std::vector<float> pointDistance(neighbours);
                    if (search.nearestKSearch(m_followingTemplate->points[i], neighbours, pointIdx, pointDistance))
                    {
                        if (pointDistance[0] < distance)
                        {
                            points.push_back(i);
                        }
                    }
                }
            }

            typename TCloud::Ptr Update(typename TCloud::Ptr &newFrame)
            {
                m_followingTransform = m_mapper->GetTransformForTwoCloudsDynamically(m_followingTemplate, newFrame, m_icpParams);

                if (m_transformationFilter != nullptr)
                {
                    m_followingTransform = m_transformationFilter->Filter(m_followingTransform);
                }

                typename TCloud::Ptr updatedCloud(new TCloud);
                pcl::transformPointCloud(*m_followingTemplate, *updatedCloud, m_followingTransform);
                pcl::copyPointCloud(*updatedCloud, *m_followingTemplate);

                std::vector<int> pointIndexes;
                GetRelevantPoints(newFrame, pointIndexes, 5e-6);
                if (m_pointHistory.size() >= HistorySize)
                {
                    m_pointHistory.pop_front();
                }
                m_pointHistory.push_back(pointIndexes);
                std::vector<double> weights = WeightsFromPointHistory(m_pointHistory, m_followingTemplate->size());
                m_mapper->SetWeights(weights);

                return updatedCloud;
            }

            Eigen::Matrix4f GetCurrentTransform()
            {
                return m_followingTransform;
            }

            void SetTransformationFilter(std::shared_ptr<DataFilter<Eigen::Matrix4f>> &dataFilter)
            {
                m_transformationFilter(dataFilter);
            }

            bool CanPredict()
            {
                return m_canPredict;
            }

            void ResetTracking()
            {
                m_canPredict = false;
            }

        private:
            std::vector<double> WeightsUniform(int totalPoints, double weight = 1)
            {
                std::vector<double> weights;
                weights.reserve(totalPoints);
                for (int i = 0; i < totalPoints; i += 1)
                {
                    weights[i] = weight;
                }
                return weights;
            }

            std::vector<double> WeightsFromPointHistory(PointHistory &history, int totalPoints)
            {
                std::vector<double> weights(totalPoints);
                for (auto indexes = history.begin(); indexes != history.end(); indexes++)
                {
                    for (auto index = indexes->begin(); index != indexes->end(); index++)
                    {
                        weights[*index] += 1;
                    }
                }
                for (int i = 0; i < weights.size(); i += 1)
                {
                    weights[i] = 0.3 + 0.4 * weights[i];
                }
                return weights;
            }

            std::vector<double> WeightsGaussian(typename TCloud::Ptr &cloud)
            {
                TPoint &noseTip = cloud->points[0];

                std::vector<double> weights(cloud->points.size());
                const double sigma = 0.1;
                for (int i = 0; i < cloud->points.size(); i += 1)
                {
                    TPoint &pt = cloud->points[i];
                    double dist = (pt.x - noseTip.x) * (pt.x - noseTip.x) + (pt.y - noseTip.y) * (pt.y - noseTip.y) + (pt.z - noseTip.z) * (pt.z - noseTip.z);
                    weights[i] = 0.3 + std::exp(- dist / 2 / sigma / sigma);
                }
                return weights;
            }

            std::vector<double> WeightsGaussianAndHistory(PointHistory &history, typename TCloud::Ptr &cloud)
            {
                std::vector<double> weightsFromHistory = WeightsFromPointHistory(history, cloud->size());
                std::vector<double> weightsFromGaussian = WeightsGaussian(cloud);
                std::vector<double> weights(cloud->size());
                for (int i = 0; i < cloud->size(); i += 1)
                {
                    weights[i] = weightsFromGaussian[i] + weightsFromHistory[i];
                }
                return weights;
            }

            bool m_canPredict;
            static const int HistorySize = 5;

            typename TCloud::Ptr m_templateCloud;
            typename TCloud::Ptr m_followingTemplate;

            std::shared_ptr<CloudMapper> m_mapper;
            PointHistory m_pointHistory;

            Eigen::Matrix4f m_followingTransform;

            CloudMapper::DynamicICPParams m_icpParams;

            std::shared_ptr<DataFilter<Eigen::Matrix4f>> m_transformationFilter;

    };
}

#endif // ICPTEMPLATETRACKER_H
