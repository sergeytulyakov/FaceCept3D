#pragma once

#ifndef CYLINDEROPTIMIZEDFEATURECALCULATOR_H
#define CYLINDEROPTIMIZEDFEATURECALCULATOR_H

#include <Features/Calculator/FeatureCalculatorBase.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/math/special_functions/fpclassify.hpp>

#include <UI/ShowCloud.h>

namespace hpe
{
    enum FeatureType { Depth, RGB };

    template <class PointType, class NormalType>
    class CylinderOptimizedFeatureCalculator : public FeatureCalculatorBase <PointType, NormalType>
    {
        public:
            typedef PointType TPoint;
            typedef NormalType TNormal;
            typedef std::shared_ptr<IPatchSampler<PointType>> Sampler;
            typedef pcl::PointCloud<PointType> Cloud;
            typedef pcl::PointCloud<NormalType> NormalCloud;
            typedef pcl::search::KdTree<PointType> Search;

            CylinderOptimizedFeatureCalculator(Sampler sampler, FeatureType featureType)
                : FeatureCalculatorBase<PointType, NormalType>(sampler), m_visualize(false), m_featureType(featureType)
            {
            }

            ~CylinderOptimizedFeatureCalculator(void)
            {
            }

            void SetFeatureSize(cv::Size size)
            {
                m_featureSize = size;
            }

            cv::Mat GetFeatures(typename Cloud::Ptr &cloud)
            {
                m_result = CreateResult();
                m_featureNumber = 0;
                this->Calculate(cloud);
                return m_result;
            }

        protected:
            void PreCompute() {}

            cv::Mat CreateResult()
            {
                if (m_featureType == FeatureType::Depth)
                {
                    return cv::Mat::ones(1, m_featureSize.width * m_featureSize.height, CV_32FC1) * -100;
                }
                else if (m_featureType == FeatureType::RGB)
                {
                    return cv::Mat::zeros(1, m_featureSize.width * m_featureSize.height, CV_8UC3);
                }
            }

            void PreCompute(typename Cloud::Ptr cloud, pcl::ModelCoefficients &coefficients)
            {
                m_columns.clear();
                m_rows.clear();

                m_referencePoint.x = coefficients.values[0];
                m_referencePoint.y = coefficients.values[1];
                m_referencePoint.z = coefficients.values[2];

                m_phiRange.first = coefficients.values[3];
                m_phiRange.second = coefficients.values[4];
                m_phiSteps = (int)coefficients.values[5];

                m_yRange.first = coefficients.values[6];
                m_yRange.second = coefficients.values[7];
                m_ySteps = (int)coefficients.values[8];

                //std::vector<std::vector<int>> indices;
                m_columns.resize(m_phiSteps);
                m_rows.resize(m_ySteps);

                for (int i = 0; i < cloud->points.size(); i += 1)
                {
                    PointType pt = cloud->points[i];
                    if (boost::math::isnan(pt.x) || boost::math::isnan(pt.y) || boost::math::isnan(pt.z))
                    {
                        continue;
                    }
                    int column = GetPointColumn(pt);
                    if (column != -1)
                    {
                        if (column > 0) m_columns[column - 1].push_back(i);
                        m_columns[column].push_back(i);
                        if (column < m_phiSteps - 1) m_columns[column + 1].push_back(i);
                    }
                    int row = GetPointRow(pt);
                    if (row != -1)
                    {
                        if (row > 0) m_rows[row - 1].push_back(i);
                        m_rows[row].push_back(i);
                        if (row < m_ySteps - 1) m_rows[row + 1].push_back(i);
                    }
                }

                for (int i = 0; i < m_phiSteps; i += 1)
                {
                    std::sort(m_columns[i].begin(), m_columns[i].end());
                }
                for (int i = 0; i < m_ySteps; i += 1)
                {
                    std::sort(m_rows[i].begin(), m_rows[i].end());
                }

                if (m_visualize)
                {
                    pcl::visualization::PCLVisualizer v("Segments");
                    v.addPointCloud<PointType>(cloud);
                    v.addPointCloud<PointType>(cloud, "pts");
                    v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "pts");
                    for (int i = 0; i < m_phiSteps; i += 1)
                    {
                        typename Cloud::Ptr subcloud(new Cloud);
                        pcl::copyPointCloud(*cloud, m_columns[i], *subcloud);
                        v.updatePointCloud<PointType>(subcloud, "pts");
                        v.spinOnce(100);
                    }
                    v.removeAllPointClouds();
                }

                /*m_trees.clear();
                m_trees.resize(m_phiSteps);
                for (int i = 0; i < m_phiSteps; i += 1)
                {
                    Cloud::Ptr subcloud(new Cloud);
                    pcl::copyPointCloud(*cloud, indices[i], *subcloud);
                    m_trees[i].setInputCloud(subcloud);
                }*/

            }
            void ComputeFeature(typename Cloud::Ptr cloud, PointType &point, NormalType &normal)
            {
                int column = GetPointColumn(point);
                if (column == -1)
                {
                    throw HPEException("CylinderOptimizedFeatureCalculator::ComputeFeature - internal structures corrupted");
                }
                int row = GetPointRow(point);
                if (column == -1)
                {
                    throw HPEException("CylinderOptimizedFeatureCalculator::ComputeFeature - internal structures corrupted");
                }

                std::vector<int> indices = GetIntersection(column, row);

                if (m_visualize)
                {
                    /*Cloud::Ptr subcloud(new Cloud);
                    pcl::copyPointCloud(*treeCloud, indices, *subcloud);
                    subcloud->push_back(point);
                    pcl::visualization::PCLVisualizer v("sub");
                    v.addPointCloud<PointType>(cloud);
                    v.addPointCloud<PointType>(subcloud, "1");
                    v.addPointCloud<PointType>(treeCloud, "2");
                    v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "1");
                    v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "1");
                    v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "2");
                    v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "2");
                    v.spin();*/
                }

                if (indices.size() > 0)
                {
                    std::vector<float> distances;
                    for (int i = 0; i < indices.size(); i += 1)
                    {
                        distances.push_back((point.getVector3fMap() - cloud->points[indices[i]].getVector3fMap()).norm());
                    }

                    PointType interpolated;
                    if (indices.size() == 1)
                    {
                        interpolated = cloud->points[indices[0]];
                    }
                    else
                    {
                        interpolated = Interpolate(cloud, indices, distances);
                    }

                    if (m_featureType == FeatureType::Depth)
                    {
                        float depth = (interpolated.getVector3fMap() - point.getVector3fMap()).norm();

                        float pointProjectionRadius = (std::pow(m_referencePoint.x - point.x, 2) + std::pow(m_referencePoint.z - point.z, 2));
                        float interpolatedProjectionRadius = (std::pow(m_referencePoint.x - interpolated.x, 2) + std::pow(m_referencePoint.z - interpolated.z, 2));

                        if (pointProjectionRadius < interpolatedProjectionRadius)
                        {
                            depth = -depth;
                        }

                        m_result.at<float>(m_featureNumber) = depth;
                    }
                    else if (m_featureType == FeatureType::RGB)
                    {
                        cv::Vec3b color = cv::Vec3b(interpolated.b, interpolated.g, interpolated.r);
                        m_result.at<cv::Vec3b>(m_featureNumber) = color;
                    }
                }

                m_featureNumber += 1;
            }
        private:

            int GetPointColumn(PointType pt)
            {
                float phiStep = (m_phiRange.second - m_phiRange.first) / m_phiSteps;
                float angle = std::atan2(pt.z - m_referencePoint.z, pt.x - m_referencePoint.x);
                int segment = (angle - m_phiRange.first - phiStep / 2) / phiStep;
                if (segment > m_phiSteps - 1 || segment < 0)
                {
                    return -1;
                }
                return segment;
            }

            int GetPointRow(PointType pt)
            {
                float yStep = (m_yRange.second - m_yRange.first) / m_ySteps;
                int row = (pt.y - m_yRange.first - yStep / 2) / yStep;
                if (row > m_ySteps - 1 || row < 0)
                {
                    return -1;
                }
                return row;
            }

            std::vector<int> GetIntersection(int column, int row)
            {
                std::vector<int> result;

                std::set_intersection(m_columns[column].begin(), m_columns[column].end(), m_rows[row].begin(), m_rows[row].end(), std::back_inserter(result));

                return result;
            }

            PointType Interpolate(typename Cloud::Ptr &cloud, std::vector<int> &indices, std::vector<float> &distances)
            {
                float totalDistance = 0;
                for (int i = 0; i < distances.size(); i += 1)
                {
                    totalDistance += distances[i];
                }

                PointType result;
                int n = indices.size();

                for (int i = 0; i < n; i++)
                {
                    PointType p = cloud->at(indices[i]);
                    float currentDistance = distances[i];
                    float c = (totalDistance - currentDistance) / ((n - 1) * totalDistance);
                    result.x += c * p.x;
                    result.y += c * p.y;
                    result.z += c * p.z;
                    result.r += c * (float)p.r;
                    result.g += c * (float)p.g;
                    result.b += c * (float)p.b;
                }
                return result;
            }

            std::vector<std::vector<int>> m_columns;
            std::vector<std::vector<int>> m_rows;

            std::vector<pcl::search::KdTree<PointType>> m_trees;

            pcl::PointXYZ m_referencePoint;

            std::pair<float, float> m_phiRange;
            int m_phiSteps;

            std::pair<float, float> m_yRange;
            int m_ySteps;

            bool m_visualize;
            cv::Mat m_result;
            cv::Size m_featureSize;
            int m_featureNumber;

            FeatureType m_featureType;
    };

}

#endif