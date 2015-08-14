#ifndef FILTERS_H
#define FILTERS_H

#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>

#include <pcl/surface/mls.h>
#include <Exception/HPEException.h>

#include "UI/ShowCloud.h"

namespace hpe
{
    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr SortPoints(typename pcl::PointCloud<PointT>::Ptr cloud/*, std::function<PointT(PointT &, PointT &)> comparer*/)
    {
        typename pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*cloud, *result);

        std::sort(result->begin(), result->end(), [](PointT a, PointT b) -> bool
        {
            float value = 100000;
            return a.y * value + a.x > b.y * value + b.x;
        });

        return result;
    }

    template<typename PointT, typename NormalT>
    typename pcl::PointCloud<NormalT>::Ptr ComputeNormals(typename pcl::PointCloud<PointT>::Ptr &cloud, int kSearch = 30, int radiusSearch = 0)
    {
        typename pcl::PointCloud<NormalT>::Ptr result(new pcl::PointCloud<NormalT>);

        typedef pcl::search::KdTree<PointT> Search;
        typename Search::Ptr search(new Search);

        pcl::NormalEstimation<PointT, NormalT> estimation;
        estimation.setSearchMethod(search);
        if (kSearch != 0)
        {
            estimation.setKSearch(kSearch);
        }
        else if (radiusSearch != 0)
        {
            estimation.setRadiusSearch(radiusSearch);
        }
        else
        {
            throw HPEException("ComputeNormals : Specify either kSearch or radiusSearch");
        }

        estimation.setInputCloud(cloud);
        estimation.compute(*result);
        return result;
    }


    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr Voxelize(typename pcl::PointCloud<PointT>::Ptr cloud, float leafSize)
    {
        typename pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>);

        pcl::VoxelGrid<PointT> grid;
        grid.setLeafSize(leafSize, leafSize, leafSize);
        grid.setInputCloud(cloud);
        grid.filter(*result);

        return result;
    }

    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr RemoveOutliersByDistance(typename pcl::PointCloud<PointT>::Ptr cloud,
            int numberOfNeighborsToAnalyze, int stdThreshold)
    {
        typename pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>);

        pcl::StatisticalOutlierRemoval<PointT> outlierRemoval;
        outlierRemoval.setInputCloud(cloud);
        outlierRemoval.setMeanK(numberOfNeighborsToAnalyze);
        outlierRemoval.setStddevMulThresh(stdThreshold);
        outlierRemoval.filter(*result);

        return result;
    }

    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr PassThrough(typename pcl::PointCloud<PointT>::Ptr &cloud,
            float from, float to,
            std::string filterName,
            bool keepOrganzed = true)
    {
        typename pcl::PointCloud<PointT>::Ptr result(new typename pcl::PointCloud<PointT>);

        pcl::PassThrough<PointT> filter;
        filter.setFilterLimits(from, to);
        if (cloud->isOrganized())
        {
            filter.setKeepOrganized(keepOrganzed);
        }
        filter.setFilterFieldName(filterName);
        filter.setInputCloud(cloud);
        filter.filter(*result);

        return result;
    }

    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr ResampleMLS(typename pcl::PointCloud<PointT>::Ptr &cloud, int polynomialOrder = 2, float searchRadius = 0.005)
    {
        typename pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>);

        typename pcl::search::KdTree<PointT>::Ptr search(new pcl::search::KdTree<PointT>);
        pcl::MovingLeastSquares<PointT, PointT> mls;

        mls.setInputCloud(cloud);
        mls.setPolynomialFit(true);
        mls.setPolynomialOrder(polynomialOrder);
        mls.setSearchMethod(search);
        mls.setSearchRadius(searchRadius);

        mls.process(*result);

        return result;
    }

}

#endif // FILTERS_H
