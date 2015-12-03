#pragma once

#ifndef HPEHELPERS_H
#define HPEHELPERS_H

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <opencv2/opencv.hpp>

namespace hpe
{
    Eigen::Vector3f VectorToEulerAngles(Eigen::Vector3f v);

    Eigen::Vector3f EulerAnglesToVector(double yaw, double tilt, bool inDegrees = true);

    template <typename PointType>
    Eigen::Vector3f GetNormal(typename pcl::PointCloud<PointType>::Ptr cloud, int noseInCloud)
    {
        Eigen::Vector3f result;

        pcl::search::KdTree<PointType>::Ptr search(new pcl::search::KdTree<PointType>);

        pcl::IndicesPtr idx(new std::vector<int>);
        idx->push_back(noseInCloud);

        pcl::NormalEstimation<PointType, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        ne.setIndices(idx);
        ne.setKSearch(50);
        ne.setSearchMethod(search);

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
        ne.compute(*normals);
        result = normals->points[0].getNormalVector3fMap();

        return result;
    }

    cv::Point2f MeanPoint(std::vector<cv::Point2f> points);
}

#endif