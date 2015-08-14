#ifndef LANDMARKS_H
#define LANDMARKS_H

#include <fstream>
#include <ostream>
#include <istream>
#include <string>
#include <algorithm>

#include <boost/algorithm/string/split.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/assign.hpp>
#include <Eigen/Eigen>

#include <Common.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

namespace hpe
{
    using namespace boost::assign;

    template <typename PointType>
    void SaveLandmarks(typename Common<PointType>::Landmarks landmarks, std::string filename)
    {
        std::ofstream stream(filename);

        std::for_each(landmarks.begin(), landmarks.end(), [&](typename Common<PointType>::Landmarks::value_type l)
        {
            stream << l.index << " " << l.point.x << " " << l.point.y << " " << l.point.z << std::endl;
        });
    }

    template <typename PointType>
    typename Common<PointType>::Landmarks LoadLandmarks(std::string filename)
    {
        typename Common<PointType>::Landmarks landmarks;
        std::ifstream istream(filename.c_str());

        std::string line;
        while (std::getline(istream, line))
        {
            std::vector<std::string> values;
            boost::split(values, line, boost::is_any_of(" \t"), boost::token_compress_on);
            typename Common<PointType>::Landmark l;
            if (values[0] != "n")
            {
                l.index = boost::lexical_cast<int>(values[0]);
                l.point.x = boost::lexical_cast<float>(values[1]);
                l.point.y = boost::lexical_cast<float>(values[2]);
                l.point.z = boost::lexical_cast<float>(values[3]);
                landmarks.push_back(l);
            }
        }
        return landmarks;
    }

    template <typename PointType>
    typename Common<PointType>::Landmarks ResampleLandmarks(typename Common<PointType>::Landmarks &oldLandmarks,
            typename pcl::PointCloud<PointType>::Ptr &cloudToResample)
    {
        typename Common<PointType>::Landmarks newLandmarks;
        for (int i = 0; i < oldLandmarks.size(); i++)
        {
            typename Common<PointType>::Landmark l;
            auto oldLandmark = oldLandmarks.at(i);
            l.index = oldLandmark.index;
            l.point = cloudToResample->at(l.index);
            newLandmarks.push_back(l);
        }

        return newLandmarks;
    }

    template<typename PointType>
    typename Common<PointType>::Landmarks ResampleLandmarksByPosition(typename Common<PointType>::Landmarks &oldLandmarks, typename pcl::PointCloud<PointType>::Ptr &cloudToResample)
    {
        pcl::search::KdTree<PointType> tree;
        tree.setInputCloud(cloudToResample);

        typename Common<PointType>::Landmarks newLandmarks;

        for (int i = 0; i < oldLandmarks.size(); i += 1)
        {
            std::vector<int> indices;
            std::vector<float> distances;
            tree.nearestKSearch(oldLandmarks[i].point, 1, indices, distances);

            typename Common<PointType>::Landmark newLandmark;
            newLandmark.index = indices[0];
            newLandmark.point = cloudToResample->points[newLandmark.index];

            newLandmarks.push_back(newLandmark);
        }

        return newLandmarks;
    }


    template <typename PointType>
    PointType MeanLandmark(typename Common<PointType>::Landmarks &landmarks)
    {
        PointType res;

        for (int i = 0; i < landmarks.size(); i++)
        {
            res.getVector3fMap() += landmarks[i].point.getVector3fMap();
        }

        res.getVector3fMap() /= landmarks.size();

        return res;
    }

    template <typename PointType>
    typename Common<PointType>::Landmarks TransformLandmarks(typename Common<PointType>::Landmarks &oldLandmarks, Eigen::Matrix4f transform)
    {
        typename pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        for (auto it = oldLandmarks.begin(); it != oldLandmarks.end(); it++)
        {
            cloud->push_back(it->point);
        }

        pcl::transformPointCloud(*cloud, *cloud, transform);

        typename Common<PointType>::Landmarks result;
        for (int i = 0; i < oldLandmarks.size(); i += 1)
        {
            typename Common<PointType>::Landmark l;
            l.index = oldLandmarks[i].index;
            l.point = cloud->points[i];
            result.push_back(l);
        }

        return result;
    }

}

#endif // LANDMARKS_H
