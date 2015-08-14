#ifndef SHOWCLOUD_H
#define SHOWCLOUD_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <boost/format.hpp>

#include <string>
#include <vector>

//#include <Provider/DataProvider.h>
#include <Common.h>

namespace hpe
{
    template<typename PointType>
    typename pcl::PointCloud<PointType>::Ptr MoveCloudTo(typename pcl::PointCloud<PointType>::Ptr &cloud, float x, float y, float z)
    {
        Eigen::Vector4d centroid;
        pcl::compute3DCentroid(*cloud, centroid);

        typename pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>);
        Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
        translation(0, 3) = (x - centroid(0));
        translation(1, 3) = (y - centroid(1));
        translation(2, 3) = (z - centroid(2));

        pcl::demeanPointCloud(*cloud, centroid, *result);

        return result;
    }


    template<typename PointType>
    void ShowTwoClouds(typename pcl::PointCloud<PointType>::Ptr &cloud1,
                       typename pcl::PointCloud<PointType>::Ptr &cloud2,
                       std::string caption = "Two Clouds",
                       int waitMillisecods = 0)
    {
        pcl::visualization::PCLVisualizer viewer(caption);
        viewer.addPointCloud(cloud1, "1");
        viewer.addPointCloud(cloud2, "2");

        if (waitMillisecods == 0)
        {
            while (viewer.wasStopped() == false)
            {
                viewer.spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }
        }
        else
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(waitMillisecods));
        }
    }

    template<typename PointType>
    std::shared_ptr<pcl::visualization::Camera> ShowCloud(typename pcl::PointCloud<PointType>::Ptr &cloud,
            std::string caption = "Cloud",
            std::shared_ptr<pcl::visualization::Camera> camera = nullptr,
            double r = 0, double g = 0, double b = 0)
    {
        pcl::visualization::PCLVisualizer viewer(caption);

        auto t = MoveCloudTo<PointType>(cloud, 0, 0, 0);
        viewer.addPointCloud(t, "1");

        if (camera != nullptr)
        {
            viewer.setCameraParameters(*camera);
        }

        viewer.setBackgroundColor(r, g, b);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "1");

        while (viewer.wasStopped() == false)
        {
            viewer.spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

        std::vector<pcl::visualization::Camera> cameras;
        viewer.getCameras(cameras);

        std::shared_ptr<pcl::visualization::Camera> result(new pcl::visualization::Camera);
        *result = cameras[0];

        return result;
    }

    template<typename PointType>
    void ShowAllClouds(std::vector<typename pcl::PointCloud<PointType>::Ptr> &clouds, std::string caption = "Clouds")
    {
        pcl::visualization::PCLVisualizer viewer(caption);

        auto it = clouds.begin();
        int i = 0;
        for (; it != clouds.end(); ++it)
        {
            std::string cloudName = (boost::format("%1%") % i++).str();
            viewer.addPointCloud(*it, cloudName);
        }

        while (viewer.wasStopped() == false)
        {
            viewer.spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    template<typename PointType>
    void ShowTwoCloudsInDifferentColors(typename pcl::PointCloud<PointType>::Ptr &cloud1,
                                        typename pcl::PointCloud<PointType>::Ptr &cloud2,
                                        pcl::CorrespondencesPtr correspondences,
                                        std::string caption = "Two clouds different color",
                                        int waitMillisecods = 0)
    {
        pcl::visualization::PCLVisualizer viewer(caption);
        viewer.addCoordinateSystem();
        viewer.addPointCloud<PointType>(cloud1, "1");
        viewer.addPointCloud<PointType>(cloud2, "2");

        if (correspondences != nullptr)
        {
            viewer.addCorrespondences<PointType>(cloud1, cloud2, *correspondences);
        }



        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "1");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "2");

        if (waitMillisecods == 0)
        {
            while (viewer.wasStopped() == false)
            {
                viewer.spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }
        }
        else
        {
            viewer.spinOnce(waitMillisecods);
        }

    }

    template<typename PointType>
    void ShowCloudInColor(typename pcl::PointCloud<PointType>::Ptr &cloud,
                          std::shared_ptr<pcl::visualization::Camera> camera = nullptr,
                          double r = 0, double g = 0, double b = 1,
                          double rb = 0, double gb = 0, double bb = 0, double pointSize = 3,
                          std::string caption = "Cloud in color")
    {
        pcl::visualization::PCLVisualizer viewer(caption);
        viewer.addPointCloud(cloud, "1");

        if (camera != nullptr)
        {
            viewer.setCameraParameters(*camera);
        }

        viewer.setBackgroundColor(rb, gb, bb);

        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "1");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "1");


        while (viewer.wasStopped() == false)
        {
            viewer.spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    template<typename PointType>
    void ShowTwoCloudsInDifferentColors(typename pcl::PointCloud<PointType>::Ptr &cloud1,
                                        typename pcl::PointCloud<PointType>::Ptr &cloud2,
                                        std::string caption = "Two clouds different color",
                                        int waitMillisecods = 0)
    {
        ShowTwoCloudsInDifferentColors<PointType>(cloud1, cloud2, pcl::CorrespondencesPtr(), caption, waitMillisecods);
    }

    template<typename PointType>
    void ShowCloudWithLandmarks(typename pcl::PointCloud<PointType>::Ptr &cloud,
                                typename Common<PointType>::Landmarks &landmarks,
                                std::string caption = "Landmarks", bool showLandmarksNumbers = false)
    {
        typename pcl::PointCloud<PointType>::Ptr markerCloud(new pcl::PointCloud<PointType>);

        for (auto it = landmarks.begin(); it != landmarks.end(); ++it)
        {
            markerCloud->push_back((*it).point);
        }

        pcl::visualization::PCLVisualizer viewer(caption);
        viewer.addPointCloud<PointType>(cloud, "Main Cloud");
        viewer.addPointCloud<PointType>(markerCloud, "Marker Cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Marker Cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "Marker Cloud");

        if (showLandmarksNumbers)
        {
            for (int i = 0; i < markerCloud->size(); i++)
            {
                std::string text = (boost::format("%1%") % i).str();
                viewer.addText3D(text, markerCloud->at(i), 2, 1, 1, 1, text);
            }
        }

        while (viewer.wasStopped() == false)
        {
            viewer.spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }
}

#endif // SHOWCLOUD_H
