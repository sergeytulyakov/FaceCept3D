#pragma once

#ifndef POINTPICKER_H
#define POINTPICKER_H

#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/visualization/keyboard_event.h>

#include <pcl/search/kdtree.h>

#include <boost/format.hpp>

#include <Exception/HPEException.h>

#include <Common.h>

namespace hpe
{
    /**
     \class	PointPicker
    
     \brief	A component that shows a window asking to manually pick a set of points.
			
			To pick a point press SHIFT key and left click on the point, the point index will be 
			displayed.
			
			To delete the last point press DELETE.

			When done press Space
    
     \author	Sergey
     \date	8/11/2015
    
     \tparam	PointType	Type of the point type.
     */

    template <typename PointType>
    class PointPicker
    {
        public:
            typedef typename Common<PointType>::Landmark PickedPoint;
            typedef typename Common<PointType>::Landmarks PickedPoints;

            PointPicker(bool showColor = true, bool allowLessKeypoints = false)
                : m_cloudIsSet(false), m_done(false), m_sphereNumber(0),
                  m_warningId("warning text"), m_pointsListId("m_pointsListId"),
                  m_selectedPointsCloudId("m_selectedPointsCloudId"),
                  m_addedPointList(false), m_addedCloud(false),
                  m_showColor(showColor), m_allowLessKeypoints(allowLessKeypoints)
            {

            }

            PickedPoints Pick(std::string message, int numberOfPoints, std::string extendedMessage="")
            {
                if (m_cloudIsSet == false)
                {
                    throw HPEException("void Pick(std::string &message, int numberOfPoints): Cloud was not set");
                }

                m_numberOfPointsToPick = numberOfPoints;

                pcl::visualization::PCLVisualizer viewer(message);
                m_visualizer = &viewer;

                viewer.registerPointPickingCallback(&PointPicker::PointPickingCallback, this);
                viewer.registerKeyboardCallback(&PointPicker::KeyboardCallback, this);

                viewer.addPointCloud(m_cloud, "cloud");

                if (m_showColor == false)
                {
                    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "cloud");
                }

                viewer.addText("Press:\n SHIFT + Left Click to select\n DELETE to clear\n BACKSPACE to delete last point\n SPACE to finish", 10, 50, 11, 1, 1, 1, "instructions");
                viewer.addText(extendedMessage, 200, 50, 11, 1, 1, 1, "extendedMessage");

                while (viewer.wasStopped() == false && m_done == false)
                {
                    viewer.spinOnce(100);
                    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
                }

                std::vector<pcl::visualization::Camera> cameras;
                viewer.getCameras(cameras);
                if (cameras.size() > 0)
                {
                    m_camera = std::shared_ptr<pcl::visualization::Camera>(new pcl::visualization::Camera);
                    *m_camera = cameras[0];
                }

                m_visualizer->close();

                return m_pickedPoints;
            }

            void SetCloud(const typename pcl::PointCloud<PointType>::Ptr &cloud)
            {
                m_cloud = cloud;
                m_search.setInputCloud(cloud);
                m_cloudIsSet = true;
                m_distanceBetweenPoints = std::abs(cloud->at(0).y - cloud->at(1).y);
                ClearPicker();
            }

            std::shared_ptr<pcl::visualization::Camera> GetCamera() {
                return m_camera;
            }

        private:
            std::shared_ptr<pcl::visualization::Camera> m_camera;

            bool m_showColor;
            bool m_cloudIsSet;
            bool m_done;
            bool m_addedCloud;
            bool m_addedPointList;
            bool m_allowLessKeypoints;
            int m_numberOfPointsToPick;
            float m_distanceBetweenPoints;

            std::string m_warningId;
            std::string m_pointsListId;
            std::string m_selectedPointsCloudId;

            pcl::visualization::PCLVisualizer *m_visualizer;

            typename pcl::PointCloud<PointType>::Ptr m_cloud;
            typename pcl::search::KdTree<PointType> m_search;

            PickedPoints m_pickedPoints;

            int m_sphereNumber;

            void ShowSelectedPoints()
            {
                if (m_addedCloud)
                {
                    m_visualizer->removePointCloud(m_selectedPointsCloudId);
                    m_addedCloud = false;
                }

                if (m_pickedPoints.empty())
                {
                    return;
                }

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                for (int i = 0; i < m_pickedPoints.size(); i++)
                {
                    auto point = m_pickedPoints[i].point;
                    pcl::PointXYZRGB p;
                    p.x = point.x;
                    p.z = point.z;
                    p.y = point.y;
                    p.r = 255;
                    p.g = 255;
                    p.b = 255;
                    cloud->points.push_back(p);
                }

                m_visualizer->addPointCloud(cloud, m_selectedPointsCloudId);
                m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, m_selectedPointsCloudId);
                m_addedCloud = true;
            }

            void RemoveLastPoint()
            {
                if (m_pickedPoints.empty())
                {
                    return;
                }

                m_pickedPoints.pop_back();
            }

            void PrintPointsList()
            {
                if (m_addedPointList)
                {
                    m_visualizer->removeShape(m_pointsListId);
                    m_addedPointList = false;
                }

                if (m_pickedPoints.empty())
                {
                    return;
                }

                std::stringstream stream;
                for (int i = 0; i < m_pickedPoints.size(); i++)
                {
                    auto p = m_pickedPoints[i].point;
                    stream << boost::format("%1% - %5% - [%2%  %3%  %4%]") % (i + 1) % p.x % p.y % p.z % m_pickedPoints[i].index << std::endl;
                }

                m_addedPointList = true;
                auto text = stream.str();
                m_visualizer->addText(text, 10, 150, 10, 1, 1, 1, m_pointsListId);
            }

            void ClearPicker()
            {
                m_pickedPoints.clear();
                m_done = false;
            }

            void ShowWarning()
            {
                std::string warning = (boost::format("You need to pick exactly %1% points") % m_numberOfPointsToPick).str();
                m_visualizer->addText(warning, 10, 100, 10, 1, 0, 0, m_warningId);
            }

            bool CheckNumberOfPoints()
            {
                if (m_pickedPoints.size() >= m_numberOfPointsToPick)
                {
                    ShowWarning();
                    return false;
                }
                return true;
            }

            bool CheckEnoughPointsSelected()
            {
                if (m_allowLessKeypoints)
                {
                    return true;
                }
                if (m_pickedPoints.size() == m_numberOfPointsToPick)
                {
                    return true;
                }
                ShowWarning();
                return false;
            }

            void ClearWarning()
            {
                m_visualizer->removeShape(m_warningId);
            }

            static void PointPickingCallback(const pcl::visualization::PointPickingEvent &event, void *sender)
            {
                PointPicker *_this = (PointPicker *)sender;

                if (_this->CheckNumberOfPoints() == false)
                {
                    return;
                }

                float x, y, z;
                event.getPoint(x, y, z);

                PointType p;
                p.x = x;
                p.y = y;
                p.z = z;

                std::vector<int> foundIndices;
                std::vector<float> distances;
                _this->m_search.nearestKSearch(p, 1, foundIndices, distances);

                PickedPoint pickedPoint;
                pickedPoint.index = foundIndices.at(0);
                pickedPoint.point = _this->m_cloud->at(pickedPoint.index);
                _this->m_pickedPoints.push_back(pickedPoint);

                _this->ShowSelectedPoints();
                _this->PrintPointsList();
            }

            static void KeyboardCallback(const pcl::visualization::KeyboardEvent &event, void *sender)
            {
                if (event.keyDown() == false)
                {
                    return;
                }

                PointPicker *_this = (PointPicker *)sender;

                _this->ClearWarning();

                auto keysym = event.getKeySym();

                if (keysym == "space")
                {
                    if (_this->CheckEnoughPointsSelected())
                    {
                        _this->m_done = true;
                    }
                }
                if (keysym == "Delete")
                {
                    //                _this->ClearPicker();

                    //                _this->PrintPointsList();
                    //                _this->ShowSelectedPoints();
                }
                if (keysym == "BackSpace")
                {
                    _this->RemoveLastPoint();

                    _this->PrintPointsList();
                    _this->ShowSelectedPoints();
                }
            }


    };

}

#endif // POINTPICKER_H
