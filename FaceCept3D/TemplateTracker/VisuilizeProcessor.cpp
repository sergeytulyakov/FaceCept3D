#include "stdafx.h"
#include "VisuilizeProcessor.h"

#include <DataObject/LandmarksObject.h>
#include <DataObject/CloudXYZRGBA.h>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <math.h>
#include <fstream>

using namespace hpe;

VisuilizeProcessor::VisuilizeProcessor(std::string cloudKey)
    : m_cloudKey(cloudKey), m_landmarksKey("Landmarks"), m_visualizer("VisualizerProcessor"), m_first(true), m_saveScreenshots(false), m_haveFerData(false)
{
    InitExpressionLabels();
}

VisuilizeProcessor::VisuilizeProcessor(bool saveScreenshots)
    : m_cloudKey("Cloud"), m_landmarksKey("Landmarks"), m_visualizer("VisualizerProcessor"), m_first(true), m_saveScreenshots(saveScreenshots), m_haveFerData(false)
{
    InitExpressionLabels();
}

VisuilizeProcessor::~VisuilizeProcessor(void)
{
}

void VisuilizeProcessor::Process(hpe::IDataStorage::Ptr dataStorage)
{
    CloudXYZRGBA::Ptr cloud = dataStorage->GetAndCast<CloudXYZRGBA>(m_cloudKey);
    LandmarksObject<pcl::PointXYZRGBA>::Ptr landmarks = dataStorage->GetAndCast<LandmarksObject<pcl::PointXYZRGBA>>(m_landmarksKey);
    if (cloud.get() == nullptr || landmarks.get() == nullptr)
    {
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr eyes(new pcl::PointCloud<pcl::PointXYZRGBA>);
    eyes->push_back(landmarks->landmarks[0].point);
    eyes->push_back(landmarks->landmarks[1].point);

    pcl::ModelCoefficients cylinderCoefficients;
    cylinderCoefficients.values.push_back(landmarks->landmarks[2].point.x);
    cylinderCoefficients.values.push_back(landmarks->landmarks[2].point.y);
    cylinderCoefficients.values.push_back(landmarks->landmarks[2].point.z);
    cylinderCoefficients.values.push_back((landmarks->landmarks[3].point.x - landmarks->landmarks[2].point.x) / 10);
    cylinderCoefficients.values.push_back((landmarks->landmarks[3].point.y - landmarks->landmarks[2].point.y) / 10);
    cylinderCoefficients.values.push_back((landmarks->landmarks[3].point.z - landmarks->landmarks[2].point.z) / 10);
    cylinderCoefficients.values.push_back(0.002);

    Eigen::Vector3f angles = VectorToEulerAngles(landmarks->landmarks[2].point.getVector3fMap() - landmarks->landmarks[3].point.getVector3fMap());
    std::string anglesText = (boost::format("Head pose in degrees:\n Yaw   %5.2f\n Tilt  %5.2f\n Roll  %5.2f") % angles(0) % angles(1) % angles(2)).str();

    if (m_first)
    {
        m_first = false;

        m_visualizer.registerKeyboardCallback(&VisuilizeProcessor::KeyboardEventCallback, this);

        m_visualizer.addCylinder(cylinderCoefficients);
        m_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, ((float)0x73) / 255, ((float)0x3C) / 255, "cylinder");
        m_visualizer.addPointCloud(cloud->cloud, m_cloudKey);
        m_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, m_cloudKey);
        m_visualizer.addPointCloud(eyes, m_landmarksKey);
        m_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 25, 0, 0, m_landmarksKey);
        m_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, m_landmarksKey);
        m_visualizer.addText(anglesText, 25, 175, 17, 0, 0, 1, "anglesText");
		m_visualizer.addText("Facial expression:", 25, 140, 17, 0, 0, 1, "anglesTextCaption");
		m_visualizer.setBackgroundColor(1, 1, 1);
        if (m_saveScreenshots)
        {
            if (boost::filesystem::exists("camera.cam"))
            {
                pcl::visualization::Camera camera;
                LoadCamera(camera, "camera.cam");
                m_visualizer.setCameraParameters(camera);
            }
            else
            {
                while (m_visualizer.wasStopped() == false)
                {
                    m_visualizer.spinOnce(100);
                }
                std::vector<pcl::visualization::Camera> cameras;
                m_visualizer.getCameras(cameras);
                pcl::visualization::Camera camera = cameras[0];
                SaveCamera(camera, "camera.cam");
            }
        }
    }
    else
    {
        m_visualizer.removeShape("cylinder");
        m_visualizer.addCylinder(cylinderCoefficients);
        m_visualizer.removeShape("anglesText");
        m_visualizer.addText(anglesText, 25, 175, 17, 0, 0, 1, "anglesText");
        m_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, ((float)0x73) / 255, ((float)0x3C) / 255, "cylinder");
        m_visualizer.updatePointCloud(cloud->cloud, m_cloudKey);
        m_visualizer.updatePointCloud(eyes, m_landmarksKey);

        if (m_haveFerData)
        {
            int delta = 19;
			int x = 20;
			int y = 140 - delta;

            auto recognizedExpression = std::max_element(m_ferData.begin(), m_ferData.end());
            int recognizedExpressionIndex = std::distance(m_ferData.begin(), recognizedExpression);

            for (int i = 0; i < m_ferData.size(); i++)
            {
                double colorScale = i == recognizedExpressionIndex ? 2 : 1;
                std::string message = (boost::format("  %s %f") % m_expressionLabels[i] % m_ferData[i]).str();
                std::string textObjectLabel = (boost::format("ferText%1%") % i).str();
                m_visualizer.removeShape(textObjectLabel);
				m_visualizer.addText(message, x, y, 17, (2 - colorScale) * 0.5, (2 - colorScale) * 0.5, colorScale * 0.5, textObjectLabel);
                y -= delta;
            }
        }
    }

    m_visualizer.spinOnce(1);

    if (m_saveScreenshots)
    {
        PCDGrabber::CloudFileInfo::Ptr fileInfo = dataStorage->GetAndCast<PCDGrabber::CloudFileInfo>("FileInfo");
        if (fileInfo.get() != nullptr)
        {
            std::string screenshotPath = boost::replace_all_copy(fileInfo->Filename, ".pcd", ".png");
            m_visualizer.saveScreenshot(screenshotPath);
        }
    }
}

Eigen::Vector3f VisuilizeProcessor::VectorToEulerAngles(Eigen::Vector3f v)
{
    Eigen::Vector3f normed = v / v.norm();
    float x = normed(0);
    float y = normed(1);
    float z = normed(2);

    Eigen::Vector3f result;
    result(0) = std::atan2(z, x) - M_PI / 2; // yaw
    result(1) = -std::atan2(y, z); // tilt
    result(2) = std::atan(y / x); // roll

    return result * 180 / M_PI;

}

void VisuilizeProcessor::SaveCamera(pcl::visualization::Camera &camera, std::string file)
{
    std::ofstream str(file);

    str << camera.clip[0] << std::endl;
    str << camera.clip[1] << std::endl;
    str << camera.focal[0] << std::endl;
    str << camera.focal[1] << std::endl;
    str << camera.focal[2] << std::endl;
    str << camera.fovy << std::endl;
    str << camera.pos[0] << std::endl;
    str << camera.pos[1] << std::endl;
    str << camera.pos[2] << std::endl;
    str << camera.view[0] << std::endl;
    str << camera.view[1] << std::endl;
    str << camera.view[2] << std::endl;
    str << camera.window_pos[0] << std::endl;
    str << camera.window_pos[1] << std::endl;
    str << camera.window_size[0] << std::endl;
    str << camera.window_size[1] << std::endl;
}

void VisuilizeProcessor::LoadCamera(pcl::visualization::Camera &camera, std::string file)
{
    std::ifstream str(file);

    str >> camera.clip[0];
    str >> camera.clip[1];
    str >> camera.focal[0];
    str >> camera.focal[1];
    str >> camera.focal[2];
    str >> camera.fovy;
    str >> camera.pos[0];
    str >> camera.pos[1];
    str >> camera.pos[2];
    str >> camera.view[0];
    str >> camera.view[1];
    str >> camera.view[2];
    str >> camera.window_pos[0];
    str >> camera.window_pos[1];
    str >> camera.window_size[0];
    str >> camera.window_size[1];
}

void VisuilizeProcessor::HandleFER(std::vector<double> ferdata)
{
    m_haveFerData = true;
    m_ferData = ferdata;
}

void VisuilizeProcessor::InitExpressionLabels()
{
    m_expressionLabels.clear();
    m_expressionLabels.push_back("Neutral");
    m_expressionLabels.push_back("Happy");
    m_expressionLabels.push_back("Surprise");
}

void VisuilizeProcessor::KeyboardEventCallback(const pcl::visualization::KeyboardEvent &event, void *sender)
{
    if (event.getKeySym() == "space")
    {
        VisuilizeProcessor *_this = static_cast<VisuilizeProcessor *>(sender);
        if (_this->m_trackingProcessor.get() != nullptr)
        {
            _this->m_trackingProcessor->ResetTracker();
        }
    }
}
