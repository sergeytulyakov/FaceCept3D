#include "stdafx.h"
#include "ShowTwoCloudsProcessor.h"

#include <DataObject/CloudXYZRGBA.h>
#include <Grabber/PCDGrabber.h>

using namespace hpe;

ShowTwoCloudsProcessor::ShowTwoCloudsProcessor(void)
    : m_cloud1Key("Cloud1"), m_cloud2Key("Cloud2"), m_first(true), m_visualizer("Two Clouds")
{
}

ShowTwoCloudsProcessor::~ShowTwoCloudsProcessor(void)
{
}

void ShowTwoCloudsProcessor::SetCloud1Key(std::string key)
{
    m_cloud1Key = key;
}

void ShowTwoCloudsProcessor::SetCloud2Key(std::string key)
{
    m_cloud2Key = key;
}

void ShowTwoCloudsProcessor::Process(hpe::IDataStorage::Ptr dataStorage)
{
    CloudXYZRGBA::Ptr cloud1 = dataStorage->GetAndCast<CloudXYZRGBA>(m_cloud1Key);
    CloudXYZRGBA::Ptr cloud2 = dataStorage->GetAndCast<CloudXYZRGBA>(m_cloud2Key);
    if (cloud1.get() == nullptr || cloud2.get() == nullptr)
    {
        return;
    }

    if (m_first)
    {
        m_first = false;
        m_visualizer.addPointCloud(cloud1->cloud, m_cloud1Key);
        m_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, m_cloud1Key);
        m_visualizer.addPointCloud(cloud2->cloud, m_cloud2Key);
        m_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, m_cloud2Key);
    }
    else
    {
        m_visualizer.updatePointCloud(cloud1->cloud, m_cloud1Key);
        m_visualizer.updatePointCloud(cloud2->cloud, m_cloud2Key);
    }

    m_visualizer.spinOnce(1);

    PCDGrabber::CloudFileInfo::Ptr fileInfo = dataStorage->GetAndCast<PCDGrabber::CloudFileInfo>("FileInfo");
    if (fileInfo.get() != nullptr)
    {
        std::string screenshotPath = boost::replace_all_copy(fileInfo->Filename, ".pcd", ".png");
        //m_visualizer.saveScreenshot(screenshotPath);
    }
}
