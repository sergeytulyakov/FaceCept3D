#include "stdafx.h"
#include "UIProcessor.h"

UIProcessor::UIProcessor(void)
    : m_grabber(nullptr), m_templateCreationTriggered(false)
{
}

UIProcessor::UIProcessor(std::string key)
    : ShowCloudProcessor(key), m_grabber(nullptr), m_templateCreationTriggered(false)
{
}


UIProcessor::~UIProcessor(void)
{
}

void UIProcessor::KeyPressed(const pcl::visualization::KeyboardEvent &event)
{
    if (event.keyDown())
    {
        std::string sym = event.getKeySym();
        std::cout << sym << std::endl;
        if (sym == "Escape")
        {
            if (m_grabber)
            {
                m_grabber->Stop();
            }
        }
    }
}

void UIProcessor::SetGrabber(hpe::GrabberBase *grabber)
{
    m_grabber = grabber;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr UIProcessor::GetTemplate()
{
    return m_template;
}

hpe::Common<pcl::PointXYZRGBA>::Landmarks UIProcessor::GetLandmarks()
{
    return m_landmarks;
}
