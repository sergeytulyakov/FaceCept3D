#include "stdafx.h"
#include "UIProcessor.h"

#include <DataObject/RawFrames.h>
#include <opencv2/opencv.hpp>

UIProcessor::UIProcessor(void)
    : m_grabber(nullptr), m_templateCreationTriggered(false)
{
}

UIProcessor::UIProcessor(std::string key)
    : m_grabber(nullptr), m_templateCreationTriggered(false)
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

void UIProcessor::Process(hpe::IDataStorage::Ptr dataStorage)
{
    using namespace hpe;
    RawFrames::Ptr frames = dataStorage->GetAndCastNotNull<RawFrames>("RawFrames");

    cv::imshow("color", frames->colorFrame);
    cv::imshow("depth", frames->depthFrame);

    int key = cv::waitKey(1);
    if (key != -1)
    {
        m_grabber->Stop();
    }
}

