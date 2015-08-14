#include "ProviderGrabber.h"

#include <Exception/HPEException.h>
#include <DataObject/MapDataStorage.h>

namespace hpe
{

    ProviderGrabber::ProviderGrabber(void)
        : m_currentFrame(0)
    {
    }


    ProviderGrabber::~ProviderGrabber(void)
    {
    }

    bool ProviderGrabber::GetNextFrame(cv::Mat &colorFrame, cv::Mat &depthFrame)
    {
        if (m_currentFrame >= m_provider->GetFrameCount())
        {
            return false;
        }

        colorFrame = m_provider->GetBgr(m_currentFrame);
        depthFrame = m_provider->GetDepth(m_currentFrame);
        m_currentFrame += 1;

        return true;
    }

    IDataStorage::Ptr ProviderGrabber::CreateDataStorage()
    {
        IDataStorage::Ptr result(new MapDataStorage);
        return result;
    }

    void ProviderGrabber::SetProvider(std::shared_ptr<ProviderType> provider)
    {
        m_provider = provider;
    }

    void ProviderGrabber::PreRun()
    {
        if (m_provider.get() == nullptr)
        {
            throw HPEException("ProviderGrabber::PreRun - m_provider == nullptr");
        }
    }

}