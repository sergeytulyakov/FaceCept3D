#include "GrabberBase.h"

#include <DataObject/RawFrames.h>
#include <pcl/common/time.h>

namespace hpe
{

    GrabberBase::GrabberBase()
    {
    }

    void GrabberBase::Start()
    {
        PreRun();

        for (auto p = m_processors.begin(); p != m_processors.end(); p++)
        {
            IProcessor::Ptr processor = *p;
            processor->Init();
        }

        m_run = true;
        while (m_run)
        {
            //TODO make thread safe
            for (auto p = m_processorsToAdd.begin(); p != m_processorsToAdd.end(); p++)
            {
                IProcessor::Ptr processor = *p;
                processor->Init();
                m_processors.push_back(processor);
            }
            m_processorsToAdd.clear();

            RawFrames::Ptr rawFrames(new RawFrames);
            bool success = GetNextFrame(rawFrames->colorFrame, rawFrames->depthFrame);
            if (success == false)
            {
                break;
            }

            IDataStorage::Ptr storage = CreateDataStorage();
            storage->Set("RawFrames", rawFrames);

            for (auto p = m_processors.begin(); p != m_processors.end(); p++)
            {
                IProcessor::Ptr processor = *p;
                processor->Process(storage);
            }
        }

        for (auto p = m_processors.begin(); p != m_processors.end(); p++)
        {
            IProcessor::Ptr processor = *p;
            processor->Cleanup();
        }
    }

    void GrabberBase::Stop()
    {
        m_run = false;
    }

    void GrabberBase::AddProcessor(IProcessor::Ptr processor)
    {
        m_processorsToAdd.push_back(processor);
    }

}