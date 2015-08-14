#include "ConverterProcessor.h"
#include <Exception/HPEException.h>
#include <DataObject/RawFrames.h>
#include <DataObject/CloudXYZRGBA.h>

namespace hpe
{

    ConverterProcessor::ConverterProcessor(void)
        : m_cloudKey("Cloud"), m_rawFramesKey("RawFrames")
    {
    }

    ConverterProcessor::ConverterProcessor(ConverterPtr converter)
        : m_cloudKey("Cloud"), m_rawFramesKey("RawFrames"), m_converter(converter)
    {
    }

    ConverterProcessor::~ConverterProcessor(void)
    {
    }

    void ConverterProcessor::Init()
    {
        if (m_converter.get() == nullptr)
        {
            throw HPEException("ConverterProcessor::Init - m_converter.get() == nullptr");
        }
    }

    void ConverterProcessor::SetCloudKey(std::string key)
    {
        m_cloudKey = key;
    }

    void ConverterProcessor::SetRawFramesKey(std::string key)
    {
        m_rawFramesKey = key;
    }

    void ConverterProcessor::SetConverter(ConverterPtr converter)
    {
        m_converter = converter;
    }

    void ConverterProcessor::Process(IDataStorage::Ptr dataStorage)
    {
        IDataObject::Ptr object = dataStorage->Get(m_rawFramesKey);
        RawFrames::Ptr rawFrames = std::dynamic_pointer_cast<RawFrames>(object);
        if (rawFrames.get() == nullptr)
        {
            throw HPEException("ConverterProcessor::Process - rawFrames.get() == nullptr");
        }

        CloudXYZRGBA::Ptr cloudObject(new CloudXYZRGBA);
        cloudObject->cloud = m_converter->DepthRGBToPointCloud(rawFrames->depthFrame, rawFrames->colorFrame);

        dataStorage->Set(m_cloudKey, cloudObject);
    }

}