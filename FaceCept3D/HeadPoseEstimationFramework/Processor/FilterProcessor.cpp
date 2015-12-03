
#include <Processor/FilterProcessor.h>
#include <DataObject/CloudXYZRGBA.h>
#include <DataObject/IDataStorage.h>

namespace hpe
{
    FilterProcessor::FilterProcessor(FilterPtr filter, std::string inputKey, std::string outputKey)
        : m_filter(filter), m_inputKey(inputKey), m_outputKey(outputKey)
    {

    }

    void FilterProcessor::Process(IDataStorage::Ptr dataStorage)
    {
        CloudXYZRGBA::Ptr cloudObject = dataStorage->GetAndCastNotNull<CloudXYZRGBA>(m_inputKey);
        CloudXYZRGBA::Ptr filteredCloudObject(new CloudXYZRGBA);
        filteredCloudObject->cloud = m_filter->Filter(cloudObject->cloud);

        dataStorage->Set(m_outputKey, filteredCloudObject);
    }
}
