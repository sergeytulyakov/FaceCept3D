#include "TemplateCreatorProcessor.h"

#include <DataObject/CloudXYZRGBA.h>

namespace hpe
{

    TemplateCreatorProcessor::TemplateCreatorProcessor(void)
        : m_cloudKey("FilteredCloud"), m_templateKey("Template")
    {
    }

    TemplateCreatorProcessor::~TemplateCreatorProcessor(void)
    {
    }

    void TemplateCreatorProcessor::Init()
    {

    }

    void TemplateCreatorProcessor::Process(IDataStorage::Ptr dataStorage)
    {
        CloudXYZRGBA::Ptr cloudObject = dataStorage->GetAndCast<CloudXYZRGBA>(m_cloudKey);

        if (cloudObject.get() != nullptr)
        {
            CloudXYZRGBA::Ptr templateObject(new CloudXYZRGBA);
            templateObject->cloud = m_templateCreator.AddCloudToTemplate(cloudObject->cloud);
            dataStorage->Set(m_templateKey, templateObject);
        }
    }

    IncrementalHeadTemplateCreator::CloudType::Ptr TemplateCreatorProcessor::GetTemplate()
    {
        return m_templateCreator.GetTemplate();
    }

}