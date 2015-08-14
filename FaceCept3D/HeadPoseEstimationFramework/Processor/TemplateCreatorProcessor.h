#pragma once

#ifndef TEMPLATECREATORPROCESSOR_H
#define TEMPLATECREATORPROCESSOR_H

#include <Processor/IProcessor.h>
#include <Align/IncrementalHeadTemplateCreator.h>


namespace hpe
{
    /**
     \class	TemplateCreatorProcessor
    
     \brief	A head template is created here. The process uses m_cloudKey as an input,
			and iteratively registers consequtive point clouds by using IncrementalHeadTemplateCreator.
    
     \author	Sergey
     \date	8/11/2015
     */

    class TemplateCreatorProcessor : public IProcessor
    {
        public:
            TemplateCreatorProcessor(void);
            ~TemplateCreatorProcessor(void);

            void Init() override;
            void Process(IDataStorage::Ptr dataStorage) override;

            IncrementalHeadTemplateCreator::CloudType::Ptr GetTemplate();

        private:
            std::string m_cloudKey;
            std::string m_templateKey;

            IncrementalHeadTemplateCreator m_templateCreator;
    };

}

#endif