#pragma once

#include <Processor/IProcessor.h>

class VoxelizeProcessor : public hpe::IProcessor
{
    public:
        VoxelizeProcessor(float size = 0.005f);
        ~VoxelizeProcessor(void);

        void Process(hpe::IDataStorage::Ptr dataStorage);
    private:
        float m_size;
};

