#include "stdafx.h"
#include "VoxelizeProcessor.h"

#include <Filter/Filters.h>
#include <DataObject/CloudXYZRGBA.h>

VoxelizeProcessor::VoxelizeProcessor(float size)
    : m_size(size)
{
}


VoxelizeProcessor::~VoxelizeProcessor(void)
{
}

void VoxelizeProcessor::Process(hpe::IDataStorage::Ptr dataStorage)
{
    using namespace hpe;
    CloudXYZRGBA::Ptr cloudObject = dataStorage->GetAndCastNotNull<CloudXYZRGBA>("Cloud", "VoxelizeProcessor::Process - Cloud is null");
    cloudObject->cloud = PassThrough<pcl::PointXYZRGBA>(cloudObject->cloud, 0, 1.5, "z", false);

    cloudObject->cloud = Voxelize<pcl::PointXYZRGBA>(cloudObject->cloud, m_size);
}
