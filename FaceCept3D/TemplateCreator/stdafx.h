// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#undef _CRT_SECURE_NO_WARNINGS

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/time.h>

#include <Settings/HPESettings.h>

#ifndef __linux__
    #include <WindowsOnly/Grabber/KinectSDKGrabber.h>
    #include <WindowsOnly/Processor/KinectSDKConverterProcessor.h>
#endif


#include <Processor/ConverterProcessor.h>
#include <Processor/DepthPreprocessingProcessor.h>
#include <Processor/HeadExtractionProcessor.h>
#include <Processor/TemplateCreatorProcessor.h>
#include <Processor/ShowCloudProcessor.h>

#include <Converter/KinectDataConverter.h>

