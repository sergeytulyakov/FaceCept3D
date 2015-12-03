// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include <stdio.h>

#include <limits>

#include <Settings/HPESettings.h>

#include <Grabber/PCDGrabber.h>
#include <Processor/DepthPreprocessingProcessor.h>
#include <Processor/TrackingProcessor.h>
#include <Processor/ShowCloudProcessor.h>
#include <Processor/SaveCloudProcessor.h>
#include <Processor/FacialExpressionsProcessor.h>

#include <pcl/io/pcd_io.h>

#ifndef __linux__
	#include <objbase.h>
	#include <WindowsOnly/Grabber/KinectSDKGrabber.h>
	#include <WindowsOnly/Processor/KinectSDKConverterProcessor.h>
#else
	#include <Grabber/OpenNIGrabber.h>
#endif