// TemplateTracker.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "ShowTwoCloudsProcessor.h"
#include "VoxelizeProcessor.h"
#include "UIProcessor.h"
#include "VisuilizeProcessor.h"

#include <UI/PointPicker.h>
#include <Common.h>
#include <Landmarks.h>
#include <Grabber/ProviderGrabber.h>

#include <Filter/Filters.h>
#include <Filter/FunctorFilter.h>

#include <Processor/ConverterProcessor.h>
#include <Processor/DetectorProcessor.h>
#include <Processor/FilterProcessor.h>

#include <boost/bind.hpp>


using namespace hpe;

int main(int argc, char *argv[])
{
    HPESettings settings;

    KinectSDKGrabber grabber;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr templateCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile(settings.GetString("Template"), *templateCloud);

    std::shared_ptr<UIProcessor> uiProcessor(new UIProcessor);
    uiProcessor->SetGrabber(&grabber);

    std::shared_ptr<DepthPreprocessingProcessor> preprocessor(new DepthPreprocessingProcessor(1, 3, 13, 7, 1.2));

    std::shared_ptr<hpe::TrackingProcessor> templateTracker(new hpe::TrackingProcessor(templateCloud));
    templateTracker->SetTemplateLandmarks(LoadLandmarks<pcl::PointXYZRGBA>(settings.GetString("Landmarks")));
    templateTracker->SetSaveLandmakrs(false);
    

    std::string cloudToShowKey = "FilteredOriginalCloud";
    std::shared_ptr<VisuilizeProcessor> visualizer(new VisuilizeProcessor(cloudToShowKey));
    visualizer->SetTrackingProcessor(templateTracker);

    std::shared_ptr<hpe::FacialExpressionProcessor> ferProcessor(new hpe::FacialExpressionProcessor(settings.GetString("FERData")));
    ferProcessor->SubscribeFacialExpressionReadySignal(boost::bind(&VisuilizeProcessor::HandleFER, visualizer.get(), _1));


    std::shared_ptr<FunctorFilter<pcl::PointXYZRGBA>> functorFilter(new FunctorFilter<pcl::PointXYZRGBA>(
    [&settings](pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)->pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  {
        return PassThrough<pcl::PointXYZRGBA>(cloud, 0., settings.GetValue<float>("DistanceToShow"), "z");
    }));

    std::shared_ptr<FilterProcessor> filterProcessor(new FilterProcessor(functorFilter, "OriginalCloud", "FilteredOriginalCloud"));

    grabber.AddProcessor(IProcessor::Ptr(new KinectSDKConverterProcessor("OriginalCloud")));
    grabber.AddProcessor(filterProcessor);
    grabber.AddProcessor(IProcessor::Ptr(new DetectorProcessor(settings.GetString("DetectorData"))));
    grabber.AddProcessor(preprocessor);
    grabber.AddProcessor(IProcessor::Ptr(new KinectSDKConverterProcessor("Cloud")));
    grabber.AddProcessor(IProcessor::Ptr(new VoxelizeProcessor(0.005)));
    grabber.AddProcessor(templateTracker);
    grabber.AddProcessor(ferProcessor);
    grabber.AddProcessor(visualizer);

    grabber.Start();

    return 0;
}

