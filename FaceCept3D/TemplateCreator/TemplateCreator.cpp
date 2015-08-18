// TemplateCreator.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <Filter/MovingLeastSquaresFilter.h>
#include <Filter/Filters.h>

#include <UI/ShowCloud.h>
#include <UI/PointPicker.h>

#include <Landmarks.h>
#include <Exception/HPEException.h>

#include "UIProcessor.h"

using namespace hpe;
typedef pcl::PointXYZRGBA PointType;


int main(int argc, char *argv[])
{
    HPESettings settings;

    try
    {
#ifdef __linux__
        OpenNIGrabber grabber;
#else
        KinectSDKGrabber grabber;
#endif

        std::shared_ptr<hpe::TemplateCreatorProcessor> templateCreator(new TemplateCreatorProcessor);
        std::shared_ptr<UIProcessor> uiProcessor(new UIProcessor("Cloud"));
        uiProcessor->SetGrabber(&grabber);

        grabber.AddProcessor(IProcessor::Ptr(new DepthPreprocessingProcessor(1, 3, 5, 5, 1.2)));

#ifndef __linux__
        grabber.AddProcessor(IProcessor::Ptr(new KinectSDKConverterProcessor));
#endif

        grabber.AddProcessor(IProcessor::Ptr(new ConverterProcessor(ConverterProcessor::ConverterPtr(new KinectDataConverter))));

        grabber.AddProcessor(IProcessor::Ptr(new HeadExtractionProcessor));
        grabber.AddProcessor(std::static_pointer_cast<IProcessor>(templateCreator));
        grabber.AddProcessor(std::static_pointer_cast<IProcessor>(uiProcessor));


        grabber.Start();

        auto t = templateCreator->GetTemplate();

        t = Voxelize<pcl::PointXYZRGBA>(t, 0.0025);

        if (t->size() != 0)
        {
            MovingLeastSquaresFilter<pcl::PointXYZRGBA> mls;
            auto resampledTemplate = mls.Filter(t);

            PointPicker<pcl::PointXYZRGBA> picker;
            picker.SetCloud(resampledTemplate);
            Common<pcl::PointXYZRGBA>::Landmarks l = picker.Pick("Select the eyes and the nose", 3, "Select the left eye center, "
                                                                 "then the right eye center, then the tip of the nose.\n"
                                                                 "NOTE: Make sure use see the front part of the face, not the backward facing part");

            pcl::io::savePCDFileBinary(settings.GetString("Template"), *resampledTemplate);
            hpe::SaveLandmarks<pcl::PointXYZRGBA>(l, settings.GetString("Landmarks"));
        }
    }
    catch (HPEException ex)
    {
        std::cout << ex.what() << std::endl;
        std::cin.ignore();
    }

    return 0;
}

