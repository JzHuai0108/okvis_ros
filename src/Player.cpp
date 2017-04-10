/**
 * @file Player.cpp
 * @brief Source file for the Player class.
 * @author Jianzhu Huai
 */

#include <glog/logging.h>
#include <okvis/Player.hpp>
#include <functional>

#define THRESHOLD_DATA_DELAY_WARNING 0.1 // in seconds

/// \brief okvis Main namespace of this package.
namespace okvis {

Player::~Player()
{
}

Player::Player(okvis::VioInterface* vioInterfacePtr,
               const okvis::VioParameters & params)
    : vioInterface_(vioInterfacePtr),
      vioParameters_(params),
      mVideoFile(params.input.videoFile),
      mImuFile(params.input.imuFile),
      mIG(mImuFile, vio::SensorStreamCSV,0.005),
      mFG(mVideoFile, params.input.startIndex, params.input.finishIndex)
{

}

Player::Player(okvis::VioInterface* vioInterfacePtr,
               const okvis::VioParameters & params, std::string imageFolder)
    : vioInterface_(vioInterfacePtr),
      vioParameters_(params),
      mImageFolder(params.input.imageFolder),
      mTimeFile(params.input.timeFile),
      mImuFile(params.input.imuFile),
      mIG(mImuFile, vio::PlainText,0.01),
      mFG(mImageFolder, mTimeFile, params.input.startIndex, params.input.finishIndex)
{

}


void Player::Run()
{
    //for the first frame
    ros::Rate rate(vioParameters_.sensors_information.cameraRate);
    const double advance = 0.5; //+ advance to retrieve a little more imu data, thus,
            // avoid waiting in processing frames which causes false warning of delayed frames
    cv::Mat frame;
    double frameTime;
    int frameCounter(0);

    while(mFG.grabFrame(frame, frameTime)){

        cv::Mat filtered;
        if (vioParameters_.optimization.useMedianFilter) {
            cv::medianBlur(frame, filtered, 3);
        } else {
            filtered = frame;
        }
        okvis::Time t(frameTime);
        t -= okvis::Duration(vioParameters_.sensors_information.imageDelay);
        if (!vioInterface_->addImage(t, 0, filtered, NULL, mFG.getCurrentId()))
            LOG(WARNING) << "Frame delayed at time "<<t;

        // add corresponding imu data
        if(frameCounter ==0){
            mIG.getObservation(t.toSec() - 0.1); // 0.1 to avoid reading the first entry that may be useful for later processing
        }

        bool isMeasGood = mIG.getObservation(t.toSec() + advance);
        if(!isMeasGood) // the measurements can be bad when appraoching the end of a file
        {
            ++frameCounter;
            rate.sleep();
            continue;
        }
        std::vector<Eigen::Matrix<double, 7,1> > imuObservations = mIG.measurement;
//        std::cout <<"obs size "<<imuObservations.size();
        imuObservations.pop_back(); //remove the first entry which was the last in the previous observations

//        std::cout <<" start and finish timestamp " <<std::setprecision(12)<< imuObservations.front()[0] <<" "<< imuObservations.back()[0]<<std::endl;
        for(const Eigen::Matrix<double, 7,1> & obs: imuObservations)
        {            
            vioInterface_->addImuMeasurement(okvis::Time(obs[0]),
                                             obs.segment<3>(1), obs.segment<3>(4));
        }
        ++frameCounter;
        rate.sleep();
    }
}

} // namespace okvis
