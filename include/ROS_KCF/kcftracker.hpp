#pragma once

#include "tracker.h"

#ifndef _OPENCV_KCFTRACKER_HPP_
#define _OPENCV_KCFTRACKER_HPP_
#endif

class KCFTracker : public Tracker
{
public:

    KCFTracker(bool hog = true, bool fixed_window = true, bool multiscale = true, bool lab = true);


    virtual void init(const cv::Rect &roi, cv::Mat image);
    

    virtual cv::Rect update(cv::Mat image);

    float interp_factor; 
    float sigma;
    float lambda; 
    int cell_size; 
    int cell_sizeQ; 
    float padding; 
    float output_sigma_factor; 
    int template_size; 
    float scale_step;
    float scale_weight;  

protected:
    
    cv::Point2f detect(cv::Mat z, cv::Mat x, float &peak_value);

    
    void train(cv::Mat x, float train_interp_factor);

   
    cv::Mat gaussianCorrelation(cv::Mat x1, cv::Mat x2);

  
    cv::Mat createGaussianPeak(int sizey, int sizex);


    cv::Mat getFeatures(const cv::Mat & image, bool inithann, float scale_adjust = 1.0f);


    void createHanningMats();


    float subPixelPeak(float left, float center, float right);

    cv::Mat _alphaf;
    cv::Mat _prob;
    cv::Mat _tmpl;
    cv::Mat _num;
    cv::Mat _den;
    cv::Mat _labCentroids;

private:
    int size_patch[3];
    cv::Mat hann;
    cv::Size _tmpl_sz;
    float _scale;
    int _gaussian_size;
    bool _hogfeatures;
    bool _labfeatures;
};
