//
//  UIViewController+ImageViewController.h
//  EdgeInitializationStatic
//
//  Created by Hartisan on 15/10/4.
//  Copyright © 2015年 Hartisan. All rights reserved.
//

#ifdef __cplusplus
#import <opencv2/opencv.hpp>
#endif

#import <UIKit/UIKit.h>
#import <Foundation/Foundation.h>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/imgproc.hpp>
#import <opencv2/features2d.hpp>
#import <opencv2/highgui.hpp>
#import <opencv2/core/core.hpp>
#import <opencv2/line_descriptor/descriptor.hpp>
#import "GLViewController.h"

@interface ImageViewController : UIViewController {
    
    IBOutlet UIImageView* _imgView;
    NSString* _imgName;
    GLViewController* _glVC;
    cv::Mat _Ri;
    cv::Mat _cameraParamMatrix;
    cv::Mat _cameraParamMatrixInvert;
    std::vector<cv::Mat> _modelLinesVec;
    int _modelLineVertsNum;
    double* _modelLineVerts;
    std::vector<std::vector<cv::line_descriptor::KeyLine>> _classifiedImgLines;
    std::vector<std::vector<cv::Point2i>> _candidateCorrespondences;
}


@property (nonatomic, strong) IBOutlet UIImageView* _imgView;
@property (nonatomic, strong) NSString* _imgName;
@property (nonatomic, strong) GLViewController* _glVC;
@property cv::Mat _Ri;
@property cv::Mat _cameraParamMatrix;
@property cv::Mat _cameraParamMatrixInvert;
@property std::vector<cv::Mat> _modelLinesVec;
@property int _modelLineVertsNum;
@property double* _modelLineVerts;
@property std::vector<std::vector<cv::line_descriptor::KeyLine>> _classifiedImgLines;
@property std::vector<std::vector<cv::Point2i>> _candidateCorrespondences;


- (void)processWithImage:(cv::Mat)cvImage;
- (cv::Mat)getMaskfromHSVMat:(cv::Mat)hsvMat ofImg:(NSString*)imgName;
- (std::vector<double>)getCandidateAzimuthsWithImgLines:(std::vector<cv::line_descriptor::KeyLine>)imgLines;
- (std::vector<std::vector<cv::line_descriptor::KeyLine>>)classifyImgLinesBySlope:(std::vector<cv::line_descriptor::KeyLine>)imgLines;
- (cv::Mat)getCameraParamMatrix;
- (cv::Mat)getInclinationMatrix;
- (std::vector<cv::Mat>)getModelLinesWithName:(NSString*)imgName;
- (cv::Mat)getRotationMatrixWithAzimuth:(double)azimuth andInclinationMatrix:(cv::Mat)inclination;
- (void)setGLModelViewMatrixWithRotation:(cv::Mat)rotation andTranslation:(cv::Mat)translation;
- (bool)isCorrespondingToVerticalLines:(cv::line_descriptor::KeyLine)imgLine;
- (cv::Mat)convertImageCoord2CameraCoordAtU:(double)u andV:(double)v;
- (std::vector<double>)getGMAzimuthsFromCandidators:(std::vector<double>)candidateAzimuths;
- (cv::Mat)getProjectionOf3DPointX:(double)x Y:(double)y Z:(double)z withR:(cv::Mat)rotat andT:(cv::Mat)trans;
- (cv::Mat)getProjectionOf3DPointX:(double)x Y:(double)y Z:(double)z withCVMatrix:(cv::Mat)cvMatrix;
- (double)calcParalSimScoreBetweenProjPoints:(std::vector<cv::Mat>)projPoints andClassiLines:(std::vector<std::vector<cv::line_descriptor::KeyLine>>)classiLines;
- (cv::Mat)getTranlationWithGMAzimuths:(std::vector<double>)gmAzimuths;
- (double)distanceFromPoint:(cv::Point2d)ptC ToLineDeterminedByPoint:(cv::Point2d)ptA andPoint:(cv::Point2d)ptB;
- (cv::Mat)getTranslationWithThreePairsOfModelLines:(std::vector<int>)modelLinesIndex andImgLines:(std::vector<cv::line_descriptor::KeyLine>)imgLines andRotationMatrix:(cv::Mat)rotation;
- (int)calcDistanceScoreOfTranslation:(cv::Mat)translation withRotation:(cv::Mat)rotation;
- (void)setGLModelViewMatrixWithCVMatrix:(cv::Mat)cvMatrix;
- (void)trick;

@end
