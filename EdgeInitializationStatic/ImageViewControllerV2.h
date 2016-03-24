//
//  UIViewController+ImageViewControllerV2.h
//  EdgeInitializationStatic
//
//  Created by Hartisan on 16/1/2.
//  Copyright © 2016年 Hartisan. All rights reserved.
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


@interface ImageViewControllerV2 : UIViewController {
    
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


- (cv::Mat)getCameraParamMatrix;
- (void)processWithImage:(cv::Mat)cvImage;
- (cv::Mat)getMaskfromHSVMat:(cv::Mat)hsvMat ofImg:(NSString*)imgName;
- (bool)isCorrespondingToVerticalLines:(cv::line_descriptor::KeyLine)imgLine;
- (cv::Mat)convertImageCoord2CameraCoordAtU:(double)u andV:(double)v;
- (std::vector<std::vector<cv::line_descriptor::KeyLine>>)classifyImgLinesBySlope:(std::vector<cv::line_descriptor::KeyLine>)imgLines;
- (std::vector<double>)getCandidateAzimuthsWithImgLines:(std::vector<cv::line_descriptor::KeyLine>)imgLines;
- (cv::Mat)getInclinationMatrix;
- (cv::Mat)getRotationMatrixWithAzimuth:(double)azimuth andInclinationMatrix:(cv::Mat)inclination;
- (void)setGLModelViewMatrixWithRotation:(cv::Mat)rotation andTranslation:(cv::Mat)translation;
- (std::vector<std::vector<int>>)findAlignsWithRotation:(cv::Mat)rotationMatrix andImgLines:(std::vector<cv::line_descriptor::KeyLine>)imgLines;
- (cv::Mat)getProjectionOf3DPointX:(double)x Y:(double)y Z:(double)z withR:(cv::Mat)rotat andT:(cv::Mat)trans;
- (float)distanceFromPoint:(cv::Point2f)ptC ToLineDeterminedByPoint:(cv::Point2f)ptA andPoint:(cv::Point2f)ptB;
- (bool)isProjectedLineDetermByPoint:(cv::Point2f)projPtS andPoint:(cv::Point2f)projPtE alignedWithImgLineDetermByPoint:(cv::Point2f)imgPtS andPoint:(cv::Point2f)imgPtE;
- (cv::Mat)calcTransInAligns:(std::vector<std::vector<int>>)aligns withRot:(cv::Mat)rotation andImgLines:(std::vector<cv::line_descriptor::KeyLine>)imgLines;
- (cv::Mat)getTranslationWithThreePairsOfModelLines:(std::vector<int>)modelLinesIndex andImgLines:(std::vector<cv::line_descriptor::KeyLine>)imgLines andRotationMatrix:(cv::Mat)rotation;
- (void)setGLModelViewMatrixWithCVMatrix:(cv::Mat)cvMatrix;
- (bool)isModelVecAtIndex:(int)indexA ParallelWithVecAtIndex:(int)indexB;
- (cv::Mat)calcApproxTranslationWithColorMask:(cv::Mat)mask;
- (cv::Point2f)getMassCenterOfContour:(std::vector<cv::Point>)contour;
- (int)calcPoseScoreWithRot:(cv::Mat)rot andTrans:(cv::Mat)trans andAligns:(std::vector<std::vector<int>>)aligns andImgLines:(std::vector<cv::line_descriptor::KeyLine>)imgLines;
- (bool)isProjectedLineDetermByPoint:(cv::Point2f)projPtS andPoint:(cv::Point2f)projPtE overlapWithImgLineDetermByPoint:(cv::Point2f)imgPtS andPoint:(cv::Point2f)imgPtE;

@end
