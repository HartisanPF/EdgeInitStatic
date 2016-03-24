//
//  UIViewController+ImageViewController.m
//  EdgeInitializationStatic
//
//  Created by Hartisan on 15/10/4.
//  Copyright © 2015年 Hartisan. All rights reserved.
//

#import "ImageViewController.h"
//#import "Rib_Lines.h"
#import "lines_536A2070.h"
#import "lines_536A8210.h"
#import "lines_227A4120.h"
#import "lines_522A3211.h"
#import "lines_534A2090.h"
#import "lines_551A2100.h"

#define IPAD_CAMERA_PARAM_FX 536.84710693359375
#define IPAD_CAMERA_PARAM_FY 536.7637939453125
#define IPAD_CAMERA_PARAM_U 316.23187255859375
#define IPAD_CAMERA_PARAM_V 223.457733154296875
#define IMAGE_LINE_SLOPE_THREASH 2.0
#define AZIMUTH_SIMILAR_THREASH 2.5
#define VOTE_BOX_INTERVAL 6.0
#define VOTE_BOX_RATE 0.8
#define AZIMUTH_GM_RATE 0.9
#define LINE_VERTICAL_THREASH 4.0
#define LINE_DISTANCE_THRESH 5.0
#define RANDOM_TIME_N 20
#define LINE_OVERLAP_DIST 2.0
#define PRE_ASSUME_X 0.059
#define PRE_ASSUME_Y 0.018
#define PRE_ASSUME_Z 3.3

@implementation ImageViewController
    
@synthesize _imgView, _imgName, _cameraParamMatrix, _cameraParamMatrixInvert, _modelLinesVec, _modelLineVertsNum, _modelLineVerts, _classifiedImgLines, _glVC, _Ri, _candidateCorrespondences;


- (void)viewDidLoad {
    
    [super viewDidLoad];
    
    self._imgName = @"Rib_3.JPG";
    
    // 初始化内参矩阵并得到其逆矩阵
    cv::Mat cameraParamMatrix = [self getCameraParamMatrix];
    self._cameraParamMatrix = cameraParamMatrix;
    cv::Mat invert;
    cv::invert(cameraParamMatrix, invert);
    self._cameraParamMatrixInvert = invert;
    
    // 初始化模型直线向量集
    self._modelLinesVec = [self getModelLinesWithName:self._imgName];
    
    // 加载图像
    UIImage* image = [UIImage imageNamed:self._imgName];
    cv::Mat cvImage;
    UIImageToMat(image, cvImage);
    
    // 加载GL层
    GLViewController* glVC = [[GLViewController alloc] init];
    self._glVC = glVC;
    [self._glVC setModelNameWithImg:self._imgName];
    [self.view insertSubview:self._glVC.view atIndex:1];
    
    // 处理图像
    [self processWithImage:cvImage];
}


- (void)processWithImage:(cv::Mat)cvImage {
    
    if (!cvImage.empty()) {
        
        // 转换为hsv空间用来筛选颜色
        cv::Mat hsvMat;
        cvtColor(cvImage, hsvMat, CV_RGB2HSV);
        
        // 选出黄绿色的部分
        cv::Mat greenMask;
        greenMask = [self getMaskfromHSVMat:hsvMat ofImg:self._imgName];
        
        // 灰度化
        cv::Mat gray;
        cv::cvtColor(cvImage, gray, CV_RGB2GRAY);
        cv::Mat grayROI;
        gray.copyTo(grayROI, greenMask);
        
        // 创建BinaryDescriptor
        cv::Ptr<cv::line_descriptor::BinaryDescriptor> bd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
        
        // 用来保存提取出来的直线段
        std::vector<cv::line_descriptor::KeyLine> imgLines;
        std::vector<cv::line_descriptor::KeyLine> imgLinesVertical;   //对应于重力方向上的直线
        std::vector<cv::line_descriptor::KeyLine> imgLinesUnVertical; //其余可用来投票的直线
        
        NSDate* tmpStartData = [NSDate date];
        
        // 提取直线
        cv::Mat mask = cv::Mat::ones(cvImage.rows, cvImage.cols, CV_8UC1);
        bd->detect(gray, imgLines, mask);
        //bd->detect(grayROI, imgLines, greenMask);
        
        if (!imgLines.empty()) {
            
            // 筛选长度
            std::vector<cv::line_descriptor::KeyLine> filteredImgLines;
            for (int i = 0; i < imgLines.size(); ++i) {
                
                if (imgLines[i].lineLength < 300.0 && imgLines[i].lineLength > 10.0) {
                    
                    filteredImgLines.push_back(imgLines[i]);
                }
            }
            imgLines = filteredImgLines;
            
            // 把对应于重力方向上的直线筛选出来
            for (int i = 0; i < imgLines.size(); ++i) {
                
                if ([self isCorrespondingToVerticalLines:imgLines[i]]) {
                    
                    imgLinesVertical.push_back(imgLines[i]);
                    
                } else {
                    
                    imgLinesUnVertical.push_back(imgLines[i]);
                }
            }
            
            // 把提取到的直线按照斜率进行分类
            self._classifiedImgLines = [self classifyImgLinesBySlope:imgLinesUnVertical];
            
            // 先计算方位角
            std::vector<double> candidateAzimuths = [self getCandidateAzimuthsWithImgLines:imgLinesUnVertical];
            NSLog(@"candidateSize:%lu", candidateAzimuths.size());
            
            // 从候选方位角中进一步筛选
            std::vector<double> goldenMasterAzimuths = [self getGMAzimuthsFromCandidators:candidateAzimuths];
            NSLog(@"goldenMasterSize:%lu", goldenMasterAzimuths.size());
            
            // 结合方位角选出最可能的平移并组合为大矩阵
            cv::Mat cvMatrix = [self getTranlationWithGMAzimuths:goldenMasterAzimuths];
            
            double deltaTime = [[NSDate date] timeIntervalSinceDate:tmpStartData] * 1000.0;
            NSLog(@"cost time = %f", deltaTime);
            
            // 设置GL环境下的ModelView矩阵
            [self setGLModelViewMatrixWithCVMatrix:cvMatrix];
            //[self trick];
            
            /*
            for (int p = 0; p < cvMatrix.rows; p++) {
                
                for (int q = 0; q < cvMatrix.cols; q++) {
                    
                    NSLog(@"%f", cvMatrix.at<double>(p, q));
                }
                NSLog(@"--------");
            }
            NSLog(@"%f", goldenMasterAzimuths[0]);
            */
            
            // 测试用
            /*
            double t[] = {-0.05, 0.0, 1.0};
            cv::Mat trans = cv::Mat(3, 1, CV_64FC1, t);
            cv::Mat rot = [self getRotationMatrixWithAzimuth:goldenMasterAzimuths[0] andInclinationMatrix:self._Ri];
            [self setGLModelViewMatrixWithRotation:rot andTranslation:trans];
            */
            
            // 画线
            //cv::cvtColor(gray, gray, CV_GRAY2RGB);
            //cv::line_descriptor::drawKeylines(gray, imgLinesUnVertical, gray, cv::Scalar(0.0, 0.0, 255.0));
            
            /*
            // 把3D模型线根据计算得出的位姿重新画在图像上
            for (int j = 0; j < self._modelLineVertsNum; j += 6) {
                
                cv::Mat startPoint = [self getProjectionOf3DPointX:self._modelLineVerts[j] Y:self._modelLineVerts[j + 1] Z:self._modelLineVerts[j + 2] withCVMatrix:cvMatrix];
                cv::Mat endPoint = [self getProjectionOf3DPointX:self._modelLineVerts[j + 3] Y:self._modelLineVerts[j + 4] Z:self._modelLineVerts[j + 5] withCVMatrix:cvMatrix];
                cv::Point2d start;
                cv::Point2d end;
                start.x = startPoint.at<double>(0, 0);
                start.y = startPoint.at<double>(1, 0);
                end.x = endPoint.at<double>(0, 0);
                end.y = endPoint.at<double>(1, 0);
                cv::line(gray, start, end, cv::Scalar(0, 255, 0), 2.5);
            }*/
        }
        
        // 显示结果
        self._imgView.image = MatToUIImage(gray);
    }
}


/* 计时
NSDate* tmpStartData = [NSDate date];
double deltaTime = [[NSDate date] timeIntervalSinceDate:tmpStartData] * 1000.0;
NSLog(@"cost time = %f", deltaTime);
*/


// 获得某图像的目标颜色蒙版,其中S、V的参考值从PS中取得，但PS以255的百分比表示，需转换一下
- (cv::Mat)getMaskfromHSVMat:(cv::Mat)hsvMat ofImg:(NSString*)imgName {
    
    cv::Mat mask;
    cv::Scalar lower;
    cv::Scalar upper;
    
    if ([imgName isEqualToString:@"534-2090-1.JPG"]) {
        
        lower = cv::Scalar(25, 30 * 2.55, 10 * 2.55, 0.0);
        upper = cv::Scalar(35, 80 * 2.55, 90 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::erode(mask, mask, cv::Mat(3, 3, CV_8U));
        cv::dilate(mask, mask, cv::Mat(3, 3, CV_8U));
        cv::dilate(mask, mask, cv::Mat(5, 5, CV_8U));
        
    } else if ([imgName isEqualToString:@"534-2090-2.JPG"]) {

        lower = cv::Scalar(25, 30 * 2.55, 10 * 2.55, 0.0);
        upper = cv::Scalar(35, 80 * 2.55, 90 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::erode(mask, mask, cv::Mat(3, 3, CV_8U));
        cv::dilate(mask, mask, cv::Mat(9, 9, CV_8U));

    } else if ([imgName isEqualToString:@"536-2100-1.JPG"]) {
        
        lower = cv::Scalar(25, 15 * 2.55, 20 * 2.55, 0.0);
        upper = cv::Scalar(65, 80 * 2.55, 90 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::erode(mask, mask, cv::Mat(3, 3, CV_8U));
        cv::dilate(mask, mask, cv::Mat(9, 9, CV_8U));
        
    } else if ([imgName isEqualToString:@"536-2100-2.JPG"]) {
        
        lower = cv::Scalar(20, 5 * 2.55, 20 * 2.55, 0.0);
        upper = cv::Scalar(85, 80 * 2.55, 90 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::erode(mask, mask, cv::Mat(3, 3, CV_8U));
        cv::dilate(mask, mask, cv::Mat(7, 7, CV_8U));
        
    } else if ([imgName isEqualToString:@"536-1270-1.JPG"]) {
        
        lower = cv::Scalar(15, 0 * 2.55, 0 * 2.55, 0.0);
        upper = cv::Scalar(90, 100 * 2.55, 100 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::erode(mask, mask, cv::Mat(3, 3, CV_8U));
        cv::dilate(mask, mask, cv::Mat(9, 9, CV_8U));
        
    } else if ([imgName isEqualToString:@"536-1270-2.JPG"]) {
        
        lower = cv::Scalar(15, 0 * 2.55, 0 * 2.55, 0.0);
        upper = cv::Scalar(90, 100 * 2.55, 100 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::dilate(mask, mask, cv::Mat(7, 7, CV_8U));
        
    } else if ([imgName isEqualToString:@"522-3211-1.JPG"]) {
        
        lower = cv::Scalar(24, 0 * 2.55, 0 * 2.55, 0.0);
        upper = cv::Scalar(45, 90 * 2.55, 90 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::dilate(mask, mask, cv::Mat(3, 3, CV_8U));
        
    } else if ([imgName isEqualToString:@"522-3211-2.JPG"]) {
        
        lower = cv::Scalar(19, 0 * 2.55, 0 * 2.55, 0.0);
        upper = cv::Scalar(80, 100 * 2.55, 100 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::dilate(mask, mask, cv::Mat(3, 3, CV_8U));
        
    } else if ([imgName isEqualToString:@"551-2100-1.JPG"]) {
        
        lower = cv::Scalar(22, 25 * 2.55, 20 * 2.55, 0.0);
        upper = cv::Scalar(40, 70 * 2.55, 80 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::erode(mask, mask, cv::Mat(3, 3, CV_8U));
        cv::dilate(mask, mask, cv::Mat(5, 5, CV_8U));
        
    } else if ([imgName isEqualToString:@"551-2100-2.JPG"]) {
        
        lower = cv::Scalar(20, 25 * 2.55, 20 * 2.55, 0.0);
        upper = cv::Scalar(40, 70 * 2.55, 80 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::erode(mask, mask, cv::Mat(3, 3, CV_8U));
        cv::dilate(mask, mask, cv::Mat(5, 5, CV_8U));
        
    } else if ([imgName isEqualToString:@"536-2070-1.JPG"]) {
        
        lower = cv::Scalar(20, 25 * 2.55, 10 * 2.55, 0.0);
        upper = cv::Scalar(35, 80 * 2.55, 100 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::dilate(mask, mask, cv::Mat(3, 3, CV_8U));
        
    } else if ([imgName isEqualToString:@"536-2070-2.JPG"]) {
        
        lower = cv::Scalar(20, 15 * 2.55, 10 * 2.55, 0.0);
        upper = cv::Scalar(35, 80 * 2.55, 100 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::dilate(mask, mask, cv::Mat(3, 3, CV_8U));
        
    } else if ([imgName isEqualToString:@"536-8210-1.JPG"]) {
        
        lower = cv::Scalar(10, 15 * 2.55, 10 * 2.55, 0.0);
        upper = cv::Scalar(35, 80 * 2.55, 100 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        //cv::dilate(mask, mask, cv::Mat(3, 3, CV_8U));
        
    } else if ([imgName isEqualToString:@"536-8210-2.JPG"]) {
        
        lower = cv::Scalar(10, 15 * 2.55, 10 * 2.55, 0.0);
        upper = cv::Scalar(35, 80 * 2.55, 100 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        //cv::dilate(mask, mask, cv::Mat(3, 3, CV_8U));
        
    } else if ([imgName isEqualToString:@"227-4120-1.JPG"]) {
        
        lower = cv::Scalar(20, 27 * 2.55, 10 * 2.55, 0.0);
        upper = cv::Scalar(35, 80 * 2.55, 100 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::dilate(mask, mask, cv::Mat(3, 3, CV_8U));
        
    } else if ([imgName isEqualToString:@"227-4120-2.JPG"]) {
        
        lower = cv::Scalar(20, 27 * 2.55, 10 * 2.55, 0.0);
        upper = cv::Scalar(35, 80 * 2.55, 100 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::dilate(mask, mask, cv::Mat(3, 3, CV_8U));
        
    } else if ([imgName isEqualToString:@"572-5010-1.JPG"]) {
        
        lower = cv::Scalar(20, 50 * 2.55, 20 * 2.55, 0.0);
        upper = cv::Scalar(30, 80 * 2.55, 100 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::dilate(mask, mask, cv::Mat(7, 7, CV_8U));
        
    } else if ([imgName isEqualToString:@"572-5010-2.JPG"]) {
        
        lower = cv::Scalar(20, 50 * 2.55, 20 * 2.55, 0.0);
        upper = cv::Scalar(30, 80 * 2.55, 100 * 2.55, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::dilate(mask, mask, cv::Mat(5, 5, CV_8U));
        
    }
    
    return mask.clone();
}


// 判断一条图像直线是否对应于空间中的竖直线
- (bool)isCorrespondingToVerticalLines:(cv::line_descriptor::KeyLine)imgLine {
    
    bool result = false;
    
    // 像平面与竖直面的夹角
    double theta;
    if ([self._imgName isEqualToString:@"Rib_1.JPG"]) {
        
        theta = (90.0 - 53.741241) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"Rib_2.JPG"]) {
        
        theta = (90.0 - 62.839667) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"Rib_3.JPG"]) {
        
        theta = (90.0 - 64.008630) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"536-2070-1.JPG"]) {
        
        theta = (90.0 - 36.689331) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"536-2070-2.JPG"]) {
        
        theta = (90.0 - 35.855005) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"536-8210-1.JPG"]) {
        
        theta = (90.0 - 44.27457) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"536-8210-2.JPG"]) {
        
        theta = (90.0 - 41.693393) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"227-4120-1.JPG"]) {
        
        theta = (90.0 - 43.441994) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"227-4120-2.JPG"]) {
        
        theta = (90.0 - 32.392191) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"522-3211-1.JPG"]) {
        
        theta = (90.0 - 49.908894) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"522-3211-2.JPG"]) {
        
        theta = (90.0 - 60.466334) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"551-2100-1.JPG"]) {
        
        theta = (90.0 - 64.215849) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"551-2100-2.JPG"]) {
        
        theta = (90.0 - 61.553012) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"534-2090-1.JPG"]) {
        
        theta = (90.0 - 86.405039) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"534-2090-2.JPG"]) {
        
        theta = (90.0 - 85.138326) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"536-1270-1.JPG"]) {
        
        theta = (90.0 - 66.851341) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"536-1270-2.JPG"]) {
        
        theta = (90.0 - 60.366051) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"572-5010-1.JPG"]) {
        
        theta = (90.0 - 77.386077) * CV_PI / 180.0;
        
    } else if ([self._imgName isEqualToString:@"572-5010-2.JPG"]) {
        
        theta = (90.0 - 79.010749) * CV_PI / 180.0;
    }
    
    // 图像左上(A)、右上(B)、右下(C)三个点的坐标（相机坐标系）
    cv::Mat ptA = [self convertImageCoord2CameraCoordAtU:0.0 andV:0.0];
    cv::Mat ptB = [self convertImageCoord2CameraCoordAtU:640.0 andV:0.0];
    cv::Mat ptC = [self convertImageCoord2CameraCoordAtU:640.0 andV:480.0];
    
    // 计算出竖直面上C的对应点C'的坐标（相机坐标系）
    double dataC_[] = {ptC.at<double>(0, 0), (2.0 * cos(theta) - 1.0) * fabs(ptB.at<double>(1, 0)), 1.0 + 2.0 * sin(theta) * fabs(ptB.at<double>(1, 0))};
    cv::Mat ptC_ = cv::Mat(3, 1, CV_64FC1, dataC_);
    
    // 计算由A、B、C'三点确立的平面方程ax + by + cz = 1中的a、b、c (Ax=b)
    double dataA[] = {ptA.at<double>(0, 0), ptA.at<double>(1, 0), ptA.at<double>(2, 0),
                      ptB.at<double>(0, 0), ptB.at<double>(1, 0), ptB.at<double>(2, 0),
                      ptC_.at<double>(0, 0), ptC_.at<double>(1, 0), ptC_.at<double>(2, 0)};
    cv::Mat A = cv::Mat(3, 3, CV_64FC1, dataA);
    double datab[] = {1.0, 1.0, 1.0};
    cv::Mat b = cv::Mat(3, 1, CV_64FC1, datab);
    cv::Mat x = cv::Mat(3, 1, CV_64FC1);
    cv::solve(A, b, x);
    
    // 计算图像直线两端点的坐标（相机坐标系)
    cv::Mat startPt = [self convertImageCoord2CameraCoordAtU:imgLine.startPointX andV:imgLine.startPointY];
    cv::Mat endPt = [self convertImageCoord2CameraCoordAtU:imgLine.endPointX andV:imgLine.endPointY];
    
    // 计算图像直线两端点在竖直面上的投影坐标（相机坐标系)
    double dataAStart[] = {startPt.at<double>(1, 0), - startPt.at<double>(0, 0), 0.0,
                           startPt.at<double>(2, 0), 0.0, - startPt.at<double>(0, 0),
                           x.at<double>(0, 0), x.at<double>(1, 0), x.at<double>(2, 0)};
    cv::Mat AStart = cv::Mat(3, 3, CV_64FC1, dataAStart);
    double databStart[] = {0.0, 0.0, 1.0};
    cv::Mat bStart = cv::Mat(3, 1, CV_64FC1, databStart);
    cv::Mat startPt_ = cv::Mat(3, 1, CV_64FC1);
    cv::solve(AStart, bStart, startPt_);
    
    double dataAEnd[] = {endPt.at<double>(1, 0), - endPt.at<double>(0, 0), 0.0,
                         endPt.at<double>(2, 0), 0.0, - endPt.at<double>(0, 0),
                         x.at<double>(0, 0), x.at<double>(1, 0), x.at<double>(2, 0)};
    cv::Mat AEnd = cv::Mat(3, 3, CV_64FC1, dataAEnd);
    double databEnd[] = {0.0, 0.0, 1.0};
    cv::Mat bEnd = cv::Mat(3, 1, CV_64FC1, databEnd);
    cv::Mat endPt_ = cv::Mat(3, 1, CV_64FC1);
    cv::solve(AEnd, bEnd, endPt_);
    
    // 计算投影坐标的方向向量
    cv::Mat lineVec = startPt_ - endPt_;
    
    // 获得G的方向向量（相机坐标系)
    double gravityX;
    double gravityY;
    double gravityZ;
    
    if ([self._imgName isEqualToString:@"Rib_1.JPG"]) {
        
        gravityX = cos(CV_PI * 89.059116 / 180.0);
        gravityY = cos(CV_PI * 143.725044 / 180.0);
        gravityZ = - cos(CV_PI * 53.741241 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"Rib_2.JPG"]) {
        
        gravityX = cos(CV_PI * 86.900839 / 180.0);
        gravityY = cos(CV_PI * 139.390761 / 180.0);
        gravityZ = - cos(CV_PI * 62.839667 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"Rib_3.JPG"]) {
        
        gravityX = cos(CV_PI * 87.406410 / 180.0);
        gravityY = cos(CV_PI * 153.860007 / 180.0);
        gravityZ = - cos(CV_PI * 64.008630 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"536-2070-1.JPG"]) {
        
        gravityX = cos(CV_PI * 88.198512 / 180.0);
        gravityY = cos(CV_PI * 126.630219 / 180.0);
        gravityZ = - cos(CV_PI * 36.689331 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"536-2070-2.JPG"]) {
        
        gravityX = cos(CV_PI * 89.565554 / 180.0);
        gravityY = cos(CV_PI * 125.851537 / 180.0);
        gravityZ = - cos(CV_PI * 35.855005 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"536-8210-1.JPG"]) {
        
        gravityX = cos(CV_PI * 89.201693 / 180.0);
        gravityY = cos(CV_PI * 134.263445 / 180.0);
        gravityZ = - cos(CV_PI * 44.27457 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"536-8210-2.JPG"]) {
        
        gravityX = cos(CV_PI * 88.027158 / 180.0);
        gravityY = cos(CV_PI * 131.625025 / 180.0);
        gravityZ = - cos(CV_PI * 41.693393 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"227-4120-1.JPG"]) {
        
        gravityX = cos(CV_PI * 88.448033 / 180.0);
        gravityY = cos(CV_PI * 133.399903 / 180.0);
        gravityZ = - cos(CV_PI * 43.441994 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"227-4120-2.JPG"]) {
        
        gravityX = cos(CV_PI * 87.759108 / 180.0);
        gravityY = cos(CV_PI * 122.29529 / 180.0);
        gravityZ = - cos(CV_PI * 32.392191 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"522-3211-1.JPG"]) {
        
        gravityX = cos(CV_PI * 89.10001 / 180.0);
        gravityY = cos(CV_PI * 139.894547 / 180.0);
        gravityZ = - cos(CV_PI * 49.908894 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"522-3211-2.JPG"]) {
        
        gravityX = cos(CV_PI * 88.133952 / 180.0);
        gravityY = cos(CV_PI * 150.39556 / 180.0);
        gravityZ = - cos(CV_PI * 60.466334 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"551-2100-1.JPG"]) {
        
        gravityX = cos(CV_PI * 93.276431 / 180.0);
        gravityY = cos(CV_PI * 153.97714 / 180.0);
        gravityZ = - cos(CV_PI * 64.215849 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"551-2100-2.JPG"]) {
        
        gravityX = cos(CV_PI * 79.086255 / 180.0);
        gravityY = cos(CV_PI * 149.163319 / 180.0);
        gravityZ = - cos(CV_PI * 61.553012 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"534-2090-1.JPG"]) {
        
        gravityX = cos(CV_PI * 90.162304 / 180.0);
        gravityY = cos(CV_PI * 176.40136 / 180.0);
        gravityZ = - cos(CV_PI * 86.405039 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"534-2090-2.JPG"]) {
        
        gravityX = cos(CV_PI * 84.385956 / 180.0);
        gravityY = cos(CV_PI * 172.5632 / 180.0);
        gravityZ = - cos(CV_PI * 85.138326 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"536-1270-1.JPG"]) {
        
        gravityX = cos(CV_PI * 84.631664 / 180.0);
        gravityY = cos(CV_PI * 156.165403 / 180.0);
        gravityZ = - cos(CV_PI * 66.851341 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"536-1270-2.JPG"]) {
        
        gravityX = cos(CV_PI * 86.73147 / 180.0);
        gravityY = cos(CV_PI * 150.149844 / 180.0);
        gravityZ = - cos(CV_PI * 60.366051 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"572-5010-1.JPG"]) {
        
        gravityX = cos(CV_PI * 88.348465 / 180.0);
        gravityY = cos(CV_PI * 167.274879 / 180.0);
        gravityZ = - cos(CV_PI * 77.386077 / 180.0);
        
    } else if ([self._imgName isEqualToString:@"572-5010-2.JPG"]) {
        
        gravityX = cos(CV_PI * 88.16814 / 180.0);
        gravityY = cos(CV_PI * 168.855352 / 180.0);
        gravityZ = - cos(CV_PI * 79.010749 / 180.0);
    }
    
    double gravity[] = {gravityX, - gravityY, - gravityZ};
    cv::Mat gravityVec = cv::Mat(3, 1, CV_64FC1, gravity);
    
    // 计算lineVec和gravityVec的夹角
    double product = lineVec.dot(gravityVec);
    double lengthLine = sqrt(lineVec.dot(lineVec));
    double lengthGravity = sqrt(gravityVec.dot(gravityVec));
    double angle = acos(product / (lengthGravity * lengthLine)) * 180.0 / CV_PI;

    // 判断
    if (angle < LINE_VERTICAL_THREASH || fabs(angle - 180.0) < LINE_VERTICAL_THREASH) {
        
        result = true;
    }
    
    return result;
}


// 把图像坐标u、v转换为摄像机坐标系下的坐标
- (cv::Mat)convertImageCoord2CameraCoordAtU:(double)u andV:(double)v {
    
    cv::Mat cameraCoord;
    
    double data[] = {u, v, 1.0};
    cv::Mat imgPt = cv::Mat(3, 1, CV_64FC1, data);
    cameraCoord = self._cameraParamMatrixInvert * imgPt;
    
    return cameraCoord.clone();
}


// 计算候选的azimuth angle
- (std::vector<double>)getCandidateAzimuthsWithImgLines:(std::vector<cv::line_descriptor::KeyLine>)imgLines {
    
    // 根据重力分量获得倾斜矩阵Ri
    self._Ri = [self getInclinationMatrix];
    
    // 针对每一条检测到的直线，计算方位角并存储
    std::vector<double> azimuthAll;
    for (int i = 0; i < imgLines.size(); ++i) {
        
        // 先算法向量
        cv::Mat startVector = [self convertImageCoord2CameraCoordAtU:imgLines[i].startPointX andV:imgLines[i].startPointY];
        cv::Mat endVector = [self convertImageCoord2CameraCoordAtU:imgLines[i].endPointX andV:imgLines[i].endPointY];
        cv::Mat normVector = startVector.cross(endVector);
        
        // 算thetaZ
        cv::Mat normVectorTranspose;
        cv::transpose(normVector, normVectorTranspose);
        cv::Mat temp = normVectorTranspose * self._Ri;
        
        double a = temp.at<double>(0, 0);
        double b = temp.at<double>(0, 1);
        double c = temp.at<double>(0, 2);
        
        for (int j = 0; j < self._modelLinesVec.size(); ++j) {
            
            double d = self._modelLinesVec[j].at<double>(0, 0);
            double e = self._modelLinesVec[j].at<double>(1, 0);
            double f = self._modelLinesVec[j].at<double>(2, 0);
            
            double m = a * e - b * d;
            double n = a * d + b * e;
            double p = f * c;
            
            if ((m * m + n * n) > 0.0) {
                
                double b4ac = p * p * n * n - ((m * m + n * n) * (p * p - m * m));
                if (b4ac >= 0.0) {
                    
                    double cosTheta1 = ((- p * n) + sqrt(b4ac)) / (m * m + n * n);
                    double cosTheta2 = ((- p * n) - sqrt(b4ac)) / (m * m + n * n);
                    double azi1 = acos(cosTheta1) * 180.0 / CV_PI;
                    double azi2 = acos(cosTheta2) * 180.0 / CV_PI;
                    azimuthAll.push_back(azi1);
                    azimuthAll.push_back(180.0 + azi1);
                    azimuthAll.push_back(azi2);
                    azimuthAll.push_back(180.0 + azi2);
                }
                
            } else {
                
                continue;
            }
        }
    }
    
    // 准备选取候选的thetaZ
    std::vector<double> candidateAzimuths;
    int voteBox[(int)(360.0 / VOTE_BOX_INTERVAL)] = {0};
    double voteSum[(int)(360.0 / VOTE_BOX_INTERVAL)] = {0.0};
    int maxInBox = 0;
    
    // 先投票
    for (int i = 0; i < azimuthAll.size(); ++i) {
        
        int index = (int)(azimuthAll[i] / VOTE_BOX_INTERVAL);
        voteBox[index]++;
        voteSum[index] += azimuthAll[i];
    }
    
    // 找到最高票
    for (int i = 0; i < (int)(360.0 / VOTE_BOX_INTERVAL); ++i) {
        
        if (voteBox[i] > maxInBox) {
            
            maxInBox = voteBox[i];
        }
    }
    
    // 把排在前几名的都选出来当做候选
    for (int i = 0; i < (int)(360.0 / VOTE_BOX_INTERVAL); ++i) {
        
        if ((float)voteBox[i] / (float)maxInBox >= VOTE_BOX_RATE) {
            
            candidateAzimuths.push_back(voteSum[i] / (double)voteBox[i]);
        }
    }
    
    return candidateAzimuths;
}


// 从候选者中进一步筛选可能的azimuth
- (std::vector<double>)getGMAzimuthsFromCandidators:(std::vector<double>)candidateAzimuths {
    
    std::vector<double> gmAzimuths;
    std::vector<double> paraScore;
    double maxScore = 0.0;
    std::vector<int> maxScoreIndex;
    
    // 顺便把可能对应的线条对保存下来
    std::vector<std::vector<cv::Point2i>> possibleCorrespd;
    std::vector<std::vector<cv::Point2i>> candidateCorrespd;
    
    // 针对每个方位角，计算3D线条在图像上的投影并计算相似度
    for (int i = 0; i < candidateAzimuths.size(); ++i) {
        
        std::vector<cv::Mat> projectedPoints;
        std::vector<cv::Point2i> correspds;
        
        // 得到旋转矩阵
        cv::Mat rotationMatrix = [self getRotationMatrixWithAzimuth:candidateAzimuths[i] andInclinationMatrix:self._Ri];
        
        // 平移(假设在图像中心附近)
        double trans[] = {PRE_ASSUME_X, PRE_ASSUME_Y, PRE_ASSUME_Z};
        cv::Mat translate = cv::Mat(3, 1, CV_64FC1, trans);
        
        // 获得3D线条并计算在图像上的坐标
        for (int j = 0; j < self._modelLineVertsNum; j += 6) {
            
            cv::Mat startPoint = [self getProjectionOf3DPointX:self._modelLineVerts[j] Y:self._modelLineVerts[j + 1] Z:self._modelLineVerts[j + 2] withR:rotationMatrix andT:translate];
            cv::Mat endPoint = [self getProjectionOf3DPointX:self._modelLineVerts[j + 3] Y:self._modelLineVerts[j + 4] Z:self._modelLineVerts[j + 5] withR:rotationMatrix andT:translate];
            projectedPoints.push_back(startPoint.clone());
            projectedPoints.push_back(endPoint.clone());
            
            // 根据近似平行关系把可能对应的线条索引保存下来
            double slope = atan((endPoint.at<double>(1, 0) - startPoint.at<double>(1, 0))
                                / (endPoint.at<double>(0, 0) - startPoint.at<double>(0, 0))) * 180.0 / CV_PI;
            if (slope < 0.0) {
                
                slope += 180.0;
            }
            
            for (int k = 0; k < self._classifiedImgLines.size(); ++k) {
                
                double angle = self._classifiedImgLines[k][0].angle * 180.0 / CV_PI;
                if (angle < 0.0) {
                    
                    angle += 180.0;
                }
                
                if (fabs(slope - angle) < 3.0) {
                    
                    cv::Point2i corres;
                    corres.x = j;
                    corres.y = k;
                    correspds.push_back(corres);
                }
            }
        }
        
        double score = [self calcParalSimScoreBetweenProjPoints:projectedPoints andClassiLines:self._classifiedImgLines];
        paraScore.push_back(score);
        possibleCorrespd.push_back(correspds);
    }
    
    for (int i = 0; i < paraScore.size(); ++i) {
        
        if (paraScore[i] > maxScore) {
            
            maxScore = paraScore[i];
        }
    }
    
    for (int i = 0; i < paraScore.size(); ++i) {
        
        if (paraScore[i] / maxScore >= AZIMUTH_GM_RATE) {
            
            gmAzimuths.push_back(candidateAzimuths[i]);
            candidateCorrespd.push_back(possibleCorrespd[i]);
        }
    }
    
    self._candidateCorrespondences = candidateCorrespd;
    
    return gmAzimuths;
}


// 计算由某个azimuth得到的变换矩阵决定的投影线平行相似度
- (double)calcParalSimScoreBetweenProjPoints:(std::vector<cv::Mat>)projPoints andClassiLines:(std::vector<std::vector<cv::line_descriptor::KeyLine>>)classiLines {
    
    double score = 0.0;
    
    for (int i = 0; i < projPoints.size(); i += 2) {
        
        double diff = 181.0;
        double slope = atan((projPoints[i].at<double>(1, 0) - projPoints[i + 1].at<double>(1, 0))
                            / (projPoints[i].at<double>(0, 0) - projPoints[i + 1].at<double>(0, 0))) * 180.0 / CV_PI;
        
        if (slope < 0.0) {
            
            slope += 180.0;
        }
        
        for (int j = 0; j < classiLines.size(); ++j) {
            
            double angle = classiLines[j][0].angle * 180.0 / CV_PI;
            if (angle < 0.0) {
                
                angle += 180.0;
            }
            
            if (fabs(slope - angle) < diff) {
                
                diff = fabs(slope - angle);
            }
        }
        
        if (diff <= 3.0) {
            
            score += 15.0;
            
        } else if (diff > 3.0 && diff <= 6.0) {
            
            score += 10.0;
            
        } else if (diff > 6.0 && diff <= 10.0) {
            
            score += 6.0;
            
        } else if (diff > 10.0 && diff <= 15.0) {
            
            score += 2.0;
            
        } else if (diff > 15.0 && diff <= 20.0) {
            
            score += 1.0;
        }
    }
    
    return score;
}


// 根据最可能的方位角选出平移向量，并组合为大矩阵
- (cv::Mat)getTranlationWithGMAzimuths:(std::vector<double>)gmAzimuths {
    
    std::vector<int> scores;
    std::vector<cv::Mat> gmTrans;
    
    for (int i = 0; i < gmAzimuths.size(); ++i) {
        
        std::vector<cv::Point2i> candidateCorres;
        cv::Mat rotationMatrix = [self getRotationMatrixWithAzimuth:gmAzimuths[i] andInclinationMatrix:self._Ri];
        double trans[] = {PRE_ASSUME_X, PRE_ASSUME_Y, PRE_ASSUME_Z};
        cv::Mat translate = cv::Mat(3, 1, CV_64FC1, trans);
        
        if (self._candidateCorrespondences[i].size() == 0) {
            
            continue;
        }
        
        // 先剔除与方位角不匹配的
        for (int j = 0; j < self._candidateCorrespondences[i].size(); ++j) {
            
            int indexModel = self._candidateCorrespondences[i][j].x;
            int indexImg = self._candidateCorrespondences[i][j].y;
            
            for (int k = 0; k < self._classifiedImgLines[indexImg].size(); ++k) {
                
                cv::line_descriptor::KeyLine line = self._classifiedImgLines[indexImg][k];
                
                // 投影线与图像线距离不能太远
                cv::Mat startPt = [self getProjectionOf3DPointX:self._modelLineVerts[indexModel] Y:self._modelLineVerts[indexModel + 1] Z:self._modelLineVerts[indexModel + 2]
                                                          withR:rotationMatrix andT:translate];
                cv::Mat endPt = [self getProjectionOf3DPointX:self._modelLineVerts[indexModel + 3] Y:self._modelLineVerts[indexModel + 4] Z:self._modelLineVerts[indexModel + 5]
                                                        withR:rotationMatrix andT:translate];
                cv::Point2d start;
                cv::Point2d end;
                cv::Point2d beside;
                start.x = startPt.at<double>(0, 0);
                start.y = startPt.at<double>(1, 0);
                end.x = endPt.at<double>(0, 0);
                end.y = endPt.at<double>(1, 0);
                beside.x = line.pt.x;
                beside.y = line.pt.y;
                
                if ([self distanceFromPoint:beside ToLineDeterminedByPoint:start andPoint:end] < LINE_DISTANCE_THRESH) {
                    
                    cv::Point2i corr;
                    corr.x = j;
                    corr.y = k;
                    candidateCorres.push_back(corr);
                }
                /*
                // 先算法向量
                cv::Mat startVector = [self convertImageCoord2CameraCoordAtU:line.startPointX andV:line.startPointY];
                cv::Mat endVector = [self convertImageCoord2CameraCoordAtU:line.endPointX andV:line.endPointY];
                cv::Mat normVector = startVector.cross(endVector);
                
                // 算thetaZ
                cv::Mat normVectorTranspose;
                cv::transpose(normVector, normVectorTranspose);
                cv::Mat temp = normVectorTranspose * self._Ri;
                
                double a = temp.at<double>(0, 0);
                double b = temp.at<double>(0, 1);
                double c = temp.at<double>(0, 2);
                
                    
                double d = self._modelLineVerts[indexModel] - self._modelLineVerts[indexModel + 3];
                double e = self._modelLineVerts[indexModel + 1] - self._modelLineVerts[indexModel + 4];
                double f = self._modelLineVerts[indexModel + 2] - self._modelLineVerts[indexModel + 5];
                    
                double m = a * e - b * d;
                double n = a * d + b * e;
                double p = f * c;
                    
                if ((m * m + n * n) > 0.0) {
                        
                    double b4ac = p * p * n * n - ((m * m + n * n) * (p * p - m * m));
                    if (b4ac >= 0.0) {
                            
                        double cosTheta1 = ((- p * n) + sqrt(b4ac)) / (m * m + n * n);
                        double cosTheta2 = ((- p * n) - sqrt(b4ac)) / (m * m + n * n);
                        double azi1 = acos(cosTheta1) * 180.0 / CV_PI;
                        double azi2 = acos(cosTheta2) * 180.0 / CV_PI;
                            
                        if (fabs(azi1 - gmAzimuths[i]) < 2.0 || fabs(azi2 - gmAzimuths[i]) < 2.0 || fabs(180.0 + azi1 - gmAzimuths[i]) < 2.0 || fabs(180.0 + azi2 - gmAzimuths[i]) < 2.0) {
                            
                            // 投影线与图像线距离不能太远
                            cv::Mat startPt = [self getProjectionOf3DPointX:self._modelLineVerts[indexModel] Y:self._modelLineVerts[indexModel + 1] Z:self._modelLineVerts[indexModel + 2]
                                                                      withR:rotationMatrix andT:translate];
                            cv::Mat endPt = [self getProjectionOf3DPointX:self._modelLineVerts[indexModel + 3] Y:self._modelLineVerts[indexModel + 4] Z:self._modelLineVerts[indexModel + 5]
                                                                      withR:rotationMatrix andT:translate];
                            cv::Point2d start;
                            cv::Point2d end;
                            cv::Point2d beside;
                            start.x = startPt.at<double>(0, 0);
                            start.y = startPt.at<double>(1, 0);
                            end.x = endPt.at<double>(0, 0);
                            end.y = endPt.at<double>(1, 0);
                            beside.x = line.pt.x;
                            beside.y = line.pt.y;
                            
                            if ([self distanceFromPoint:beside ToLineDeterminedByPoint:start andPoint:end] < LINE_DISTANCE_THRESH) {
                                
                                cv::Point2i corr;
                                corr.x = j;
                                corr.y = k;
                                candidateCorres.push_back(corr);
                            }
                        }
                    }
                }*/
            }
        }
        
        // 重新组织数据结构准备随机抽选
        std::vector<std::vector<cv::Point2i>> sampleDB;
        
        // 先把第一个装进去
        std::vector<cv::Point2i> first;
        first.push_back(candidateCorres[0]);
        sampleDB.push_back(first);
        
        // 再循环遍历装载剩下的
        for (int m = 1; m < candidateCorres.size(); ++m) {
            
            long size = sampleDB.size();
            
            for (int n = 0; n < size; ++n) {
                
                if (self._candidateCorrespondences[i][candidateCorres[m].x].x == self._candidateCorrespondences[i][sampleDB[n][0].x].x) {
                    
                    sampleDB[n].push_back(candidateCorres[m]);
                    break;
                }

                if (n == (size - 1)) {
                    
                    std::vector<cv::Point2i> newOne;
                    newOne.push_back(candidateCorres[m]);
                    sampleDB.push_back(newOne);
                }
            }
        }        
        
        // 随机n次，每次随机选3对线
        int maxScore = 0;
        cv::Mat maxTrans;
        
        for (int p = 0; p < RANDOM_TIME_N; ++p) {
            
            // 产生三个不同的随机数来决定哪条模型线参与计算
            int firstI = arc4random() % sampleDB.size();
            int secondI = firstI;
            int thirdI = firstI;
            
            if (sampleDB.size() >= 3) {
                
                while (secondI == firstI) {
                    
                    secondI = arc4random() % sampleDB.size();
                }
                
                while (thirdI == firstI || thirdI == secondI) {
                    
                    thirdI = arc4random() % sampleDB.size();
                }
                
            } else if (sampleDB.size() == 2) {
                
                while (secondI == firstI) {
                    
                    secondI = arc4random() % sampleDB.size();
                }
                
                thirdI = arc4random() % sampleDB.size();
            }
            
            // 再产生3个随机数来确定哪条图像直线参与计算
            int firstJ = arc4random() % sampleDB[firstI].size();
            int secondJ = arc4random() % sampleDB[secondI].size();
            int thirdJ = arc4random() % sampleDB[thirdI].size();
            
            // 由3条空间直线与3条图像直线联立方程计算出平移
            std::vector<int> modelLinesIndex;
            modelLinesIndex.push_back(self._candidateCorrespondences[i][sampleDB[firstI][0].x].x);
            modelLinesIndex.push_back(self._candidateCorrespondences[i][sampleDB[secondI][0].x].x);
            modelLinesIndex.push_back(self._candidateCorrespondences[i][sampleDB[thirdI][0].x].x);
            std::vector<cv::line_descriptor::KeyLine> imgLines;
            imgLines.push_back(self._classifiedImgLines[self._candidateCorrespondences[i][sampleDB[firstI][firstJ].x].y][sampleDB[firstI][firstJ].y]);
            imgLines.push_back(self._classifiedImgLines[self._candidateCorrespondences[i][sampleDB[secondI][secondJ].x].y][sampleDB[secondI][secondJ].y]);
            imgLines.push_back(self._classifiedImgLines[self._candidateCorrespondences[i][sampleDB[thirdI][thirdJ].x].y][sampleDB[thirdI][thirdJ].y]);
            
            cv::Mat translation = [self getTranslationWithThreePairsOfModelLines:modelLinesIndex andImgLines:imgLines andRotationMatrix:rotationMatrix];
            
            // 对于可能的平移计算其最小距离分数
            if (translation.at<double>(2, 0) > 0.0 && fabs(translation.at<double>(2, 0)) < 20.0) {
                
                if (fabs(translation.at<double>(0, 0)) < 10.0 && fabs(translation.at<double>(1, 0)) < 10.0) {
                    
                    int score = [self calcDistanceScoreOfTranslation:translation withRotation:rotationMatrix];
                    
                    if (score > maxScore) {
                        
                        maxScore = score;
                        maxTrans = translation;
                    }
                }
            }
        }
        
        scores.push_back(maxScore);
        gmTrans.push_back(maxTrans);
    }
    
    // 从所有方位角对应的候选平移中选出最牛逼的那个
    int maxIndex = 0;
    int maxScore = 0;
    
    for (int i = 0; i < scores.size(); ++i) {
        
        if (scores[i] > maxScore) {
            
            maxScore = scores[i];
            maxIndex = i;
        }
    }
    
    //NSLog(@"score:%d", maxScore);
    //NSLog(@"translate: %f, %f, %f", gmTrans[maxIndex].at<double>(0, 0), gmTrans[maxIndex].at<double>(1, 0), gmTrans[maxIndex].at<double>(2, 0));
    
    // 整合为大矩阵
    cv::Mat R = [self getRotationMatrixWithAzimuth:gmAzimuths[maxIndex] andInclinationMatrix:self._Ri];
    double data[] = {R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), gmTrans[maxIndex].at<double>(0, 0),
                     R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), gmTrans[maxIndex].at<double>(1, 0),
                     R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), gmTrans[maxIndex].at<double>(2, 0),
                     0.0, 0.0, 0.0, 1.0};
    cv::Mat cvMatrix = cv::Mat(4, 4, CV_64FC1, data);
    
    return cvMatrix.clone();
}


// 由3条空间直线与3条图像直线联立方程计算出平移
- (cv::Mat)getTranslationWithThreePairsOfModelLines:(std::vector<int>)modelLinesIndex andImgLines:(std::vector<cv::line_descriptor::KeyLine>)imgLines andRotationMatrix:(cv::Mat)rotation {
    
    cv::Mat translation;
    
    // 法向量
    cv::Mat startVector1 = [self convertImageCoord2CameraCoordAtU:imgLines[0].startPointX andV:imgLines[0].startPointY];
    cv::Mat endVector1 = [self convertImageCoord2CameraCoordAtU:imgLines[0].endPointX andV:imgLines[0].endPointY];
    cv::Mat normVector1 = startVector1.cross(endVector1);
    cv::Mat startVector2 = [self convertImageCoord2CameraCoordAtU:imgLines[1].startPointX andV:imgLines[1].startPointY];
    cv::Mat endVector2 = [self convertImageCoord2CameraCoordAtU:imgLines[1].endPointX andV:imgLines[1].endPointY];
    cv::Mat normVector2 = startVector2.cross(endVector2);
    cv::Mat startVector3 = [self convertImageCoord2CameraCoordAtU:imgLines[2].startPointX andV:imgLines[2].startPointY];
    cv::Mat endVector3 = [self convertImageCoord2CameraCoordAtU:imgLines[2].endPointX andV:imgLines[2].endPointY];
    cv::Mat normVector3 = startVector3.cross(endVector3);
    
    // R * Pw
    double p1[] = {self._modelLineVerts[modelLinesIndex[0]], self._modelLineVerts[modelLinesIndex[0] + 1], self._modelLineVerts[modelLinesIndex[0] + 2]};
    double p2[] = {self._modelLineVerts[modelLinesIndex[1]], self._modelLineVerts[modelLinesIndex[1] + 1], self._modelLineVerts[modelLinesIndex[1] + 2]};
    double p3[] = {self._modelLineVerts[modelLinesIndex[2]], self._modelLineVerts[modelLinesIndex[2] + 1], self._modelLineVerts[modelLinesIndex[2] + 2]};
    
    cv::Mat pw1 = cv::Mat(3, 1, CV_64FC1, p1);
    cv::Mat pw2 = cv::Mat(3, 1, CV_64FC1, p2);
    cv::Mat pw3 = cv::Mat(3, 1, CV_64FC1, p3);
    cv::Mat Rp1 = rotation * pw1;
    cv::Mat Rp2 = rotation * pw2;
    cv::Mat Rp3 = rotation * pw3;
    
    double n11 = normVector1.at<double>(0, 0);
    double n12 = normVector1.at<double>(1, 0);
    double n13 = normVector1.at<double>(2, 0);
    double n21 = normVector2.at<double>(0, 0);
    double n22 = normVector2.at<double>(1, 0);
    double n23 = normVector2.at<double>(2, 0);
    double n31 = normVector3.at<double>(0, 0);
    double n32 = normVector3.at<double>(1, 0);
    double n33 = normVector3.at<double>(2, 0);
    double a1 = Rp1.at<double>(0, 0);
    double b1 = Rp1.at<double>(1, 0);
    double c1 = Rp1.at<double>(2, 0);
    double a2 = Rp2.at<double>(0, 0);
    double b2 = Rp2.at<double>(1, 0);
    double c2 = Rp2.at<double>(2, 0);
    double a3 = Rp3.at<double>(0, 0);
    double b3 = Rp3.at<double>(1, 0);
    double c3 = Rp3.at<double>(2, 0);
    
    double Adata[] = {n11, n12, n13, n21, n22, n23, n31, n32, n33};
    double bdata[] = {- a1 * n11 - b1 * n12 - c1 * n13, - a2 * n21 - b2 * n22 - c2 * n23, - a3 * n31 - b3 * n32 - c3 * n33};
    cv::Mat A = cv::Mat(3, 3, CV_64FC1, Adata);
    cv::Mat b = cv::Mat(3, 1, CV_64FC1, bdata);
    cv::solve(A, b, translation, CV_SVD);
    
    return translation.clone();
}


// 计算某个平移的最小距离分数
- (int)calcDistanceScoreOfTranslation:(cv::Mat)translation withRotation:(cv::Mat)rotation {
    
    int score = 0;
    
    // 对每一条模型线计算其投影线
    for (int i = 0; i < self._modelLineVertsNum; i += 6) {
        
        cv::Mat startPt = [self getProjectionOf3DPointX:self._modelLineVerts[i] Y:self._modelLineVerts[i + 1] Z:self._modelLineVerts[i + 2] withR:rotation andT:translation];
        cv::Mat endPt = [self getProjectionOf3DPointX:self._modelLineVerts[i + 3] Y:self._modelLineVerts[i + 4] Z:self._modelLineVerts[i+5] withR:rotation andT:translation];
        
        double slope = atan((endPt.at<double>(1, 0) - startPt.at<double>(1, 0))
                            / (endPt.at<double>(0, 0) - startPt.at<double>(0, 0))) * 180.0 / CV_PI;
        
        if (slope < 0.0) {
            
            slope += 180.0;
        }
        
        // 在相似斜率的图像线中寻找是否存在吻合线
        int flag = 0;
        for (int j = 0; j < self._classifiedImgLines.size(); ++j) {
            
            if (flag == 1) {
                break;
            }
            
            double angle = self._classifiedImgLines[j][0].angle * 180.0 / CV_PI;
            if (angle < 0.0) {
                
                angle += 180.0;
            }
            
            if (fabs(slope - angle) < 2.0) {
                
                for (int k = 0; k < self._classifiedImgLines[j].size(); ++k) {
                    
                    cv::Point2d startProj;
                    cv::Point2d endProj;
                    startProj.x = startPt.at<double>(0, 0);
                    startProj.y = startPt.at<double>(1, 0);
                    endProj.x = endPt.at<double>(0, 0);
                    endProj.y = endPt.at<double>(1, 0);
                    cv::Point2d startImg;
                    cv::Point2d endImg;
                    cv::Point2d midImg;
                    startImg.x = self._classifiedImgLines[j][k].startPointX;
                    startImg.y = self._classifiedImgLines[j][k].startPointY;
                    endImg.x = self._classifiedImgLines[j][k].endPointX;
                    endImg.y = self._classifiedImgLines[j][k].endPointY;
                    midImg.x = (startImg.x + endImg.x) / 2.0;
                    midImg.y = (startImg.y + endImg.y) / 2.0;
                    cv::Rect2d rect = cv::Rect2d(startProj, endProj);
                    
                    double distanceStart = [self distanceFromPoint:startProj ToLineDeterminedByPoint:startImg andPoint:endImg];
                    double distanceEnd = [self distanceFromPoint:endProj ToLineDeterminedByPoint:startImg andPoint:endImg];
                    
                    if (distanceStart <= LINE_OVERLAP_DIST && distanceEnd <= LINE_OVERLAP_DIST) {
                        
                        if (midImg.inside(rect)) {
                            
                            score++;
                            flag = 1;
                            break;
                        }
                    }
                }
            }
        }
    }
    
    return score;
}


// 平面上一点到直线的距离（直线由两点决定）
- (double)distanceFromPoint:(cv::Point2d)ptC ToLineDeterminedByPoint:(cv::Point2d)ptA andPoint:(cv::Point2d)ptB {
    
    float distance;
    
    if (ptA.x - ptB.x != 0.0) {
        
        float A = (ptB.y - ptA.y) / (ptB.x - ptA.x);
        float C = ptA.y - A * ptA.x;
        distance = fabs(A * ptC.x - ptC.y + C) / sqrt(A * A + 1.0);
        
    } else {
        
        distance = fabs(ptC.x - ptA.x);
    }
    
    return distance;
}


// 计算3D空间某点在像平面的投影点(三维矩阵)
- (cv::Mat)getProjectionOf3DPointX:(double)x Y:(double)y Z:(double)z withR:(cv::Mat)rotat andT:(cv::Mat)trans {
    
    cv::Mat projection;
    double pt[] = {x, y, z};
    cv::Mat point3D = cv::Mat(3, 1, CV_64FC1, pt);
    cv::Mat pointCam = rotat * point3D + trans;
    cv::Mat pointCamUniform = pointCam / pointCam.at<double>(2, 0);
    projection = self._cameraParamMatrix * pointCamUniform;
    
    return projection.clone();
}


// 计算3D空间某点在像平面的投影点(四维矩阵)
- (cv::Mat)getProjectionOf3DPointX:(double)x Y:(double)y Z:(double)z withCVMatrix:(cv::Mat)cvMatrix {
    
    cv::Mat projection;
    double pt[] = {x, y, z, 1.0};
    cv::Mat point3D = cv::Mat(4, 1, CV_64FC1, pt);
    cv::Mat pointCam = cvMatrix * point3D;
    double ptU[] = {pointCam.at<double>(0, 0) / pointCam.at<double>(2, 0), pointCam.at<double>(1, 0) / pointCam.at<double>(2, 0), 1.0};
    cv::Mat pointCamUniform = cv::Mat(3, 1, CV_64FC1, ptU);
    projection = self._cameraParamMatrix * pointCamUniform;
    
    return projection.clone();
}


// 获得摄像机内参矩阵
- (cv::Mat)getCameraParamMatrix {
    
    cv::Mat matrix;
    
    double elements[] = {IPAD_CAMERA_PARAM_FX, 0.0, IPAD_CAMERA_PARAM_U, 0.0, IPAD_CAMERA_PARAM_FY, IPAD_CAMERA_PARAM_V, 0.0, 0.0, 1.0};
    matrix = cv::Mat(3, 3, CV_64FC1, elements);
    
    return matrix.clone();
}


// 获得模型直线向量集合,同时获得所有顶点
- (std::vector<cv::Mat>)getModelLinesWithName:(NSString*)imgName {
    
    std::vector<cv::Mat> lines;
    
    if ([imgName isEqualToString:@"Rib_1.JPG"] || [imgName isEqualToString:@"Rib_2.JPG"] || [imgName isEqualToString:@"Rib_3.JPG"]) {
        /*
        // 解析Rib_Lines.h
        for (int i = 0; i < _ribLineVertsNum; i += 36) {
            
            double data[] = {_ribLineVerts[i] - _ribLineVerts[i + 3],
                _ribLineVerts[i + 1] - _ribLineVerts[i + 4],
                _ribLineVerts[i + 2] - _ribLineVerts[i + 5]};
            cv::Mat vec = cv::Mat(3, 1, CV_64FC1, data).clone();
            lines.push_back(vec);
        }
        
        self._modelLineVertsNum = _ribLineVertsNum;
        self._modelLineVerts = _ribLineVerts;
        */
    } else if ([imgName isEqualToString:@"536-2070-1.JPG"] || [imgName isEqualToString:@"536-2070-2.JPG"]) {
        
        // 解析lines_536A2070.h
        for (int i = 0; i < _536A2070LineVertsNum; i += 6) {
            
            double data[] = {_536A2070LineVerts[i] - _536A2070LineVerts[i + 3],
                _536A2070LineVerts[i + 1] - _536A2070LineVerts[i + 4],
                _536A2070LineVerts[i + 2] - _536A2070LineVerts[i + 5]};
            cv::Mat vec = cv::Mat(3, 1, CV_64FC1, data).clone();
            lines.push_back(vec);
        }
        
        self._modelLineVertsNum = _536A2070LineVertsNum;
        self._modelLineVerts = _536A2070LineVerts;

    } else if ([imgName isEqualToString:@"536-8210-1.JPG"] || [imgName isEqualToString:@"536-8210-2.JPG"]) {
        
        // 解析lines_536A8210.h
        for (int i = 0; i < _536A8210LineVertsNum; i += 6) {
            
            double data[] = {_536A8210LineVerts[i] - _536A8210LineVerts[i + 3],
                _536A8210LineVerts[i + 1] - _536A8210LineVerts[i + 4],
                _536A8210LineVerts[i + 2] - _536A8210LineVerts[i + 5]};
            cv::Mat vec = cv::Mat(3, 1, CV_64FC1, data).clone();
            lines.push_back(vec);
        }
        
        self._modelLineVertsNum = _536A8210LineVertsNum;
        self._modelLineVerts = _536A8210LineVerts;
        
    } else if ([imgName isEqualToString:@"227-4120-1.JPG"] || [imgName isEqualToString:@"227-4120-2.JPG"]) {
        
        // 解析lines_227A4120.h
        for (int i = 0; i < _227A4120LineVertsNum; i += 6) {
            
            double data[] = {_227A4120LineVerts[i] - _227A4120LineVerts[i + 3],
                _227A4120LineVerts[i + 1] - _227A4120LineVerts[i + 4],
                _227A4120LineVerts[i + 2] - _227A4120LineVerts[i + 5]};
            cv::Mat vec = cv::Mat(3, 1, CV_64FC1, data).clone();
            lines.push_back(vec);
        }
        
        self._modelLineVertsNum = _227A4120LineVertsNum;
        self._modelLineVerts = _227A4120LineVerts;
        
    } else if ([imgName isEqualToString:@"522-3211-1.JPG"] || [imgName isEqualToString:@"522-3211-2.JPG"]) {
        
        // 解析lines_522A3211.h
        for (int i = 0; i < _522A3211LineVertsNum; i += 6) {
            
            double data[] = {_522A3211LineVerts[i] - _522A3211LineVerts[i + 3],
                _522A3211LineVerts[i + 1] - _522A3211LineVerts[i + 4],
                _522A3211LineVerts[i + 2] - _522A3211LineVerts[i + 5]};
            cv::Mat vec = cv::Mat(3, 1, CV_64FC1, data).clone();
            lines.push_back(vec);
        }
        
        self._modelLineVertsNum = _522A3211LineVertsNum;
        self._modelLineVerts = _522A3211LineVerts;
        
    } else if ([imgName isEqualToString:@"534-2090-1.JPG"] || [imgName isEqualToString:@"534-2090-2.JPG"]) {
        
        // 解析lines_534A2090.h
        for (int i = 0; i < _534A2090LineVertsNum; i += 6) {
            
            double data[] = {_534A2090LineVerts[i] - _534A2090LineVerts[i + 3],
                _534A2090LineVerts[i + 1] - _534A2090LineVerts[i + 4],
                _534A2090LineVerts[i + 2] - _534A2090LineVerts[i + 5]};
            cv::Mat vec = cv::Mat(3, 1, CV_64FC1, data).clone();
            lines.push_back(vec);
        }
        
        self._modelLineVertsNum = _534A2090LineVertsNum;
        self._modelLineVerts = _534A2090LineVerts;
        
    } else if ([imgName isEqualToString:@"551-2100-1.JPG"] || [imgName isEqualToString:@"551-2100-2.JPG"]) {
        
        // 解析lines_551A2100.h
        for (int i = 0; i < _551A2100LineVertsNum; i += 6) {
            
            double data[] = {_551A2100LineVerts[i] - _551A2100LineVerts[i + 3],
                _551A2100LineVerts[i + 1] - _551A2100LineVerts[i + 4],
                _551A2100LineVerts[i + 2] - _551A2100LineVerts[i + 5]};
            cv::Mat vec = cv::Mat(3, 1, CV_64FC1, data).clone();
            lines.push_back(vec);
        }
        
        self._modelLineVertsNum = _551A2100LineVertsNum;
        self._modelLineVerts = _551A2100LineVerts;
        
    }
    
    return lines;
}


// 由方位角和倾斜矩阵整合出旋转矩阵
- (cv::Mat)getRotationMatrixWithAzimuth:(double)azimuth andInclinationMatrix:(cv::Mat)inclination {
    
    cv::Mat rotation;

    double azimuthMatrix[] = {cos(azimuth * CV_PI / 180.0), sin(azimuth * CV_PI / 180.0), 0.0,
                            -sin(azimuth * CV_PI / 180.0), cos(azimuth * CV_PI / 180.0), 0.0,
                            0.0, 0.0, 1.0};
    cv::Mat azimuthMat = cv::Mat(3, 3, CV_64FC1, azimuthMatrix);
    
    rotation = inclination * azimuthMat;
    
    return rotation.clone();
}


// 根据旋转和平移得到GL环境下的ModelView矩阵
- (void)setGLModelViewMatrixWithRotation:(cv::Mat)rotation andTranslation:(cv::Mat)translation {
    
    float matrix[16];
    double conv[] = {1.0,  0.0,  0.0,
                     0.0, -1.0,  0.0,
                     0.0,  0.0, -1.0};
    cv::Mat convert = cv::Mat(3, 3, CV_64FC1, conv);
    rotation = convert * rotation;
    translation = convert * translation;
    
    matrix[0] = rotation.at<double>(0, 0);
    matrix[1] = rotation.at<double>(1, 0);
    matrix[2] = rotation.at<double>(2, 0);
    matrix[3] = 0.0f;
    
    matrix[4] = rotation.at<double>(0, 1);
    matrix[5] = rotation.at<double>(1, 1);
    matrix[6] = rotation.at<double>(2, 1);
    matrix[7] = 0.0f;
    
    matrix[8] = rotation.at<double>(0, 2);
    matrix[9] = rotation.at<double>(1, 2);
    matrix[10] = rotation.at<double>(2, 2);
    matrix[11] = 0.0f;
    
    matrix[12] = translation.at<double>(0, 0);
    matrix[13] = translation.at<double>(1, 0);
    matrix[14] = translation.at<double>(2, 0);
    matrix[15] = 1.0f;
    
    self._glVC._modelViewMatrix = GLKMatrix4MakeWithArray(matrix);
}


// 把CV环境下的大矩阵转为GL环境下的ModelView矩阵
- (void)setGLModelViewMatrixWithCVMatrix:(cv::Mat)cvMatrix {
    
    float matrix[16];
    double conv[] = {1.0,  0.0,  0.0, 0.0,
                     0.0, -1.0,  0.0, 0.0,
                     0.0,  0.0, -1.0, 0.0,
                     0.0,  0.0,  0.0, 1.0};
    cv::Mat convert = cv::Mat(4, 4, CV_64FC1, conv);
    cvMatrix = convert * cvMatrix;
    
    matrix[0] = cvMatrix.at<double>(0, 0);
    matrix[1] = cvMatrix.at<double>(1, 0);
    matrix[2] = cvMatrix.at<double>(2, 0);
    matrix[3] = 0.0f;
    
    matrix[4] = cvMatrix.at<double>(0, 1);
    matrix[5] = cvMatrix.at<double>(1, 1);
    matrix[6] = cvMatrix.at<double>(2, 1);
    matrix[7] = 0.0f;
    
    matrix[8] = cvMatrix.at<double>(0, 2);
    matrix[9] = cvMatrix.at<double>(1, 2);
    matrix[10] = cvMatrix.at<double>(2, 2);
    matrix[11] = 0.0f;
    
    matrix[12] = cvMatrix.at<double>(0, 3);
    matrix[13] = cvMatrix.at<double>(1, 3);
    matrix[14] = cvMatrix.at<double>(2, 3);
    matrix[15] = 1.0f;
    
    self._glVC._modelViewMatrix = GLKMatrix4MakeWithArray(matrix);
}


// 把提取到的直线按照斜率分类组织
- (std::vector<std::vector<cv::line_descriptor::KeyLine>>)classifyImgLinesBySlope:(std::vector<cv::line_descriptor::KeyLine>)imgLines {
    
    std::vector<std::vector<cv::line_descriptor::KeyLine>> classifiedImgLines;
    
    // 先把第一个装进去
    std::vector<cv::line_descriptor::KeyLine> firstLineClass;
    firstLineClass.push_back(imgLines[0]);
    classifiedImgLines.push_back(firstLineClass);
    
    // 再循环遍历装载剩下的
    for (int i = 1; i < imgLines.size(); ++i) {
        
        long size = classifiedImgLines.size();
        for (int j = 0; j < size; ++j) {
            
            double slopeI = imgLines[i].angle * 180.0 / CV_PI;
            double slopeC = classifiedImgLines[j][0].angle * 180.0 / CV_PI;
            
            if (slopeI < 0.0) {
                
                slopeI += 180.0;
            }
            
            if (slopeC < 0.0) {
                
                slopeC += 180.0;
            }
            
            if (fabs(slopeI - slopeC) <= IMAGE_LINE_SLOPE_THREASH) {
                
                classifiedImgLines[j].push_back(imgLines[i]);
                break;
                
            } else if (j == (size - 1)) {
                
                std::vector<cv::line_descriptor::KeyLine> newLineClass;
                newLineClass.push_back(imgLines[i]);
                classifiedImgLines.push_back(newLineClass);
            }
        }
    }
    
    return classifiedImgLines;
}


// 根据重力分量获得倾斜矩阵
- (cv::Mat)getInclinationMatrix {
    
    double thetaX;
    double thetaY;
    
    if ([self._imgName isEqualToString:@"Rib_1.JPG"]) {
        
        thetaY = (90.0 - 89.059116) * CV_PI / 180.0;
        thetaX = acos(cos(53.741241 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"Rib_2.JPG"]) {
        
        thetaY = (90.0 - 86.900839) * CV_PI / 180.0;
        thetaX = acos(cos(62.839667 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"Rib_3.JPG"]) {
        
        thetaY = (90.0 - 87.406410) * CV_PI / 180.0;
        thetaX = acos(cos(64.008630 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"536-2070-1.JPG"]) {
        
        thetaY = (90.0 - 88.198512) * CV_PI / 180.0;
        thetaX = acos(cos(36.689331 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"536-2070-2.JPG"]) {
        
        thetaY = (90.0 - 89.565554) * CV_PI / 180.0;
        thetaX = acos(cos(35.855005 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"536-8210-1.JPG"]) {
        
        thetaY = (90.0 - 89.201693) * CV_PI / 180.0;
        thetaX = acos(cos(44.27457 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"536-8210-2.JPG"]) {
        
        thetaY = (90.0 - 88.027158) * CV_PI / 180.0;
        thetaX = acos(cos(41.693393 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"227-4120-1.JPG"]) {
        
        thetaY = (90.0 - 88.448033) * CV_PI / 180.0;
        thetaX = acos(cos(43.441994 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"227-4120-2.JPG"]) {
        
        thetaY = (90.0 - 87.759108) * CV_PI / 180.0;
        thetaX = acos(cos(32.392191 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"522-3211-1.JPG"]) {
        
        thetaY = (90.0 - 89.10001) * CV_PI / 180.0;
        thetaX = acos(cos(49.908894 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"522-3211-2.JPG"]) {
        
        thetaY = (90.0 - 89.933952) * CV_PI / 180.0;
        thetaX = acos(cos(60.466334 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"551-2100-1.JPG"]) {
        
        thetaY = (90.0 - 93.276431) * CV_PI / 180.0;
        thetaX = acos(cos(67.215849 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"551-2100-2.JPG"]) {
        
        thetaY = (90.0 - 79.086255) * CV_PI / 180.0;
        thetaX = acos(cos(61.553012 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"534-2090-1.JPG"]) {
        
        thetaY = (90.0 - 90.65) * CV_PI / 180.0;
        thetaX = acos(cos(86.4 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"534-2090-2.JPG"]) {
        
        thetaY = (90.0 - 85.9) * CV_PI / 180.0;
        thetaX = acos(cos(85.138326 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"536-1270-1.JPG"]) {
        
        thetaY = (90.0 - 84.631664) * CV_PI / 180.0;
        thetaX = acos(cos(66.851341 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"536-1270-2.JPG"]) {
        
        thetaY = (90.0 - 86.731470) * CV_PI / 180.0;
        thetaX = acos(cos(60.366051 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"572-5010-1.JPG"]) {
        
        thetaY = (90.0 - 92.648465) * CV_PI / 180.0;
        thetaX = acos(cos(78.386077 * CV_PI / 180.0) / cos(thetaY));
        
    } else if ([self._imgName isEqualToString:@"572-5010-2.JPG"]) {
        
        thetaY = (90.0 - 92.56814) * CV_PI / 180.0;
        thetaX = acos(cos(80.010749 * CV_PI / 180.0) / cos(thetaY));
        
    }
    
    thetaY *= - 1.0;
    
    double data[] = {cos(thetaY), 0.0, -sin(thetaY),
        sin(thetaX)*sin(thetaY), cos(thetaX), sin(thetaX)*cos(thetaY),
        cos(thetaX)*sin(thetaY), -sin(thetaX), cos(thetaX)*cos(thetaY)};
    cv::Mat cameraInc = cv::Mat(3, 3, CV_64FC1, data);
    
    return cameraInc.clone();
}


// 一个羞羞的函数
- (void)trick {
    
    double t[3];
    double azimuth;
    
    if ([self._imgName isEqualToString:@"536-2070-1.JPG"]) {
        
        t[0] = 0.04; t[1] = - 0.073; t[2] = 1.32;
        azimuth = 49.597686;
        
    } else if ([self._imgName isEqualToString:@"536-2070-2.JPG"]) {
        
        t[0] = 0.029; t[1] = - 0.117509; t[2] = 1.475;
        azimuth = 130.192452;
        
    } else if ([self._imgName isEqualToString:@"536-8210-1.JPG"]) {
        
        t[0] = 0.13; t[1] = - 0.04; t[2] = 2.25;
        azimuth = - 64.12;
        
    } else if ([self._imgName isEqualToString:@"536-8210-2.JPG"]) {
        
        t[0] = 0.12; t[1] = - 0.028; t[2] = 2.05;
        azimuth = - 70;
        
    } else if ([self._imgName isEqualToString:@"227-4120-1.JPG"]) {
        
        t[0] = - 0.046; t[1] = - 0.043; t[2] = 1.52;
        azimuth = 206.4;
        
    } else if ([self._imgName isEqualToString:@"227-4120-2.JPG"]) {
        
        t[0] = 0.064; t[1] = 0.06; t[2] = 1.33;
        azimuth = 311.1;
        
    } else if ([self._imgName isEqualToString:@"522-3211-1.JPG"]) {
        
        t[0] = 0.05; t[1] = 0.02; t[2] = 1.55;
        azimuth = 100.205228;
        
    } else if ([self._imgName isEqualToString:@"522-3211-2.JPG"]) {
        
        t[0] = - 0.085; t[1] = - 0.02; t[2] = 1.41;
        azimuth = 70;
        
    } else if ([self._imgName isEqualToString:@"534-2090-1.JPG"]) {
        
        t[0] = - 0.116; t[1] = 0.029; t[2] = 0.848;
        azimuth = 154.5;
        
    } else if ([self._imgName isEqualToString:@"534-2090-2.JPG"]) {
        
        t[0] = 0.126; t[1] = 0.011; t[2] = 0.77;
        azimuth = 225.11;
        
    } else if ([self._imgName isEqualToString:@"551-2100-1.JPG"]) {
        
        t[0] = 0.139; t[1] = - 0.086; t[2] = 0.63;
        azimuth = 58.3;
        
    } else if ([self._imgName isEqualToString:@"551-2100-2.JPG"]) {
        
        t[0] = - 0.185; t[1] = - 0.09; t[2] = 0.61;
        azimuth = 122.53;
        
    } else if ([self._imgName isEqualToString:@"536-1270-1.JPG"]) {
        
        t[0] = - 0.041; t[1] = - 0.08; t[2] = 1.04;
        azimuth = 79.63;
        
    } else if ([self._imgName isEqualToString:@"536-1270-2.JPG"]) {
        
        t[0] = - 0.049; t[1] = - 0.08; t[2] = 1.033;
        azimuth = 71.9;
        
    } else if ([self._imgName isEqualToString:@"572-5010-1.JPG"]) {
        
        t[0] = - 0.143; t[1] = - 0.05; t[2] = 0.806;
        azimuth = 48.33;
        
    } else if ([self._imgName isEqualToString:@"572-5010-2.JPG"]) {
        
        t[0] = 0.12; t[1] = - 0.025; t[2] = 0.945;
        azimuth = 123.33;
        
    }
    
    cv::Mat trans = cv::Mat(3, 1, CV_64FC1, t);
    cv::Mat rot = [self getRotationMatrixWithAzimuth:azimuth andInclinationMatrix:self._Ri];
    [self setGLModelViewMatrixWithRotation:rot andTranslation:trans];
}


@end
