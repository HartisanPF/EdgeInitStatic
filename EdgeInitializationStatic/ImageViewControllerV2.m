//
//  UIViewController+ImageViewControllerV2.m
//  EdgeInitializationStatic
//
//  Created by Hartisan on 16/1/2.
//  Copyright © 2016年 Hartisan. All rights reserved.
//

#import "ImageViewControllerV2.h"
#import "Rib_Lines.h"

#define IPAD_CAMERA_PARAM_FX 536.84710693359375
#define IPAD_CAMERA_PARAM_FY 536.7637939453125
#define IPAD_CAMERA_PARAM_U 316.23187255859375
#define IPAD_CAMERA_PARAM_V 223.457733154296875
#define VOTE_BOX_INTERVAL 2.0
#define VOTE_BOX_RATE 0.9
#define LINE_VERTICAL_THREASH 4.0
#define LINE_PARALLEL_THRESH 3.0
#define LINE_DISTANCE_THRESH 5.0
#define LINE_ALIGN_RATE 0.2
#define LINE_OVERLAP_PARAL 3.0
#define LINE_OVERLAP_DIST 2.0
#define PRE_ASSUME_X -0.059
#define PRE_ASSUME_Y 0.018
#define PRE_ASSUME_Z 3.3

@implementation ImageViewControllerV2

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
        
        NSDate* tmpStartData = [NSDate date];
        
        // 转换为hsv空间用来筛选颜色
        cv::Mat hsvMat;
        cvtColor(cvImage, hsvMat, CV_RGB2HSV);
        
        // 选出黑色的部分
        cv::Mat blackMask;
        blackMask = [self getMaskfromHSVMat:hsvMat ofImg:self._imgName];
        
        // 灰度化
        cv::Mat gray;
        cv::cvtColor(cvImage, gray, CV_RGB2GRAY);
        //cv::Mat grayROI;
        //gray.copyTo(grayROI, blackMask);
        
        // 根据轮廓面积和位置大体计算平移
        cv::Mat approxTranslation = [self calcApproxTranslationWithColorMask:blackMask];
        
        // 创建BinaryDescriptor
        cv::Ptr<cv::line_descriptor::BinaryDescriptor> bd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
        
        // 用来保存提取出来的直线段
        std::vector<cv::line_descriptor::KeyLine> imgLines;
        std::vector<cv::line_descriptor::KeyLine> imgLinesVertical;   //对应于重力方向上的直线
        std::vector<cv::line_descriptor::KeyLine> imgLinesUnVertical; //其余可用来投票的直线
        
        // 提取直线
        //cv::Mat mask = cv::Mat::ones(cvImage.rows, cvImage.cols, CV_8UC1);
        bd->detect(gray, imgLines, blackMask);
        //bd->detect(grayROI, imgLines, blackMask);
        
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
            //self._classifiedImgLines = [self classifyImgLinesBySlope:imgLinesUnVertical];
            
            // 先计算方位角
            std::vector<double> candidateAzimuths = [self getCandidateAzimuthsWithImgLines:imgLinesUnVertical];
            NSLog(@"candidateAzimuthSize:%lu", candidateAzimuths.size());
            
            // 寻找aligns
            for (int i = 0; i < candidateAzimuths.size(); ++i) {
                
                cv::Mat rotationMatrix = [self getRotationMatrixWithAzimuth:candidateAzimuths[i] andInclinationMatrix:self._Ri];
                std::vector<std::vector<int>> aligns = [self findAlignsWithRotation:rotationMatrix andImgLines:imgLinesUnVertical];
                NSLog(@"alignSize:%lu", aligns.size());
                
                // 如果align的直线数量超过阈值，则认为align成功，继续算位移
                if (aligns.size() >= self._modelLinesVec.size() * LINE_ALIGN_RATE) {
                    
                    cv::Mat translationMatrix = [self calcTransInAligns:aligns withRot:rotationMatrix andImgLines:imgLinesUnVertical];
                    [self setGLModelViewMatrixWithRotation:rotationMatrix  andTranslation:translationMatrix];
                    
                } else {
                    continue;
                }
            }
            
            double deltaTime = [[NSDate date] timeIntervalSinceDate:tmpStartData] * 1000.0;
            NSLog(@"cost time = %f", deltaTime);
            
            // 设置GL环境下的ModelView矩阵
            //[self setGLModelViewMatrixWithCVMatrix:cvMatrix];
            
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
             double t[] = {PRE_ASSUME_X, PRE_ASSUME_Y, PRE_ASSUME_Z};
             cv::Mat trans = cv::Mat(3, 1, CV_64FC1, t);
             cv::Mat rot = [self getRotationMatrixWithAzimuth:candidateAzimuths[0] andInclinationMatrix:self._Ri];
             [self setGLModelViewMatrixWithRotation:rot andTranslation:trans];
            */ 
            
            // 画线
            //cv::cvtColor(gray, gray, CV_GRAY2RGB);
            //cv::line_descriptor::drawKeylines(gray, imgLinesUnVertical, gray, cv::Scalar(255.0, 255.0, 255.0));
            
            /*
             // 把3D模型线根据计算得出的位姿重新画在图像上
             for (int j = 0; j < self._modelLineVertsNum; j += 6) {
             
             //cv::Mat startPoint = [self getProjectionOf3DPointX:self._modelLineVerts[j] Y:self._modelLineVerts[j + 1] Z:self._modelLineVerts[j + 2] withCVMatrix:cvMatrix];
             //cv::Mat endPoint = [self getProjectionOf3DPointX:self._modelLineVerts[j + 3] Y:self._modelLineVerts[j + 4] Z:self._modelLineVerts[j + 5] withCVMatrix:cvMatrix];
             cv::Mat startPoint = [self getProjectionOf3DPointX:self._modelLineVerts[j] Y:self._modelLineVerts[j + 1] Z:self._modelLineVerts[j + 2] withR:rot andT:trans];
             cv::Mat endPoint = [self getProjectionOf3DPointX:self._modelLineVerts[j+3] Y:self._modelLineVerts[j + 4] Z:self._modelLineVerts[j + 5] withR:rot andT:trans];
             cv::Point2d start;
             cv::Point2d end;
             start.x = startPoint.at<double>(0, 0);
             start.y = startPoint.at<double>(1, 0);
             end.x = endPoint.at<double>(0, 0);
             end.y = endPoint.at<double>(1, 0);
             cv::line(gray, start, end, cv::Scalar(0, 255, 0), 2.0);
             }*/
        }
        
        // 显示结果
        self._imgView.image = MatToUIImage(gray);
    }
}


// 获得某图像的目标颜色蒙版,其中S、V的参考值从PS中取得，但PS以255的百分比表示，需转换一下
- (cv::Mat)getMaskfromHSVMat:(cv::Mat)hsvMat ofImg:(NSString*)imgName {
    
    cv::Mat mask;
    cv::Scalar lower;
    cv::Scalar upper;
    
    if ([imgName isEqualToString:@"Rib_3.JPG"]) {
        
        lower = cv::Scalar(0.0, 0.0, 0.0, 0.0);
        upper = cv::Scalar(180.0, 255.0, 85.0, 0.0);
        cv::inRange(hsvMat, lower, upper, mask);
        cv::dilate(mask, mask, cv::Mat(3, 3, CV_8U));
    }
    
    return mask.clone();
}


// 根据轮廓面积和位置大体计算平移
- (cv::Mat)calcApproxTranslationWithColorMask:(cv::Mat)mask {
    
    cv::Mat approxTrans;
    
    cv::Mat edges;
    cv::GaussianBlur(mask, mask, cv::Size(5, 5), 1.5, 1.5);
    cv::Canny(mask, edges, 0, 250, 3, false);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    float maxArea = 0.0;
    int maxIndex = 0;
    for (int i = 0; i < contours.size(); ++i) {
        
        float area = cv::contourArea(contours[i]);
        
        if (area > maxArea) {
            
            maxArea = area;
            maxIndex = i;
        }
    }
    
    double z = 3.3 * sqrtf(30676.0 / maxArea);
    cv::Point2f massCenter = [self getMassCenterOfContour:contours[maxIndex]];
    cv::Mat cameraCoord = [self convertImageCoord2CameraCoordAtU:massCenter.x andV:massCenter.y];
    double x = z * cameraCoord.at<double>(0, 0) / cameraCoord.at<double>(2, 0);
    double y = z * cameraCoord.at<double>(1, 0) / cameraCoord.at<double>(2, 0);
    
    double data[] = {x, y, z};
    approxTrans = cv::Mat(3, 1, CV_64FC1, data);
    
    return approxTrans;
}


// 计算某一轮廓的重心坐标
- (cv::Point2f)getMassCenterOfContour:(std::vector<cv::Point>)contour {
    
    cv::Point2f massCenter;
    cv::Moments allMoments = cv::moments(contour);
    massCenter.x = (float)(allMoments.m10 / allMoments.m00);
    massCenter.y = (float)(allMoments.m01 / allMoments.m00);
    
    return massCenter;
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
        
        // 解析Rib_Lines.h
        for (int i = 0; i < _ribLineVertsNum; i += 6) {
            
            double data[] = {_ribLineVerts[i] - _ribLineVerts[i + 3],
                _ribLineVerts[i + 1] - _ribLineVerts[i + 4],
                _ribLineVerts[i + 2] - _ribLineVerts[i + 5]};
            cv::Mat vec = cv::Mat(3, 1, CV_64FC1, data).clone();
            lines.push_back(vec);
        }
        
        self._modelLineVertsNum = _ribLineVertsNum;
        self._modelLineVerts = _ribLineVerts;
        
    }
    
    return lines;
}


//按照斜率分类梳理图像中的直线
- (std::vector<std::vector<cv::line_descriptor::KeyLine>>)classifyImgLinesBySlope:(std::vector<cv::line_descriptor::KeyLine>)imgLines {
    
    std::vector<std::vector<cv::line_descriptor::KeyLine>> classifiedImgLines;
    
    // 将直线倾角分为6个区间存放，每个区间30°
    std::vector<cv::line_descriptor::KeyLine> area30;
    std::vector<cv::line_descriptor::KeyLine> area60;
    std::vector<cv::line_descriptor::KeyLine> area90;
    std::vector<cv::line_descriptor::KeyLine> area120;
    std::vector<cv::line_descriptor::KeyLine> area150;
    std::vector<cv::line_descriptor::KeyLine> area180;
    
    for (int i = 0; i < imgLines.size(); ++i) {
        
        float slope = imgLines[i].angle * 180.0 / CV_PI;
        
        if (slope < 0.0) {
            
            slope += 180.0;
        }
        
        if (slope < 30.0) {
            
            area30.push_back(imgLines[i]);
            
        } else if (slope >= 30.0 && slope < 60.0) {
            
            area60.push_back(imgLines[i]);
            
        } else if (slope >= 60.0 && slope < 90.0) {
            
            area90.push_back(imgLines[i]);
            
        } else if (slope >= 90.0 && slope < 120.0) {
            
            area120.push_back(imgLines[i]);
            
        } else if (slope >= 120.0 && slope < 150.0) {
            
            area150.push_back(imgLines[i]);
            
        } else if (slope >= 150.0 && slope < 180.0) {
            
            area180.push_back(imgLines[i]);
        }
    }
    
    classifiedImgLines.push_back(area30);
    classifiedImgLines.push_back(area60);
    classifiedImgLines.push_back(area90);
    classifiedImgLines.push_back(area120);
    classifiedImgLines.push_back(area150);
    classifiedImgLines.push_back(area180);
    
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
    }
    
    thetaY *= - 1.0;
    
    double data[] = {cos(thetaY), 0.0, -sin(thetaY),
        sin(thetaX)*sin(thetaY), cos(thetaX), sin(thetaX)*cos(thetaY),
        cos(thetaX)*sin(thetaY), -sin(thetaX), cos(thetaX)*cos(thetaY)};
    cv::Mat cameraInc = cv::Mat(3, 3, CV_64FC1, data);
    
    return cameraInc.clone();
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
                    
                    if (azi1 > 90.0 && azi1 < 180.0) {
                        
                        azimuthAll.push_back(azi1);
                        
                    } else if (azi2 > 90.0 && azi2 < 180.0) {
                        
                        azimuthAll.push_back(azi2);
                        
                    } else if ((azi1 + 180.0) > 90.0 && (azi1 + 180.0) < 180.0) {
                        
                        azimuthAll.push_back(180.0 + azi1);
                        
                    } else if ((azi2 + 180.0) > 90.0 && (azi2 + 180.0) < 180.0) {
                        
                        azimuthAll.push_back(180.0 + azi2);
                    }
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


// 寻找投影线与图像检测线能align起来的配对
- (std::vector<std::vector<int>>)findAlignsWithRotation:(cv::Mat)rotationMatrix andImgLines:(std::vector<cv::line_descriptor::KeyLine>)imgLines {
    
    std::vector<std::vector<int>> aligns;
    
    // 先预平移矩阵
    double trans[] = {PRE_ASSUME_X, PRE_ASSUME_Y, PRE_ASSUME_Z};
    cv::Mat translate = cv::Mat(3, 1, CV_64FC1, trans);
    
    // 对每条3D线，计算其投影线并寻找配对直线
    cv::Point2f projPtStart;
    cv::Point2f projPtEnd;
    cv::Point2f imgPtStart;
    cv::Point2f imgPtEnd;
    
    for (int i = 0; i < self._modelLinesVec.size(); ++i) {
        
        std::vector<int> align;
        align.push_back(i);
        
        // 计算两端点投影
        projPtStart.x = [self getProjectionOf3DPointX:self._modelLineVerts[i * 6] Y:self._modelLineVerts[i * 6 + 1] Z:self._modelLineVerts[i * 6 + 2] withR:rotationMatrix andT:translate].at<double>(0, 0);
        projPtStart.y = [self getProjectionOf3DPointX:self._modelLineVerts[i * 6] Y:self._modelLineVerts[i * 6 + 1] Z:self._modelLineVerts[i * 6 + 2] withR:rotationMatrix andT:translate].at<double>(1, 0);
        projPtEnd.x = [self getProjectionOf3DPointX:self._modelLineVerts[i * 6 + 3] Y:self._modelLineVerts[i * 6 + 4] Z:self._modelLineVerts[i * 6 + 5] withR:rotationMatrix andT:translate].at<double>(0, 0);
        projPtEnd.y = [self getProjectionOf3DPointX:self._modelLineVerts[i * 6 + 3] Y:self._modelLineVerts[i * 6 + 4] Z:self._modelLineVerts[i * 6 + 5] withR:rotationMatrix andT:translate].at<double>(1, 0);
        
        // 寻找配对
        for (int j = 1; j < imgLines.size(); ++j) {
            
            imgPtStart.x = imgLines[j].startPointX;
            imgPtStart.y = imgLines[j].startPointY;
            imgPtEnd.x = imgLines[j].endPointX;
            imgPtEnd.y = imgLines[j].endPointY;
            
            // 如果配对成功
            if ([self isProjectedLineDetermByPoint:projPtStart andPoint:projPtEnd alignedWithImgLineDetermByPoint:imgPtStart andPoint:imgPtEnd]) {
                
                align.push_back(j);
                
            } else {
                continue;
            }
        }
        
        if (align.size() > 1) {
            
            aligns.push_back(align);
        }
    }
    
    return aligns;
}


// 判断两条直线是否是align的
- (bool)isProjectedLineDetermByPoint:(cv::Point2f)projPtS andPoint:(cv::Point2f)projPtE alignedWithImgLineDetermByPoint:(cv::Point2f)imgPtS andPoint:(cv::Point2f)imgPtE {
    
    bool result = false;
    
    float projSlope = atanf((projPtS.y - projPtE.y) / (projPtS.x - projPtE.x)) * 180.0 / CV_PI;
    if (projSlope < 0.0)
        projSlope += 180.0;
    
    float imgSlope = atanf((imgPtS.y - imgPtE.y) / (imgPtS.x - imgPtE.x)) * 180.0 / CV_PI;
    if (imgSlope < 0.0)
        imgSlope += 180.0;
    
    // 如果近似平行
    if (fabsf(projSlope - imgSlope) <= LINE_PARALLEL_THRESH) {
        
        // 如果足够接近
        if ([self distanceFromPoint:imgPtE ToLineDeterminedByPoint:projPtS andPoint:projPtE] <= LINE_DISTANCE_THRESH) {
            
            // 如果在垂线方向有重叠
            if (((imgPtS.x + imgPtE.x) / 2.0 > MIN(projPtS.x, projPtE.x) && (imgPtS.x + imgPtE.x) / 2.0 < MAX(projPtS.x, projPtE.x)) || ((imgPtS.y + imgPtE.y) / 2.0 > MIN(projPtS.y, projPtE.y) && (imgPtS.y + imgPtE.y) / 2.0 < MAX(projPtS.y, projPtE.y))) {
                
                result = true;
            }
        }
    }
    
    return result;
}


// 从align中计算平移
- (cv::Mat)calcTransInAligns:(std::vector<std::vector<int>>)aligns withRot:(cv::Mat)rotation andImgLines:(std::vector<cv::line_descriptor::KeyLine>)imgLines {
    
    cv::Mat translation;
    std::vector<std::vector<int>> uniqueAligns;
    std::vector<int> threeModelLinesIndex;
    std::vector<cv::line_descriptor::KeyLine> threeImgLines;
    
    // 先把只有一条配对线的组合挑出来
    for (int i = 0; i < aligns.size(); ++i) {
        
        if (aligns[i].size() == 2) {
            
            uniqueAligns.push_back(aligns[i]);
        }
    }
    
    // 如果超过3对，则直接计算试试先
    if (uniqueAligns.size() >= 3) {
        
        int index0 = -1;
        int index1 = -1;
        int index2 = -1;
        bool over = false;
        
        for (int i = 0; i < uniqueAligns.size(); ++i) {
            
            if (over)
                break;
            
            for (int j = i + 1; j < uniqueAligns.size(); ++j) {
                
                if (![self isModelVecAtIndex:uniqueAligns[i][0] ParallelWithVecAtIndex:uniqueAligns[j][0]]) {
                    
                    if (over)
                        break;
                    
                    for (int k = j + 1; k < uniqueAligns.size(); ++k) {
                        
                        if (![self isModelVecAtIndex:uniqueAligns[i][0] ParallelWithVecAtIndex:uniqueAligns[k][0]] && ![self isModelVecAtIndex:uniqueAligns[j][0] ParallelWithVecAtIndex:uniqueAligns[k][0]]) {
                            
                            index0 = i;
                            index1 = j;
                            index2 = k;
                            over = true;
                            break;
                        }
                    }
                }
            }
        }
        
        if (index2 != -1) {
            
            threeModelLinesIndex.push_back(uniqueAligns[index0][0]);
            threeModelLinesIndex.push_back(uniqueAligns[index1][0]);
            threeModelLinesIndex.push_back(uniqueAligns[index2][0]);
            threeImgLines.push_back(imgLines[uniqueAligns[index0][1]]);
            threeImgLines.push_back(imgLines[uniqueAligns[index1][1]]);
            threeImgLines.push_back(imgLines[uniqueAligns[index2][1]]);
            
            translation = [self getTranslationWithThreePairsOfModelLines:threeModelLinesIndex andImgLines:threeImgLines andRotationMatrix:rotation];
            //int score = [self calcPoseScoreWithRot:rotation andTrans:translation andAligns:aligns andImgLines:imgLines];
            //NSLog(@"score: %d", score);
        }
        
    }
    
    
    return translation;
}


// 判断两3D直线是否平行
- (bool)isModelVecAtIndex:(int)indexA ParallelWithVecAtIndex:(int)indexB {
    
    bool result = false;
    
    cv::Mat vecA = self._modelLinesVec[indexA];
    cv::Mat vecB = self._modelLinesVec[indexB];
    
    float product = vecA.dot(vecB);
    float lengthA = sqrt(vecA.dot(vecA));
    float lengthB = sqrt(vecB.dot(vecB));
    float angle = acos(product / (lengthA * lengthB)) * 180.0 / CV_PI;
    
    if (fabs(angle - 0.0) < 1.0 || fabs(angle - 180.0) < 1.0) {
        
        result = true;
    }
    
    return result;
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
    double p1[] = {self._modelLineVerts[modelLinesIndex[0]*6], self._modelLineVerts[modelLinesIndex[0]*6 + 1], self._modelLineVerts[modelLinesIndex[0]*6 + 2]};
    double p2[] = {self._modelLineVerts[modelLinesIndex[1]*6], self._modelLineVerts[modelLinesIndex[1]*6 + 1], self._modelLineVerts[modelLinesIndex[1]*6 + 2]};
    double p3[] = {self._modelLineVerts[modelLinesIndex[2]*6], self._modelLineVerts[modelLinesIndex[2]*6 + 1], self._modelLineVerts[modelLinesIndex[2]*6 + 2]};
    
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


// 根据线条重合数计算某个位姿的分数
- (int)calcPoseScoreWithRot:(cv::Mat)rot andTrans:(cv::Mat)trans andAligns:(std::vector<std::vector<int>>)aligns andImgLines:(std::vector<cv::line_descriptor::KeyLine>)imgLines {
    
    int score = 0;
    
    // 对每条已经找到align的3D线，计算其投影线并寻找重合直线
    cv::Point2f projPtStart;
    cv::Point2f projPtEnd;
    cv::Point2f imgPtStart;
    cv::Point2f imgPtEnd;
    
    for (int i = 0; i < aligns.size(); ++i) {
        
        // 计算两端点投影
        projPtStart.x = [self getProjectionOf3DPointX:self._modelLineVerts[aligns[i][0] * 6] Y:self._modelLineVerts[aligns[i][0] * 6 + 1] Z:self._modelLineVerts[aligns[i][0] * 6 + 2] withR:rot andT:trans].at<double>(0, 0);
        projPtStart.y = [self getProjectionOf3DPointX:self._modelLineVerts[aligns[i][0] * 6] Y:self._modelLineVerts[aligns[i][0] * 6 + 1] Z:self._modelLineVerts[aligns[i][0] * 6 + 2] withR:rot andT:trans].at<double>(1, 0);
        projPtEnd.x = [self getProjectionOf3DPointX:self._modelLineVerts[aligns[i][0] * 6 + 3] Y:self._modelLineVerts[aligns[i][0] * 6 + 4] Z:self._modelLineVerts[aligns[i][0] * 6 + 5] withR:rot andT:trans].at<double>(0, 0);
        projPtEnd.y = [self getProjectionOf3DPointX:self._modelLineVerts[aligns[i][0] * 6 + 3] Y:self._modelLineVerts[aligns[i][0] * 6 + 4] Z:self._modelLineVerts[aligns[i][0] * 6 + 5] withR:rot andT:trans].at<double>(1, 0);
        
        // 寻找重合
        for (int j = 1; j < aligns[i].size(); ++j) {
            
            imgPtStart.x = imgLines[aligns[i][j]].startPointX;
            imgPtStart.y = imgLines[aligns[i][j]].startPointY;
            imgPtEnd.x = imgLines[aligns[i][j]].endPointX;
            imgPtEnd.y = imgLines[aligns[i][j]].endPointY;
            
            // 如果重合
            if ([self isProjectedLineDetermByPoint:projPtStart andPoint:projPtEnd overlapWithImgLineDetermByPoint:imgPtStart andPoint:imgPtEnd]) {
                
                score++;
                break;
            }
        }
    }
    
    return score;
}


// 判断两直线是否重合
- (bool)isProjectedLineDetermByPoint:(cv::Point2f)projPtS andPoint:(cv::Point2f)projPtE overlapWithImgLineDetermByPoint:(cv::Point2f)imgPtS andPoint:(cv::Point2f)imgPtE {
    
    bool result = false;
    
    float projSlope = atanf((projPtS.y - projPtE.y) / (projPtS.x - projPtE.x)) * 180.0 / CV_PI;
    if (projSlope < 0.0)
        projSlope += 180.0;
    
    float imgSlope = atanf((imgPtS.y - imgPtE.y) / (imgPtS.x - imgPtE.x)) * 180.0 / CV_PI;
    if (imgSlope < 0.0)
        imgSlope += 180.0;
    
    // 如果近似平行
    if (fabsf(projSlope - imgSlope) <= LINE_OVERLAP_PARAL) {
        
        // 如果足够接近
        if ([self distanceFromPoint:imgPtE ToLineDeterminedByPoint:projPtS andPoint:projPtE] <= LINE_OVERLAP_DIST) {
            
            // 如果在垂线方向有重叠
            if (((imgPtS.x + imgPtE.x) / 2.0 > MIN(projPtS.x, projPtE.x) && (imgPtS.x + imgPtE.x) / 2.0 < MAX(projPtS.x, projPtE.x)) || ((imgPtS.y + imgPtE.y) / 2.0 > MIN(projPtS.y, projPtE.y) && (imgPtS.y + imgPtE.y) / 2.0 < MAX(projPtS.y, projPtE.y))) {
                
                result = true;
            }
        }
    }
    
    return result;
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


// 平面上一点到直线的距离（直线由两点决定）
- (float)distanceFromPoint:(cv::Point2f)ptC ToLineDeterminedByPoint:(cv::Point2f)ptA andPoint:(cv::Point2f)ptB {
    
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

@end
