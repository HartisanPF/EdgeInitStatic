//
//  UIViewController+GLKViewController.m
//  EdgeInitializationStatic
//
//  Created by Hartisan on 15/10/18.
//  Copyright © 2015年 Hartisan. All rights reserved.
//

#import "GLViewController.h"
#import "ribWire_invert.h"
#import "ribMesh_invert.h"
#import "mesh_536A2070.h"
#import "mesh_536A8210.h"
#import "mesh_227A4120.h"
#import "mesh_522A3211.h"
#import "mesh_534A2090.h"
#import "mesh_551A2100.h"
#import "mesh_536A1270.h"
#import "mesh_572A5010.h"

@interface GLViewController ()

@end

@implementation GLViewController

@synthesize _context, _effect, _renderColor, _projectionMatrix, _modelViewMatrix, _modelName, _modelNumVerts, _modelVerts, _modelNormals;


- (void)viewDidLoad {
    
    [super viewDidLoad];
    
    [self initGL];
    [self setupGL];
}


// 初始化GL
- (void)initGL {
    
    // 上下文
    EAGLContext* eaglContext = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
    self._context = eaglContext;
    
    GLKView* view = (GLKView *)self.view;
    view.context = self._context;
    view.drawableColorFormat = GLKViewDrawableColorFormatRGBA8888;
    view.drawableDepthFormat = GLKViewDrawableDepthFormat24;
    [view.layer setNeedsDisplay];
    
    // 将view设置为透明
    [self.view setBackgroundColor:[UIColor clearColor]];
    [self.view setOpaque:YES];
    
    // 设置GL视图刷新频率
    self.preferredFramesPerSecond = 25;
}

// 配置GL
- (void)setupGL {
    
    [EAGLContext setCurrentContext:self._context];
    
    GLKBaseEffect* baseEffect = [[GLKBaseEffect alloc] init];
    self._effect = baseEffect;
    
    // 设置绘制颜色
    self._renderColor = GLKVector4Make(0.0, 1.0, 0.0, 0.0);
    
    // 计算投影矩阵
    [self getProjectionMatrix];
}


// 设置当前需要加载哪个模型,并读入模型数据
- (void)setModelNameWithImg:(NSString*)imgName {
    
    if ([imgName isEqualToString:@"Rib_1.JPG"] || [imgName isEqualToString:@"Rib_2.JPG"] || [imgName isEqualToString:@"Rib_3.JPG"]) {
        
        self._modelName = @"rib";
        self._modelVerts = ribWire_invertVerts;
        self._modelNormals = ribWire_invertNormals;
        self._modelNumVerts = ribWire_invertNumVerts;
        
    } else if ([imgName isEqualToString:@"536-2070-1.JPG"] || [imgName isEqualToString:@"536-2070-2.JPG"]) {
        
        self._modelName = @"536A2070";
        self._modelVerts = mesh_536A2070Verts;
        self._modelNormals = mesh_536A2070Normals;
        self._modelNumVerts = mesh_536A2070NumVerts;
        
    } else if ([imgName isEqualToString:@"536-8210-1.JPG"] || [imgName isEqualToString:@"536-8210-2.JPG"]) {
        
        self._modelName = @"536A8210";
        self._modelVerts = mesh_536A8210Verts;
        self._modelNormals = mesh_536A8210Normals;
        self._modelNumVerts = mesh_536A8210NumVerts;
        
    } else if ([imgName isEqualToString:@"227-4120-1.JPG"] || [imgName isEqualToString:@"227-4120-2.JPG"]) {
        
        self._modelName = @"227A4120";
        self._modelVerts = mesh_227A4120Verts;
        self._modelNormals = mesh_227A4120Normals;
        self._modelNumVerts = mesh_227A4120NumVerts;
        
    } else if ([imgName isEqualToString:@"522-3211-1.JPG"] || [imgName isEqualToString:@"522-3211-2.JPG"]) {
        
        self._modelVerts = mesh_522A3211Verts;
        self._modelNormals = mesh_522A3211Normals;
        self._modelNumVerts = mesh_522A3211NumVerts;
        
    } else if ([imgName isEqualToString:@"534-2090-1.JPG"] || [imgName isEqualToString:@"534-2090-2.JPG"]) {
        
        self._modelVerts = mesh_534A2090Verts;
        self._modelNormals = mesh_534A2090Normals;
        self._modelNumVerts = mesh_534A2090NumVerts;
        
    } else if ([imgName isEqualToString:@"551-2100-1.JPG"] || [imgName isEqualToString:@"551-2100-2.JPG"]) {
        
        self._modelVerts = mesh_551A2100Verts;
        self._modelNormals = mesh_551A2100Normals;
        self._modelNumVerts = mesh_551A2100NumVerts;
        
    } else if ([imgName isEqualToString:@"536-1270-1.JPG"] || [imgName isEqualToString:@"536-1270-2.JPG"]) {
        
        self._modelVerts = mesh_536A1270Verts;
        self._modelNormals = mesh_536A1270Normals;
        self._modelNumVerts = mesh_536A1270NumVerts;
        
    } else if ([imgName isEqualToString:@"572-5010-1.JPG"] || [imgName isEqualToString:@"572-5010-2.JPG"]) {
        
        self._modelVerts = mesh_572A5010Verts;
        self._modelNormals = mesh_572A5010Normals;
        self._modelNumVerts = mesh_572A5010NumVerts;
        
    }
}

// 刷新
- (void)update {
    
    // 更新投影矩阵
    self._effect.transform.projectionMatrix = self._projectionMatrix;
    
    // 物体
    self._effect.transform.modelviewMatrix = self._modelViewMatrix;
}


// 绘制
- (void)glkView:(GLKView *)view drawInRect:(CGRect)rect {
    
    // 清屏
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // 采用固定颜色
    self._effect.useConstantColor = YES;
    self._effect.constantColor = self._renderColor;
    
    glEnableVertexAttribArray(GLKVertexAttribPosition);
    glEnableVertexAttribArray(GLKVertexAttribNormal);
    
    glVertexAttribPointer(GLKVertexAttribPosition, 3, GL_FLOAT, GL_FALSE, 0, self._modelVerts);
    glVertexAttribPointer(GLKVertexAttribNormal, 3, GL_FLOAT, GL_FALSE, 0, self._modelNormals);
    [self._effect prepareToDraw];
    glDrawArrays(GL_TRIANGLES, 0, self._modelNumVerts);
    
    glDisableVertexAttribArray(GLKVertexAttribPosition);
    glDisableVertexAttribArray(GLKVertexAttribNormal);
}


// 根据摄像头内参、分辨率等参数计算projectionMatrix
- (void)getProjectionMatrix {
    
    float matrix[16];
    float fx = 536.84710693359375;
    float fy = 536.7637939453125;
    float cx = 316.23187255859375;
    float cy = 223.457733154296875;
    float width = 640.0;
    float height = 480.0;
    float near = 0.001;
    float far = 100.0;
    
    matrix[0] = 2.0f * fx / width;
    matrix[1] = 0.0f;
    matrix[2] = 0.0f;
    matrix[3] = 0.0f;
    
    matrix[4] = 0.0f;
    matrix[5] = 2.0f * fy / height;
    matrix[6] = 0.0f;
    matrix[7] = 0.0f;
    
    matrix[8] = 1.0f - 2.0 * cx / width;
    matrix[9] = 2.0f * cy / height - 1.0f;
    matrix[10] = - (far + near) / (far - near);
    matrix[11] = - 1.0f;
    
    matrix[12] = 0.0f;
    matrix[13] = 0.0f;
    matrix[14] = - 2.0f * far * near / (far - near);
    matrix[15] = 0.0f;
    
    self._projectionMatrix = GLKMatrix4MakeWithArray(matrix);
}


- (void)tearDownGL {
    
    if ([EAGLContext currentContext] == self._context)
        [EAGLContext setCurrentContext:nil];
    self._context = nil;
    self._effect = nil;
}


- (void)viewDidUnload {
    
    [self tearDownGL];
    [super viewDidUnload];
}

@end
