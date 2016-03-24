//
//  UIViewController+GLKViewController.h
//  EdgeInitializationStatic
//
//  Created by Hartisan on 15/10/18.
//  Copyright © 2015年 Hartisan. All rights reserved.
//

#import <GLKit/GLKit.h>
#import <OpenGLES/ES1/gl.h>
#import <OpenGLES/ES1/glext.h>
#import <OpenGLES/ES2/gl.h>
#import <OpenGLES/ES2/glext.h>

@interface GLViewController : GLKViewController {
    
    EAGLContext* _context;
    GLKBaseEffect* _effect;
    GLKVector4 _renderColor;
    GLKMatrix4 _projectionMatrix;
    GLKMatrix4 _modelViewMatrix;
    NSString* _modelName;
    int _modelNumVerts;
    float* _modelVerts;
    float* _modelNormals;
}

@property (nonatomic, strong) EAGLContext* _context;
@property (nonatomic, strong) GLKBaseEffect* _effect;
@property GLKVector4 _renderColor;
@property GLKMatrix4 _projectionMatrix;
@property GLKMatrix4 _modelViewMatrix;
@property (nonatomic, strong) NSString* _modelName;
@property int _modelNumVerts;
@property float* _modelVerts;
@property float* _modelNormals;

- (void)initGL;
- (void)setupGL;
- (void)tearDownGL;
- (void)getProjectionMatrix;
- (void)setModelNameWithImg:(NSString*)imgName;

@end
