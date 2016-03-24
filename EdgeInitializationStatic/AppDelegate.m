//
//  AppDelegate.m
//  EdgeInitializationStatic
//
//  Created by Hartisan on 15/10/4.
//  Copyright © 2015年 Hartisan. All rights reserved.
//

#import "AppDelegate.h"
#import "ImageViewControllerV2.h"

@interface AppDelegate ()

@end

@implementation AppDelegate


- (BOOL)application:(UIApplication *)application didFinishLaunchingWithOptions:(NSDictionary *)launchOptions {
    
    self.window = [[UIWindow alloc] initWithFrame:[[UIScreen mainScreen] bounds]];
    
    ImageViewControllerV2* imgViewController = [[ImageViewControllerV2 alloc] initWithNibName:@"ImageViewControllerV2" bundle:nil];
    self.window.rootViewController = imgViewController;
    
    [self.window makeKeyAndVisible];
    return YES;
}


@end
