/*
  This file is part of the Structure SDK.
  Copyright © 2016 Occipital, Inc. All rights reserved.
  http://structure.io
*/

#import "ViewController.h"
#import "ViewController+Camera.h"
#import "ViewController+Sensor.h"
#import "ViewController+SLAM.h"

#import <Structure/Structure.h>
#import <Structure/StructureSLAM.h>

#import <objc/runtime.h>

@implementation ViewController (Camera)

#pragma mark -  Color Camera

- (bool)queryCameraAuthorizationStatusAndNotifyUserIfNotGranted
{
    const NSUInteger numCameras = [[AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo] count];
    
    if (0 == numCameras)
        return false; // This can happen even on devices that include a camera, when camera access is restricted globally.

    AVAuthorizationStatus authStatus = [AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo];
    
    if (authStatus != AVAuthorizationStatusAuthorized)
    {
        NSLog(@"Not authorized to use the camera!");
        
        [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo
                                 completionHandler:^(BOOL granted)
         {
             // This block fires on a separate thread, so we need to ensure any actions here
             // are sent to the right place.
             
             // If the request is granted, let's try again to start an AVFoundation session.
             // Otherwise, alert the user that things won't go well.
             if (granted)
             {
                 dispatch_async(dispatch_get_main_queue(), ^(void) {
                     
                     [self startColorCamera];
                     
                     _appStatus.colorCameraIsAuthorized = true;
                     [self updateAppStatusMessage];
                     
                 });
             }
         }];
        
        return false;
    }
    
    return true;
}

- (void)setupColorCamera
{
    // If already setup, skip it
    if (self.avCaptureSession)
        return;
    
    bool cameraAccessAuthorized = [self queryCameraAuthorizationStatusAndNotifyUserIfNotGranted];
    
    if (!cameraAccessAuthorized)
    {
        _appStatus.colorCameraIsAuthorized = false;
        [self updateAppStatusMessage];
        return;
    }
    
    // Use VGA color.
    NSString *sessionPreset = AVCaptureSessionPreset640x480;//AVCaptureSessionPreset640x480;  changed tanaka
    
    // Set up Capture Session.
    self.avCaptureSession = [[AVCaptureSession alloc] init];
    [self.avCaptureSession beginConfiguration];
    
    // Set preset session size.
    //[self.avCaptureSession setSessionPreset:sessionPreset];
    
    // InputPriority allows us to select a more precise format (below)
    [self.avCaptureSession setSessionPreset:AVCaptureSessionPresetInputPriority];
    
    // Create a video device and input from that Device.  Add the input to the capture session.
    self.videoDevice = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
    if (self.videoDevice == nil)
        assert(0);
    
    // Configure Focus, Exposure, and White Balance
    NSError *error;
    
    // Use auto-exposure, and auto-white balance and set the focus to infinity.
    if([self.videoDevice lockForConfiguration:&error])
    {
        // add by tanaka for more color resolution ----------------------------------------------
        int imageWidth = -1;
        int imageHeight = -1;
        
        if (false)//self.enableHighResolutionColorSwitch.on)
        {
            // High-resolution uses 2592x1936, which is close to a 4:3 aspect ratio.
            // Other aspect ratios such as 720p or 1080p are not yet supported.
            imageWidth = 2592;
            imageHeight = 1936;
        }
        else
        {
            // Low resolution uses VGA.
            imageWidth = 640;
            imageHeight = 480;
        }
        
        // Select capture format
        [self selectCaptureFormat:@{ @"width": @(imageWidth),
                                     @"height": @(imageHeight)}];
        // ----------------------------------------------
        

        
        
        // Allow exposure to initially change
        if ([self.videoDevice isExposureModeSupported:AVCaptureExposureModeContinuousAutoExposure])
            [self.videoDevice setExposureMode:AVCaptureExposureModeContinuousAutoExposure];
        
        // Allow white balance to initially change
        if ([self.videoDevice isWhiteBalanceModeSupported:AVCaptureWhiteBalanceModeContinuousAutoWhiteBalance])
            [self.videoDevice setWhiteBalanceMode:AVCaptureWhiteBalanceModeContinuousAutoWhiteBalance];
        
        // We need to keep focus fixed during tracking.
        [self.videoDevice setFocusModeLockedWithLensPosition:_options.colorCameraLensPosition completionHandler:nil];
        
        [self.videoDevice unlockForConfiguration];
    }
    
    //  Add the device to the session.
    AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:self.videoDevice error:&error];
    if (error)
    {
        NSLog(@"Cannot initialize AVCaptureDeviceInput");
        assert(0);
    }
    
    [self.avCaptureSession addInput:input]; // After this point, captureSession captureOptions are filled.
    
    //  Create the output for the capture session.
    AVCaptureVideoDataOutput* dataOutput = [[AVCaptureVideoDataOutput alloc] init];
    
    // We don't want to process late frames.
    [dataOutput setAlwaysDiscardsLateVideoFrames:YES];
    
    // Use  YCbCr pixel format.
    [dataOutput setVideoSettings:[NSDictionary
                                  dictionaryWithObject:[NSNumber numberWithInt:kCVPixelFormatType_420YpCbCr8BiPlanarFullRange]
                                  forKey:(id)kCVPixelBufferPixelFormatTypeKey]];
    
    // Set dispatch to be on the main thread so OpenGL can do things with the data
    [dataOutput setSampleBufferDelegate:self queue:dispatch_get_main_queue()];
    
    [self.avCaptureSession addOutput:dataOutput];
    
    if([self.videoDevice lockForConfiguration:&error])
    {
        [self.videoDevice setActiveVideoMaxFrameDuration:CMTimeMake(1, 30)];
        [self.videoDevice setActiveVideoMinFrameDuration:CMTimeMake(1, 30)];
        [self.videoDevice unlockForConfiguration];
    }
    
    [self.avCaptureSession commitConfiguration];
}

- (void)startColorCamera
{
    if (self.avCaptureSession && [self.avCaptureSession isRunning])
        return;
    
    // Re-setup so focus is lock even when back from background
    if (self.avCaptureSession == nil)
        [self setupColorCamera];
    
    // Start streaming color images.
    [self.avCaptureSession startRunning];
}

- (void)stopColorCamera
{
    if ([self.avCaptureSession isRunning])
    {
        // Stop the session
        [self.avCaptureSession stopRunning];
    }
    
    self.avCaptureSession = nil;
    self.videoDevice = nil;
}

- (void)setColorCameraParametersForInit
{
    NSError *error;
    
    [self.videoDevice lockForConfiguration:&error];
    
    // Auto-exposure
    if ([self.videoDevice isExposureModeSupported:AVCaptureExposureModeContinuousAutoExposure])
        [self.videoDevice setExposureMode:AVCaptureExposureModeContinuousAutoExposure];
    
    // Auto-white balance.
    if ([self.videoDevice isWhiteBalanceModeSupported:AVCaptureWhiteBalanceModeContinuousAutoWhiteBalance])
        [self.videoDevice setWhiteBalanceMode:AVCaptureWhiteBalanceModeContinuousAutoWhiteBalance];
    
    [self.videoDevice unlockForConfiguration];
    
}

- (void)setColorCameraParametersForScanning
{
    NSError *error;
    
    [self.videoDevice lockForConfiguration:&error];
    
    // Exposure locked to its current value.
    if([self.videoDevice isExposureModeSupported:AVCaptureExposureModeLocked])
        [self.videoDevice setExposureMode:AVCaptureExposureModeLocked];
    
    // White balance locked to its current value.
    if([self.videoDevice isWhiteBalanceModeSupported:AVCaptureWhiteBalanceModeLocked])
        [self.videoDevice setWhiteBalanceMode:AVCaptureWhiteBalanceModeLocked];
    
    [self.videoDevice unlockForConfiguration];
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection
{
    // Pass color buffers directly to the driver, which will then produce synchronized depth/color pairs.
    [_sensorController frameSyncNewColorBuffer:sampleBuffer];
}

// add by tanaka from scanner ------------------------------
// キャプチャフォーマットの選択
- (void)selectCaptureFormat:(NSDictionary*)demandFormat
{
    AVCaptureDeviceFormat * selectedFormat = nil;
    
    for (AVCaptureDeviceFormat* format in self.videoDevice.formats)
    {
        double formatMaxFps = ((AVFrameRateRange *)[format.videoSupportedFrameRateRanges objectAtIndex:0]).maxFrameRate;
        
        CMFormatDescriptionRef formatDesc = format.formatDescription;
        FourCharCode fourCharCode = CMFormatDescriptionGetMediaSubType(formatDesc);
        
        CMVideoFormatDescriptionRef videoFormatDesc = formatDesc;
        CMVideoDimensions formatDims = CMVideoFormatDescriptionGetDimensions(videoFormatDesc);
        
        NSNumber * widthNeeded  = demandFormat[@"width"];
        NSNumber * heightNeeded = demandFormat[@"height"];
        
        if ( widthNeeded && widthNeeded .intValue!= formatDims.width )
            continue;
        
        if( heightNeeded && heightNeeded.intValue != formatDims.height )
            continue;
        
        // we only support full range YCbCr for now
        if(fourCharCode != (FourCharCode)'420f')
            continue;
        
        
        selectedFormat = format;
        break;
    }
    
    self.videoDevice.activeFormat = selectedFormat;
}


@end
