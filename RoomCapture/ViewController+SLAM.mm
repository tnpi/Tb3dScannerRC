/*
 This file is part of the Structure SDK.
 Copyright © 2016 Occipital, Inc. All rights reserved.
 http://structure.io
 */

#import "ViewController.h"
#import "ViewController+OpenGL.h"

#import <Structure/Structure.h>
#import <Structure/StructureSLAM.h>

#pragma mark - Utilities

namespace // anonymous namespace for local functions
{
    float deltaRotationAngleBetweenPosesInDegrees (const GLKMatrix4& previousPose, const GLKMatrix4& newPose)
    {
        GLKMatrix4 deltaPose = GLKMatrix4Multiply(newPose,
                                                  // Transpose is equivalent to inverse since we will only use the rotation part.
                                                  GLKMatrix4Transpose(previousPose));
        
        // Get the rotation component of the delta pose
        GLKQuaternion deltaRotationAsQuaternion = GLKQuaternionMakeWithMatrix4(deltaPose);
        
        // Get the angle of the rotation
        const float angleInDegree = GLKQuaternionAngle(deltaRotationAsQuaternion)*180.f/M_PI;
        
        return angleInDegree;
    }
    
    NSString* computeTrackerMessage (STTrackerHints hints)
    {
        if (hints.trackerIsLost) {
            return @"Tracking Lost! Please Realign or Press Reset.";
        }
        
        if (hints.modelOutOfView)
            return @"Please put the model back in view.";
        
        if (hints.sceneIsTooClose)
            return @"Too close to the scene! Please step back.";
        
        return nil;
    }
}

@implementation ViewController (SLAM)

#pragma mark - SLAM

// Setup SLAM related objects.
- (void)setupSLAM
{
    if (_slamState.initialized)
        return;
    
    // Initialize the scene.
    _slamState.scene = [[STScene alloc] initWithContext:_display.context
                                      freeGLTextureUnit:GL_TEXTURE2];
    
    // Initialize the camera pose tracker.
    NSDictionary* trackerOptions = @{
                                     kSTTrackerTypeKey: @(STTrackerDepthAndColorBased),
                                     kSTTrackerTrackAgainstModelKey: self.trackingSmallObjectSwitch.isOn ? @YES:@NO, // Tracking against model works better in smaller scale scanning.
                                     kSTTrackerQualityKey: self.trackerQualityAccurateSwitch.isOn ?  @(STTrackerQualityAccurate):@(STTrackerQualityFast),
                                     
                                     // add tanaka
                                     kSTTrackerBackgroundProcessingEnabledKey:@YES,     // defaults: NO
                                     };
    
    // Initialize the camera pose tracker.
    _slamState.tracker = [[STTracker alloc] initWithScene:_slamState.scene options:trackerOptions];
    
    // Default volume size set in options struct
    _slamState.volumeSizeInMeters = _options.initialVolumeSizeInMeters;
    
    // Setup the camera placement initializer. We will set it to the center of the volume to
    // maximize the area of scan. The rotation will also be aligned to gravity.
    _slamState.cameraPoseInitializer = [[STCameraPoseInitializer alloc]
                                        initWithVolumeSizeInMeters:_slamState.volumeSizeInMeters
                                        options:@{kSTCameraPoseInitializerStrategyKey:@(STCameraPoseInitializerStrategyGravityAlignedAtVolumeCenter)}
                                        //options:@{kSTCameraPoseInitializerStrategyKey:@(STCameraPoseInitializerStrategyGravityAlignedAtOrigin)}
    ];
    //_slamState.cameraPoseInitializer.
     
    // Setup the initial volume size.
    [self adjustVolumeSize:_slamState.volumeSizeInMeters];
    
    // Start with cube placement mode
    [self enterPoseInitializationState];
    
    NSDictionary* keyframeManagerOptions = @{
                                             kSTKeyFrameManagerMaxSizeKey: @(_options.maxNumKeyframes),
                                             kSTKeyFrameManagerMaxDeltaTranslationKey: @(_options.maxKeyFrameTranslation),
                                             kSTKeyFrameManagerMaxDeltaRotationKey: @(_options.maxKeyFrameRotation),
                                             };
    
    _slamState.keyFrameManager = [[STKeyFrameManager alloc] initWithOptions:keyframeManagerOptions];
    
    _slamState.initialized = true;
    
}

- (void)resetSLAM
{
    _slamState.prevFrameTimeStamp = -1.0;
    [_slamState.mapper reset];
    [_slamState.tracker reset];
    [_slamState.scene clear];
    [_slamState.keyFrameManager clear];
    
    _colorizedMesh = nil;
    _holeFilledMesh = nil;
}

- (void)clearSLAM
{
    _slamState.initialized = false;
    _slamState.scene = nil;
    _slamState.tracker = nil;
    _slamState.mapper = nil;
    _slamState.keyFrameManager = nil;
}

- (void)setupMapper
{
    _slamState.mapper = nil; // will be garbage collected if we still had one instance.
    
    // Scale the volume resolution with the volume size to maintain good performance.
    const float volumeResolution = _options.initialVolumeResolutionInMeters * (_slamState.volumeSizeInMeters.x / _options.initialVolumeSizeInMeters.x);
    //   0.05 * (10.0 / 10.0)

    GLKVector3 volumeBounds;
    volumeBounds.x = roundf(_slamState.volumeSizeInMeters.x / volumeResolution);
    volumeBounds.y = roundf(_slamState.volumeSizeInMeters.y / volumeResolution);
    volumeBounds.z = roundf(_slamState.volumeSizeInMeters.z / volumeResolution);
    
    NSLog(@"volumeBounds.x: %f", volumeBounds.x);
    NSLog(@"volumeResolution: %f", volumeResolution);
    
    NSDictionary* mapperOptions = @{
                                    kSTMapperVolumeResolutionKey: @(volumeResolution),
                                    kSTMapperVolumeBoundsKey: @[@(volumeBounds.x), @(volumeBounds.y), @(volumeBounds.z)],
                                    
                                    kSTMapperVolumeHasSupportPlaneKey: @(_slamState.cameraPoseInitializer.hasSupportPlane),
                                    //kSTMapperDepthIntegrationFarThresholdKey:@8.0f,     // 0.5f-0.8f (meter) SDK
                                    
                                    kSTMapperDepthIntegrationFarThresholdKey:@(self.mapperDepthThresholdSlider.value),
                                    
                                    kSTMapperEnableLiveWireFrameKey: self.liveWireframeSwitch.isOn ? @(YES):@(NO), // more speedy? test
                                    //kSTMapperEnableLiveWireFrameKey: @(YES), // We need a live wireframe mesh for our visualization.
                                    };
    /*
     NSDictionary* mapperOptions = @{
     kSTMapperVolumeResolutionKey: @(volumeResolution),
     kSTMapperVolumeBoundsKey: @[@(volumeBounds.x), @(volumeBounds.y), @(volumeBounds.z)],
     kSTMapperVolumeHasSupportPlaneKey: @(_slamState.cameraPoseInitializer.hasSupportPlane),
     kSTMapperDepthIntegrationFarThresholdKey:4.0f.
     
     kSTMapperEnableLiveWireFrameKey: @(YES), // more speedy? test
     //kSTMapperEnableLiveWireFrameKey: @(YES), // We need a live wireframe mesh for our visualization.
     };

     */
    

    // Initialize the mapper.
    _slamState.mapper = [[STMapper alloc] initWithScene:_slamState.scene options:mapperOptions];
}

- (void)processDepthFrame:(STDepthFrame *)depthFrame
               colorFrame:(STColorFrame *)colorFrame
{
    //[_slamState.tracker setOptions:@{kSTTrackerQualityKey:@(STTrackerQualityFast)}];

    /*
    if (_options.applyExpensiveCorrectionToDepth)
    {
        NSAssert (!_options.useHardwareRegisteredDepth, @"Cannot enable both expensive depth correction and registered depth.");
        BOOL couldApplyCorrection = [depthFrame applyExpensiveCorrection];
        if (!couldApplyCorrection)
        {
            NSLog(@"Warning: could not improve depth map accuracy, is your firmware too old?");
        }
    }
    */

    // Upload the new color image for next rendering.
    if (self.drawModeSwitch.isOn) {
        if (colorFrame != nil)
            [self uploadGLColorTexture:colorFrame];
    }
    
    switch (_slamState.roomCaptureState)
    {
        case RoomCaptureStatePoseInitialization:
        {
            // Estimate the new scanning volume position as soon as gravity has an estimate.
            if (GLKVector3Length(_lastCoreMotionGravity) > 1e-5f)
            {
                bool success = [_slamState.cameraPoseInitializer updateCameraPoseWithGravity:_lastCoreMotionGravity depthFrame:nil error:nil];
                NSAssert (success, @"Camera pose initializer error.");
            }
            
            break;
        }
            
        case RoomCaptureStateScanning:
        {
            // 前回の（情報を更新しない、今の時点で最新の）カメラの姿勢を取得保存する　デプスカメラの姿勢としtえ
            GLKMatrix4 depthCameraPoseBeforeTracking = [_slamState.tracker lastFrameCameraPose];
            
            NSError* trackingError = nil;
            NSString* trackingMessage = nil;
            
            // Reset previous tracker or keyframe messages, they are now obsolete.
            NSString* trackerErrorMessage = nil;
            NSString* keyframeErrorMessage = nil;

            // Estimate the new camera pose.　新しいカメラ姿勢の推定
            // ||||||||||||||| トラッカーの更新 ||||||||||||||||　今回のカメラ姿勢を推定して取得・更新を試みる
            BOOL trackingOk;
            //if (scanFrameCount == 0) {
                trackingOk = [_slamState.tracker updateCameraPoseWithDepthFrame:depthFrame colorFrame:colorFrame error:&trackingError]; // important!
                //trackingOk = [_slamState.tracker updateCameraPoseWithMotion:motion error:&trackingError];
            //}
            
            if (!trackingOk) {
                NSLog(@"[Structure] STTracker Error: %@.", [trackingError localizedDescription]);
                trackingMessage = [trackingError localizedDescription];
                // ※次のエラーが主に出る　[Structure] STTracker Error: STTracker needs DeviceMotion input for tracking.
            }
            
            // 今の状況から、今回の新しいカメラ姿勢が取得できたら
            if (trackingOk)
            {
                // ||||||||| Integrate it to update the current mesh estimate. |||||||||||||||||||||
                // マッパーのカメラ姿勢・デプス画像ともに最新の（トラッキング成功後の）のデータに更新
                GLKMatrix4 depthCameraPoseAfterTracking = [_slamState.tracker lastFrameCameraPose];
                [_slamState.mapper integrateDepthFrame:depthFrame cameraPose:depthCameraPoseAfterTracking];
                
                // Make sure the pose is in color camera coordinates in case we are not using registered depth.
                // 確認してください　私達がレジスターデプスを使わない場合、姿勢はカラーカメラの座標内です
                // colorCameraPoseInDepthCoordinateFrame: Get the rigid body transformation (RBT) between the iOS color camera and the depth image viewpoint.
                GLKMatrix4 colorCameraPoseInDepthCoordinateSpace;       // デプス座標空間内のカラーカメラの姿勢　の宣言
                [depthFrame colorCameraPoseInDepthCoordinateFrame:colorCameraPoseInDepthCoordinateSpace.m]; // デプス座標フレーム内のカラーカメラ姿勢
                // 最新の（トラッキング成功後の）カラーカメラ姿勢を、最新のデプスカメラ姿勢とデプス座標空間内のカラーカメラ姿勢から 乗算から計算？
                GLKMatrix4 colorCameraPoseAfterTracking = GLKMatrix4Multiply(depthCameraPoseAfterTracking,
                                                                             colorCameraPoseInDepthCoordinateSpace);
                
                bool showHoldDeviceStill = false;   // デバイスを固定して待っててねメッセージを出すフラグをfalseで初期化
                
                // Check if the viewpoint has moved enough to add a new keyframe
                // 新しいカラーカメラ姿勢で、新しいであろうキーフレーム　視点が新しいキーフレームを追加するほどに十分に動いたかどうかを調べた結果、必要だった場合？
                // 新しいフレームによって新しい姿勢が得られるかどうか調べる関数　真偽値が返る
                if ([_slamState.keyFrameManager wouldBeNewKeyframeWithColorCameraPose:colorCameraPoseAfterTracking])
                {
                    const bool isFirstFrame = (_slamState.prevFrameTimeStamp < 0.); // 今回が初めてのフレームかどうかを表すフラグ　だった場合true
                    bool canAddKeyframe = false;                                    // キーフレームを追加できるかどうかのフラグ
                    
                    if (isFirstFrame)
                    {
                        canAddKeyframe = true;                                      // 一番最初のフレームならばキーフレームは常に追加できる
                    }
                    else    // 一番初めのフレームでない場合　（無条件にキーフレームを追加できない場合
                    {
                        // 前回の姿勢との角度の変化差分　秒間の角度変化
                        float deltaAngularSpeedInDegreesPerSeconds = FLT_MAX;
                        NSTimeInterval deltaSeconds = depthFrame.timestamp - _slamState.prevFrameTimeStamp;
                        
                        // If deltaSeconds is 2x longer than the frame duration of the active video device, do not use it either
                        // トラッキングに成功した時間間隔がビデオデバイスのフレーム間時間の2倍以上の場合、使わない
                        // たぶん撮影者がカメラをおおきく振りすぎたときとか、画像がブレブレになるので、キーフレームとしては使わないということ
                        CMTime frameDuration = self.videoDevice.activeVideoMaxFrameDuration;        // 1フレームの時間の長さ
                        if (deltaSeconds < (float)frameDuration.value/frameDuration.timescale*2.f)
                        {
                            // Compute angular speed　角速度の計算
                            // 秒間の角速度 = 2つの姿勢から回転量を度単位で計算し、差分時間で割って秒間の角速度を求める
                            deltaAngularSpeedInDegreesPerSeconds = deltaRotationAngleBetweenPosesInDegrees (depthCameraPoseBeforeTracking, depthCameraPoseAfterTracking)/deltaSeconds;
                        }
                        
                        // If the camera moved too much since the last frame, we will likely end up
                        // with motion blur and rolling shutter, especially in case of rotation. This
                        // checks aims at not grabbing keyframes in that case.
                        // 前回のフレームからカメラが移動しすぎていた場合、ブレとかこんにゃく現象を避けるため、特に回転について、ここで変なキーフレームを掴まないように処理する
                        if (deltaAngularSpeedInDegreesPerSeconds < _options.maxKeyframeRotationSpeedInDegreesPerSecond)
                        {
                            canAddKeyframe = true;
                        }
                    }
                    
                    // キーフレーム画像を追加できるとわかった場合　最新のカラーカメラ姿勢と画像をセットで渡す
                    if (canAddKeyframe)
                    {
                        [_slamState.keyFrameManager processKeyFrameCandidateWithColorCameraPose:colorCameraPoseAfterTracking
                                                                                     colorFrame:colorFrame
                                                                                     depthFrame:nil];
                    } else { // 早すぎたりしてとにかくキーフレームが追加できなかった場合
                        // Moving too fast. Hint the user to slow down to capture a keyframe
                        // without rolling shutter and motion blur.
                        // 早く動かしすぎた場合、キーフレームを捕まえるために、ヒントをユーザーに伝えてゆっくりにさせる
                        // ブレとかこんにゃく現象とかのないものにするため。
                        if (_slamState.prevFrameTimeStamp > 0.) // only show the message if it's not the first frame.
                        {
                            showHoldDeviceStill = true;         // デバイスを固定して待っててねメッセージを出すフラグをOnに
                        }
                    }
                }
                
                // Compute the translation difference between the initial camera pose and the current one.
                // 初期のカメラの姿勢と今のカメラ姿勢の間の移動量の差を計算する
                GLKMatrix4 initialPose = _slamState.tracker.initialCameraPose;      // 初期カメラ姿勢
                // 初期と現在の移動量（位置の差分
                float deltaTranslation = GLKVector4Distance(GLKMatrix4GetColumn(depthCameraPoseAfterTracking, 3), GLKMatrix4GetColumn(initialPose, 3));
                
                // Show some messages if needed.
                // 必要なら警告メッセージを表示する
                if (showHoldDeviceStill)
                {
                    [self showTrackingMessage:@"Please hold still so we can capture a keyframe..."];
                }
                else if (deltaTranslation > _options.maxDistanceFromInitialPositionInMeters )
                {
                    // Warn the user if he's exploring too far away since this demo is optimized for a rotation around oneself.
                    [self showTrackingMessage:@"Please stay closer to the initial position."];
                }
                else
                {
                    [self hideTrackingErrorMessage];
                }

                // ここにリセットを入れてもいい
                
                // -----------------------------------------------
                // tanaka
                // ------------------------------------------------------------------
                if (((int)self.intervalSlider.value >= 1) && (scanFrameCount % (int)self.intervalSlider.value == 0)) {
                    
                    if (self.saveToFileSwitch.isOn) {
                        
                        STMesh* sceneMesh;
                        if (self.colorScanSwitch.isOn) {
                            
                            [self colorizeMesh];
                            sceneMesh = _colorizedMesh;
                            
                        } else {
                            sceneMesh = [[STMesh alloc] initWithMesh:[_slamState.scene lockAndGetSceneMesh]];
                            [_slamState.scene unlockSceneMesh];     // ロック解除
                        }
                        
                        [scanFrameDateList addObject:[NSDate date]];      // １コマ スキャンし終わった日時を保存しておく
                        
                        if (self.recordToMemorySwitch.isOn) {       // 速度を稼ぐためにメモリ上にいったん記録して、あとでまとめてファイルに保存
                            
                            [recordMeshList addObject:sceneMesh];
                            [scanGpsDataList addObject:recentLocation];
                            
                        } else {
                            
                            // Setup names and paths.
                            NSString *modelDirPath = [NSString stringWithFormat:@"%s/%d/", [saveBaseDirPath UTF8String], nowSaveDirNum];
                            
                            NSString* zipFilename = [NSString stringWithFormat:@"mesh_%d.obj", savedFrameCount]; //@"Model.zip";
                            NSString* zipTemporaryFilePath = [modelDirPath stringByAppendingPathComponent:zipFilename];
                        
                            // We want a ZIP with OBJ, MTL and JPG inside.
                            NSDictionary* fileWriteOptions = @{kSTMeshWriteOptionFileFormatKey: @(STMeshWriteOptionFileFormatObjFile) };            // need set same type file name ext(.zip/.obj)
                            
                            NSDate *scanNowDate = [NSDate date];
                            
                            NSString* scanDateListFilePath = [modelDirPath stringByAppendingPathComponent:scanDateListFileName];
                            NSLog(@"scanDateListFilePath: %@", scanDateListFilePath);
                            // self.debugInfoLabel.text = [NSString stringWithFormat:@"datePath: %@", scanDateListFilePath];
                            
                            NSError* error;
                            BOOL success = [sceneMesh writeToFile:zipTemporaryFilePath options:fileWriteOptions error:&error];
                            if (!success) {
                                return;
                            }
                            
                            dataTypeFileName = @"";
                            NSDateFormatter *df = [[NSDateFormatter alloc] init];
                            [df setLocale:[[NSLocale alloc] initWithLocaleIdentifier:@"ja_JP"]]; // Localeの指定
                            [df setDateFormat:@"yyyy/MM/dd HH:mm:ss.SSS"];
                            
                            
                            NSDate *nsd = [NSDate date];
                            NSString *strNow = [df stringFromDate:nsd];
                            
                            // ミリセカンド(ms)を取得
                            long timeFromScanStart =  (long)([nsd timeIntervalSinceDate:scanStartDate] * 1000);
                            
                            NSDateFormatter* formatter = [[NSDateFormatter alloc] init];
                            
                            // 変換用の書式を設定します。
                            [formatter setDateFormat:@"yyyy/MM/dd hh:mm:ss"];
                            
                            // NSDate を NSString に変換します。
                            NSString *gpsDateConverted = [formatter stringFromDate:recentLocation.timestamp];
                            
                            NSString *lineStr = [NSString
                                                 stringWithFormat:@"%i,%li,%@,%f,%f,%f,%f,%f,%f,%f,%@\n",
                                                 savedFrameCount,
                                                 timeFromScanStart,
                                                 strNow,
                                                 recentLocation.coordinate.latitude,
                                                 recentLocation.coordinate.longitude,
                                                 recentLocation.altitude,
                                                 recentLocation.horizontalAccuracy,
                                                 recentLocation.verticalAccuracy,
                                                 recentLocation.speed,
                                                 recentLocation.course,
                                                 gpsDateConverted
                                                 ];
                            
                            NSFileManager *fManager = [NSFileManager defaultManager];
                            BOOL result = [fManager fileExistsAtPath:scanDateListFilePath];
                            if(!result){
                                result = [self createFile:scanDateListFilePath];
                            }
                            NSFileHandle *fileHandle = [NSFileHandle fileHandleForWritingAtPath:scanDateListFilePath];
                            
                            [fileHandle seekToEndOfFile];

                            NSData *dat = [lineStr dataUsingEncoding:NSUTF8StringEncoding];
                            [fileHandle writeData:dat];
                            //効率化のためにファイルに書き込まれずにキャッシュされる場合があるらしいのでsynchronizeFileで書き込み
                            [fileHandle synchronizeFile];
                            [fileHandle closeFile];
                            
                        }
                        
                        savedFrameCount++;
                    }
                    
                    [self countFps];
                    
                    //[self resetSLAM];
                    _slamState.prevFrameTimeStamp = -1.0;
                    [_slamState.mapper reset];            // 最初に撮った形状にマッピングできればいいのであれば、リセットしなくて良さそう？　しかし、今回は立体的な動きを取るのでリセットは必須…？　有効にすると約20fpsになる=そこそこ重い　このリセットと他2つだけでは時系列再生にならない、trackerが大事？　形状変わらずに絵だけが変わる場合はok
                    //[_slamState.tracker reset];             // これのリセットを有効にすると時系列再生でき始めた  mapperリセットしないと6fps->8fpsくらいには上がる
                    //[_slamState.tracker];
                    [_slamState.scene clear];               // 軽い  あってもほぼ30fps
                    [_slamState.keyFrameManager clear];     // 軽い　 あっても30fps
                    
                    _colorizedMesh = nil;
                    _holeFilledMesh = nil;
                    
                    //[self enterPoseInitializationState];
                    //[self enterScanningState];
                    
                    
                }
                
                scanFrameCount++;
                // -----------------------------------------------
                trackingOkCounter++;
            }
            else
            {
                // Update the tracker message if there is any important feedback.
                trackerErrorMessage = computeTrackerMessage(_slamState.tracker.trackerHints);
                                
                // Integrate the depth frame if the pose accuracy is great.
                if (_slamState.tracker.poseAccuracy >= STTrackerPoseAccuracyHigh)
                {
                    [_slamState.mapper integrateDepthFrame:depthFrame cameraPose:_slamState.tracker.lastFrameCameraPose];
                }
            }
            
            if (trackerErrorMessage)
            {
                [self showTrackingMessage:trackerErrorMessage];
            }
            else if (keyframeErrorMessage)
            {
                [self showTrackingMessage:keyframeErrorMessage];
            }
            else
            {
                [self hideTrackingErrorMessage];
            }
            
            allFrameCounter++;
            
            self.debugInfoLabel.text = [NSString stringWithFormat:@"ok: %d /  all: %d", trackingOkCounter, allFrameCounter];
            
            break;
        }
            
        case RoomCaptureStateViewing:
        default:
        {} // do nothing, the MeshViewController will take care of this.
    }
}


@end
