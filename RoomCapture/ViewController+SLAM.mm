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
    // (cameraPoseInitializer: Determine an initial camera pose to make the best use of the cuboid scanning volume.)
    _slamState.cameraPoseInitializer = [[STCameraPoseInitializer alloc]
                                        initWithVolumeSizeInMeters:_slamState.volumeSizeInMeters
                                        options:@{kSTCameraPoseInitializerStrategyKey:@(STCameraPoseInitializerStrategyGravityAlignedAtVolumeCenter)
                                        }
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
    
    DLog(@"volumeBounds.x: %f", volumeBounds.x);
    DLog(@"volumeResolution: %f", volumeResolution);
    
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
    DLog(@"processDepthFrame: start ----------");

    //[_slamState.tracker setOptions:@{kSTTrackerQualityKey:@(STTrackerQualityFast)}];

    /*
    if (_options.applyExpensiveCorrectionToDepth)
    {
        NSAssert (!_options.useHardwareRegisteredDepth, @"Cannot enable both expensive depth correction and registered depth.");
        BOOL couldApplyCorrection = [depthFrame applyExpensiveCorrection];
        if (!couldApplyCorrection)
        {
            DLog(@"Warning: could not improve depth map accuracy, is your firmware too old?");
        }
    }
    */

    // Upload the new color image for next rendering.
    if (self.drawModeSwitch.isOn) {
        if (colorFrame != nil) {
            DLog(@"uploadGLColorTexture: start ----------");
            [self uploadGLColorTexture:colorFrame];
        }
    }
    
    switch (_slamState.roomCaptureState)
    {
        // スキャン前画面の場合 ======================================================================================================
        case RoomCaptureStatePoseInitialization:
        {
            DLog(@"RoomCaptureStatePoseInitialization: start ----------");
            
            // Estimate the new scanning volume position as soon as gravity has an estimate.
            if (GLKVector3Length(_lastCoreMotionGravity) > 1e-5f)
            {
                bool success = [_slamState.cameraPoseInitializer updateCameraPoseWithGravity:_lastCoreMotionGravity depthFrame:nil error:nil];
                NSAssert (success, @"Camera pose initializer error.");
            }
            
            DLog(@"RoomCaptureStatePoseInitialization: end ----------");
            break;
        }
            
        // スキャン中画面の場合 ======================================================================================================
        case RoomCaptureStateScanning:
        {
            DLog(@"RoomCaptureStateScanning: start ----------");
            
            if (firstScanFlag) {
                DLog(@"RoomCaptureStateScanning.isFirstFrame ");
                firstGetDepthFrame = depthFrame;
                firstGetColorFrame = colorFrame;
            }
            
            // 共用変数の宣言と初期化 -----
            NSError* trackingError = nil;
            NSString* trackingMessage = nil;
            NSString* trackerErrorMessage = nil;
            NSString* keyframeErrorMessage = nil;
            GLKMatrix4 colorCameraPoseAfterTracking;
            GLKMatrix4 depthCameraPoseAfterTracking;
            bool isCanRecordThisFrame = false;
            
            // 固定スキャンの場合 ===========================================================
            if (self.fixedTrackingSwitch.isOn) {
                
                // インターバルで割って0になる回でない場合、マッパーだけ更新させてフレーム終了 ==========================================
                DLog(@"self.fixedTrackingSwitch.isOn && self.intervalSlider.value != 0 is true.");
                if (((int)self.intervalSlider.value >= 1) && (scanFrameCount % (int)self.intervalSlider.value != 0)){
                    scanFrameCount++;
                    allFrameCounter++;
                    if (_slamState.tracker.poseAccuracy >= STTrackerPoseAccuracyHigh) {
                        [_slamState.mapper integrateDepthFrame:depthFrame cameraPose:firstCameraPoseOnScan];
                    }
                
                    _slamState.prevFrameTimeStamp = depthFrame.timestamp;
                    firstScanFlag = false;
                    break;
                }
                
                // リセット --------------------------------------
                DLog(@"processDepthFrame.reset() fixed");
                [_slamState.mapper reset];
                [_slamState.scene clear];
                [_slamState.keyFrameManager clear];
                _colorizedMesh = nil;
                _holeFilledMesh = nil;

                // 今回のトラッキング前の（情報を更新しない、今の時点で最新の）カメラの姿勢を取得保存する　デプスカメラの姿勢として
                GLKMatrix4 depthCameraPoseBeforeTracking = [_slamState.tracker lastFrameCameraPose];
                
                // 新しいカメラ姿勢の推定　今回のカメラ姿勢を推定して取得・更新を試みる
                // ||||||||||||||| トラッカーの更新 |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
                DLog(@"tracking process");
                BOOL trackingOk;
                if (firstScanFlag) {
                    trackingOk = [_slamState.tracker updateCameraPoseWithDepthFrame:depthFrame colorFrame:colorFrame error:&trackingError]; // important!
                    firstCameraPoseOnScan = [_slamState.tracker lastFrameCameraPose];
                } else {
                    trackingOk = true;
                }

                if (trackingOk)
                {
                    DLog(@"if trackingOk is true");
                    
                    // ||||||||| Integrate it to update the current mesh estimate. |||||||||||||||||||||||||||||||||||||||||||||||
                    // マッパーのカメラ姿勢・デプス画像ともに最新の（トラッキング成功後の）のデータに更新
                    depthCameraPoseAfterTracking = firstCameraPoseOnScan;
                    
                    [_slamState.mapper integrateDepthFrame:depthFrame cameraPose:depthCameraPoseAfterTracking];
                    [_slamState.mapper finalizeTriangleMesh];           //not default: wait for integrate background process end
                    
                    // 確認してください　私達がレジスターデプスを使わない場合、姿勢はカラーカメラの座標内です
                    // colorCameraPoseInDepthCoordinateFrame: Get the rigid body transformation (RBT) between the iOS color camera and the depth image viewpoint.
                    GLKMatrix4 colorCameraPoseInDepthCoordinateSpace;       // デプス座標空間内のカラーカメラの姿勢　の宣言
                    [depthFrame colorCameraPoseInDepthCoordinateFrame:colorCameraPoseInDepthCoordinateSpace.m]; // デプス座標フレーム内のカラーカメラ姿勢
                    // 最新の（トラッキング成功後の）カラーカメラ姿勢を、最新のデプスカメラ姿勢とデプス座標空間内のカラーカメラ姿勢から 乗算から計算？
                    colorCameraPoseAfterTracking = GLKMatrix4Multiply(depthCameraPoseAfterTracking,
                                                                                 colorCameraPoseInDepthCoordinateSpace);
                    
                    bool showHoldDeviceStill = false;   // デバイスを固定して待っててねメッセージを出すフラグをfalseで初期化
                    
                    DLog(@"add keyframe process ----");
                    [_slamState.keyFrameManager processKeyFrameCandidateWithColorCameraPose:colorCameraPoseAfterTracking
                                                                                     colorFrame:colorFrame
                                                                                     depthFrame:nil];
                } else {
                    DLog(@"[Structure] STTracker Error: %@.", [trackingError localizedDescription]);
                    trackingMessage = [trackingError localizedDescription];
                    // ※次のエラーが主に出る　[Structure] STTracker Error: STTracker needs DeviceMotion input for tracking.
                }

            // 移動しながらのスキャンの場合 ===================================================================
            } else {

                // リセット ---------------------------
                DLog(@"processDepthFrame.reset() no_fixed");
                [_slamState.mapper reset];
                //[_slamState.tracker reset];
                [_slamState.scene clear];
                [_slamState.keyFrameManager clear];
                _colorizedMesh = nil;
                _holeFilledMesh = nil;

                // 今回のトラッキング前の（情報を更新しない、今の時点で最新の）カメラの姿勢を取得保存する　デプスカメラの姿勢として
                GLKMatrix4 depthCameraPoseBeforeTracking = [_slamState.tracker lastFrameCameraPose];
                
                // 新しいカメラ姿勢の推定　今回のカメラ姿勢を推定して取得・更新を試みる
                // ||||||||||||||| トラッカーの更新 |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
                DLog(@"tracking process");
                BOOL trackingOk;
                trackingOk = [_slamState.tracker updateCameraPoseWithDepthFrame:depthFrame colorFrame:colorFrame error:&trackingError];
                
                // 今の状況から、今回の新しいカメラ姿勢が取得できたら
                if (trackingOk)
                {
                    DLog(@"if trackingOk is true");
                    
                    _slamState.prevFrameTimeStamp = -1;
                    const bool isFirstFrame = (_slamState.prevFrameTimeStamp < 0.);
                    
                    if (firstScanFlag) {
                        firstCameraPoseOnScan = [_slamState.tracker lastFrameCameraPose];
                    }
                    
                    // マッパーのカメラ姿勢・デプス画像ともに最新の（トラッキング成功後の）のデータに更新
                    depthCameraPoseAfterTracking = [_slamState.tracker lastFrameCameraPose];
                    
                    [_slamState.mapper integrateDepthFrame:depthFrame cameraPose:depthCameraPoseAfterTracking];
                    [_slamState.mapper finalizeTriangleMesh];           //not default: wait for integrate background process end
                    
                    // 確認してください　私達がレジスターデプスを使わない場合、姿勢はカラーカメラの座標内です
                    // colorCameraPoseInDepthCoordinateFrame: Get the rigid body transformation (RBT) between the iOS color camera and the depth image viewpoint.
                    GLKMatrix4 colorCameraPoseInDepthCoordinateSpace;       // デプス座標空間内のカラーカメラの姿勢　の宣言
                    [depthFrame colorCameraPoseInDepthCoordinateFrame:colorCameraPoseInDepthCoordinateSpace.m]; // デプス座標フレーム内のカラーカメラ姿勢
                    // 最新の（トラッキング成功後の）カラーカメラ姿勢を、最新のデプスカメラ姿勢とデプス座標空間内のカラーカメラ姿勢から 乗算から計算？
                    colorCameraPoseAfterTracking = GLKMatrix4Multiply(depthCameraPoseAfterTracking,
                                                                                 colorCameraPoseInDepthCoordinateSpace);
                    
                    bool showHoldDeviceStill = false;   // デバイスを固定して待っててねメッセージを出すフラグをfalseで初期化
                    
                    // ||||||| キーフレームの追加 |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
                    DLog(@"add keyframe process ----");
                    
                    // 新しいカラーカメラ姿勢で、新しいであろうキーフレーム　視点が新しいキーフレームを追加するほどに十分に動いたかどうかを調べた結果、必要だった場合？
                    // 新しいフレームによって新しい姿勢が得られるかどうか調べる関数　真偽値が返る
                    if ([_slamState.keyFrameManager wouldBeNewKeyframeWithColorCameraPose:colorCameraPoseAfterTracking])
                    {
                        bool canAddKeyframe = false;                                    // キーフレームを追加できるかどうかのフラグ
                        
                        if (isFirstFrame)
                        {
                            canAddKeyframe = true;                                      // 一番最初のフレームならばキーフレームは常に追加できる
                        }
                        else    // 初回color/depthフレームでない場合　（無条件にキーフレームを追加できない場合
                        {
                            // 前回の姿勢との角度の変化差分　秒間の角度変化
                            float deltaAngularSpeedInDegreesPerSeconds = FLT_MAX;
                            NSTimeInterval deltaSeconds = depthFrame.timestamp - _slamState.prevFrameTimeStamp;
                            
                            // トラッキングに成功した時間間隔がビデオデバイスのフレーム間時間の2倍以上の場合、使わない
                            // たぶん撮影者がカメラをおおきく振りすぎたときとか、画像がブレブレになるので、キーフレームとしては使わないということ
                            CMTime frameDuration = self.videoDevice.activeVideoMaxFrameDuration;        // 1フレームの時間の長さ
                            if (deltaSeconds < (float)frameDuration.value/frameDuration.timescale*2.f)
                            {
                                // 角速度の計算  ...  秒間の角速度 = 2つの姿勢から回転量を度単位で計算し、差分時間で割って秒間の角速度を求める
                                deltaAngularSpeedInDegreesPerSeconds = deltaRotationAngleBetweenPosesInDegrees (depthCameraPoseBeforeTracking, depthCameraPoseAfterTracking)/deltaSeconds;
                            }
                            
                            // 前回のフレームからカメラが移動しすぎていた場合、ブレとかこんにゃく現象を避けるため、特に回転について、ここで変なキーフレームを掴まないように処理する
                            if (deltaAngularSpeedInDegreesPerSeconds < _options.maxKeyframeRotationSpeedInDegreesPerSecond)
                            {
                                canAddKeyframe = true;
                            }
                        }
                        
                        // キーフレーム画像を追加できるとわかった場合　最新のカラーカメラ姿勢と画像をセットで渡す ========================
                        if (canAddKeyframe)
                        {
                            // ||||||| キーフレームの追加 ||||||||||||||||||||||||||||||||||||||||||||||||||
                            [_slamState.keyFrameManager processKeyFrameCandidateWithColorCameraPose:colorCameraPoseAfterTracking
                                                                                         colorFrame:colorFrame
                                                                                         depthFrame:nil];
                        } else { // 早すぎたりしてとにかくキーフレームが追加できなかった場合

                            // 早く動かしすぎた場合、キーフレームを捕まえるために、ヒントをユーザーに伝えてゆっくりにさせる
                            // ブレとかこんにゃく現象とかのないものにするため。
                            if (_slamState.prevFrameTimeStamp > 0.) // only show the message if it's not the first frame.
                            {
                                showHoldDeviceStill = true;         // デバイスを固定して待っててねメッセージを出すフラグをOnに
                            }
                        }
                        // ================================================================================================
                    }
                    
                    isCanRecordThisFrame = true;
                    
                // トラッキングに失敗していた場合 ---------------
                } else {
                
                    DLog(@"[Structure] STTracker Error: %@.", [trackingError localizedDescription]);
                    trackingMessage = [trackingError localizedDescription];
                    // ※次のエラーが主に出る　[Structure] STTracker Error: STTracker needs DeviceMotion input for tracking.

                    // ユーザにメッセージで指示して改善を図る
                    trackerErrorMessage = computeTrackerMessage(_slamState.tracker.trackerHints);
                    
                    // マッパーの更新 ------ 今回のトラッキングに失敗しても、姿勢の精度が良ければ、今までに撮れた中では一番新しいカメラ姿勢と最新のデプス情報からマッパーを更新する ||||||||||||||||||||
                    if (_slamState.tracker.poseAccuracy >= STTrackerPoseAccuracyHigh)
                    {
                        [_slamState.mapper integrateDepthFrame:depthFrame cameraPose:_slamState.tracker.lastFrameCameraPose];
                    }
                }
                
            } // fixed - noFixed
            
            // 今回のフレームが記録できる場合
            if (isCanRecordThisFrame) {
                
                if (scanFrameCount % (int)self.intervalSlider.value == 0) {
                    
                    // 撮影結果をファイルに保存するスイッチが入っている場合 =====================================================================
                    if (self.saveToFileSwitch.isOn)
                    {
                        DLog(@"save to file(or mem) process  ----");
                        
                        [scanFrameDateList addObject:[NSDate date]];      // １コマ スキャンし終わった日時を保存しておく

                        // メッシュデータの保存
                        STMesh* sceneMesh;
                        if (self.colorScanSwitch.isOn) {    // カラーで保存する場合 ==============================
                            
                            [self colorizeMesh];
                            sceneMesh = _colorizedMesh;
                            
                        } else {
                            
                            // モノクロスキャン
                            sceneMesh = [[STMesh alloc] initWithMesh:[_slamState.scene lockAndGetSceneMesh]];
                            [_slamState.scene unlockSceneMesh];     // ロック解除
                            
                        }
                        
                        // メモリへの視点移動情報の保存 ---------------------------------------------------------
                        NSMutableArray *matrixArray = [[NSMutableArray alloc]init];
                        for(int c=0; c<16; c++) {
                            [matrixArray addObject:[NSNumber numberWithFloat:colorCameraPoseAfterTracking.m[c]] ];
                        }
                        [colorCameraPoseList addObject:matrixArray];
                        
                        NSMutableArray *matrixArray2 = [[NSMutableArray alloc]init];
                        for(int c=0; c<16; c++) {
                            [matrixArray2 addObject:[NSNumber numberWithFloat:depthCameraPoseAfterTracking.m[c]] ];
                        }
                        [depthCameraPoseList addObject:matrixArray2];
                        
                        // メモリにいったん保存する（fpsは早い） ------------------------------------------------
                        if (self.recordToMemorySwitch.isOn) {       // 速度を稼ぐためにメモリ上にいったん記録して、あとでまとめてファイルに保存
                            
                            [recordMeshList addObject:sceneMesh];
                            [scanGpsDataList addObject:recentLocation];
                            
                        } else {    // ファイルにリアルタイムに直接保存していく （fpsは遅くなる）
                            // 一時的に無効化
                        }
                        
                        savedFrameCount++;
                    }
                    
                    [self countFps];        // インターバルのタイミングでFPSをカウント
                    
                    //DLog(@"resetSLAM started");
                    //[self resetSLAM];
                    
                }
                
                scanFrameCount++;
                trackingOkCounter++;
            }
            
            allFrameCounter++;
            
            self.debugInfoLabel.text = [NSString stringWithFormat:@"ok: %d /  all: %d", trackingOkCounter, allFrameCounter];
            
            _slamState.prevFrameTimeStamp = depthFrame.timestamp;
            firstScanFlag = false;

            DLog(@"RoomCaptureStateScanning: end --------------------");
            break;
        }
            
        // 閲覧画面の場合 ======================================================================================================
        case RoomCaptureStateViewing:
        default:
            {} // do nothing, the MeshViewController will take care of this.
            
    }
    
    DLog(@"processDepthFrame: end ----------");

}


@end
