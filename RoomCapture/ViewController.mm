/*
 This file is part of the Structure SDK.
 Copyright © 2016 Occipital, Inc. All rights reserved.
 http://structure.io
 */

#import "ViewController.h"
#import "ViewController+Camera.h"
#import "ViewController+OpenGL.h"
#import "ViewController+Sensor.h"
#import "ViewController+SLAM.h"

#import <Structure/Structure.h>
#import <Structure/StructureSLAM.h>

#import "CustomUIKitStyles.h"

#include <cmath>

@implementation ViewController

#pragma mark - ViewController Setup

- (void)dealloc
{
    [self.avCaptureSession stopRunning];
    
    if ([EAGLContext currentContext] == _display.context)
    {
        [EAGLContext setCurrentContext:nil];
    }
}

// 画面のインスタンスが初期化される時、一回だけ
// アプリを起動して、画面を読み込み終わった時
// important
- (void)viewDidLoad
{
    [self loadUserDefaultSettingData];  // add tanaka

    [super viewDidLoad];
    
    // for mem rec - tanaka add ------------------------------------------------
    scanFrameTime = 1;      // add by tanaka
    scanFrameCount = 0;      // add by tanaka
    recordMeshNum = 0;      // add by tanaka
    recordMeshList = [NSMutableArray array];
    scanFrameDateList = [NSMutableArray array];     // add 2016.6
    
    basePath = @"";
    fileManager = [[NSFileManager alloc] init];
    //filePath = [[NSBundle mainBundle] bundlePath];
    filePath = @"Documents/artdkt_structure3d";
    NSLog(@"filePathScan: %s", [filePath UTF8String] );
    for (NSString *content in [fileManager contentsOfDirectoryAtPath:filePath error:nil ]) {
        const char *chars = [content UTF8String];
        NSLog(@"filePathList: %s", chars);
    }
    
    [self setupUserInterface];
    
    // UI ---------------------------------------------------------------------------

    [self setupUserInterface];
    
    self.recordToMemorySwitch.on = udRecordToMemorySwitch ? YES:NO;
    self.nearModeSwitch.on = udNearModeSwitch ? YES:NO;
    self.recordGpsSwitch.on = udRecordGpsSwitch ? YES:NO;
    self.saveToFileSwitch.on = udSaveToFileSwitch ? YES:NO;
    self.colorScanSwitch.on = udColorScanSwitch ? YES:NO;
    self.drawModeSwitch.on = udDrawModeSwitch ? YES:NO;
    self.trackingSmallObjectSwitch.on = udTrackingSmallObjectSwitch ? YES:NO;
    self.trackerQualityAccurateSwitch.on = udTrackerQualityAccurate ? YES:NO;
    self.fixedTrackingSwitch.on = udFixedTrackingSwitch ? YES:NO;
    self.roomSizeSlider.value = udRoomSizeSlider;
    self.resolutionSlider.value = udResolutionSlider;
    self.intervalSlider.value = udIntervalSlider;
    
    [self actionOnResolutionSliderValueChanged];
    [self actionOnRoomSizeSliderValueChanged];
    
    self.resolutionSliderLabel.text = [NSString stringWithFormat:@"%.3f", self.resolutionSlider.value ];
    self.intervalSliderLabel.text = [NSString stringWithFormat:@"%d", (int)self.intervalSlider.value ];
    self.roomSizeSliderLabel.text = [NSString stringWithFormat:@"%.3f", self.roomSizeSlider.value ];

    
    // -----------------------------------------------------------------------

    [self setupGL];
    
    
    [self setupMeshViewController];
    
    [self setupIMU];
    
    [self setupSLAM];
    
    [self setupStructureSensor];
    
    // Make sure we get notified when the app becomes active to start/restore the sensor state if necessary.
    [[NSNotificationCenter defaultCenter] addObserver:self
                                             selector:@selector(appDidBecomeActive)
                                                 name:UIApplicationDidBecomeActiveNotification
                                               object:nil];
    
    // add by tananka ------------------------------------------------------------
    
    scanFrameDateList = [NSMutableArray array];     // add 2016.6
    
    scanFrameCount = 0;
    savedFrameCount = 0;
    uiHideFlag = false;
    
    basePath = @"";
    fileManager = [[NSFileManager alloc] init];
    //filePath = [[NSBundle mainBundle] bundlePath];
    filePath = @"Documents/artdkt_structure3d";
    saveBaseDirName = @"artdkt_structure3d";
    saveBaseDirPath = @"";
    scanDateListFileName = @"scanTimeRecord.csv";
    dataTypeFileName = @"";
    
    NSLog(@"filePathScan: %s", [filePath UTF8String] );
    for (NSString *content in [fileManager contentsOfDirectoryAtPath:filePath error:nil ]) {
        const char *chars = [content UTF8String];
        NSLog(@"filePathList: %s", chars);
    }

    // GPS ---------------------------------------------------
    lm = [[CLLocationManager alloc] init];
    lm.delegate = self;
    
    // 取得精度の指定
    //lm.desiredAccuracy = kCLLocationAccuracyNearestTenMeters;
    //lm.desiredAccuracy = kCLLocationAccuracyBest;
    lm.desiredAccuracy = kCLLocationAccuracyBestForNavigation;
    // 取得頻度（指定したメートル移動したら再取得する）
    //lm.distanceFilter = 5;    // 5m移動するごとに取得
    lm.distanceFilter = kCLDistanceFilterNone; // 変更があると全て通知
    // lm.activityType = CLActivityTypeFitness;     設定する必要はないがすると無駄なアクセスを減らして節電になる
    if ([CLLocationManager authorizationStatus] == kCLAuthorizationStatusNotDetermined) {       // 位置情報使用許可をもらっていない時は初回だけ確認
        [lm requestAlwaysAuthorization];
    }
    // GPS end -----------------------------------------------
    
    fpsCount = 0;
    
    [self actionOnRoomSizeSliderValueChanged];

    
}

- (void)updateClockLabel
{
    NSDate *date = [NSDate date];
    NSDateFormatter *dateFormatter = [[NSDateFormatter alloc] init];
    
    dateFormatter.dateFormat = @"yyyy/MM/dd HH:mm:ss.SSS";
    NSString *dateStr = [dateFormatter stringFromDate:date];
    
    // 2014/02/18 11:08:12
    _clockLabel.text = [NSString stringWithFormat:@"%@", dateStr ];
  
}

- (void)viewDidAppear:(BOOL)animated
{
    [super viewDidAppear:animated];
    
    // The framebuffer will only be really ready with its final size after the view appears.
    [(EAGLView *)self.view setFramebuffer];
    
    [self setupGLViewport];
    
    // We will connect to the sensor when we receive appDidBecomeActive.
    
    clockTimer = [NSTimer scheduledTimerWithTimeInterval:1.0f target:self selector:@selector(updateClockLabel) userInfo:nil repeats:YES];

}

- (void)viewWillDisappear:(BOOL)animated
{
    [super viewWillDisappear:animated];
    
    if ( clockTimer ) {
        [clockTimer invalidate];
        clockTimer = nil;
    }
}

- (void)appDidBecomeActive
{
    // Try to connect to the Structure Sensor and stream if necessary.
    if ([self currentStateNeedsSensor])
        [self connectToStructureSensorAndStartStreaming];
    
    // Abort the current scan if we were still scanning before going into background since we
    // are not likely to recover well.
    if (_slamState.roomCaptureState == RoomCaptureStateScanning)
    {
        [self resetButtonPressed:self];
    }
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
}

- (void)setupUserInterface
{
    // Make sure the status bar is hidden.
    [[UIApplication sharedApplication] setStatusBarHidden:YES withAnimation:UIStatusBarAnimationSlide];
    
    // Fully transparent message label, initially.
    self.appStatusMessageLabel.alpha = 0;
    
    // Make sure the label is on top of everything else.
    self.appStatusMessageLabel.layer.zPosition = 100;
    
    // Apply our custom style to the tracking status label.
    [self.trackingMessageLabel applyCustomStyleWithBackgroundColor:blackLabelColorWithLightAlpha];
    
    // Apply our custom style to the roomSize label.
    [self.roomSizeLabel applyCustomStyleWithBackgroundColor:blackLabelColorWithLightAlpha];
    
    // Setup the roomSize slider range. It represents a scale factor, applied to the initial size.
    //self.roomSizeSlider.value = 1.0;
    //self.roomSizeSlider.minimumValue = 1.0/3.0;
    //self.roomSizeSlider.maximumValue = 5.0/3.0;
    // tanaka changed
    self.roomSizeSlider.value = 0.999;
    self.roomSizeSlider.minimumValue = 0.0500;
    self.roomSizeSlider.maximumValue = 0.999;

    self.roomSizeLabel.hidden = true;
    
    _calibrationOverlay = nil;
    
}

// Make sure the status bar is disabled (iOS 7+)
- (BOOL)prefersStatusBarHidden
{
    return YES;
}

- (void)setupMeshViewController
{
    // The mesh viewer will be used after scanning.
    _meshViewController = [[MeshViewController alloc] initWithNibName:@"MeshView" bundle:nil];
    _meshViewController.delegate = self;
    _meshViewNavigationController = [[UINavigationController alloc] initWithRootViewController:_meshViewController];
}



- (void)enterPoseInitializationState
{
    // Switch to the Scan button.
    if (!uiHideFlag) {
        self.scanButton.hidden = NO;
        self.doneButton.hidden = YES;
        self.resetButton.hidden = YES;
    
        // Show the room size controls.
        self.roomSizeSlider.hidden = NO;
    
        // Cannot be lost in cube placement mode.
        _trackingMessageLabel.hidden = YES;
    }
    
    // We leave exposure unlock during init.
    [self setColorCameraParametersForInit];
    
    _slamState.roomCaptureState = RoomCaptureStatePoseInitialization;
    
    [self updateIdleTimer];
}

- (void)enterScanningState
{
    
    NSLog(@"enterScanningState start");
    
    scanStartDate = [NSDate date];
    fpsBasetime = [NSDate date];
    
    if (!uiHideFlag) {
        // Switch to the Done button.
        self.scanButton.hidden = YES;
        self.doneButton.hidden = NO;
        self.resetButton.hidden = NO;
        
        // Hide the room size controls.
        self.roomSizeLabel.hidden = YES;
        self.roomSizeSlider.hidden = YES;
    }
    
    // Create a mapper.
    [self setupMapper];
    
    // Set the initial tracker camera pose.
    _slamState.tracker.initialCameraPose = _slamState.cameraPoseInitializer.cameraPose;
    
    // We will lock exposure during scanning to ensure better coloring.
    [self setColorCameraParametersForScanning];
    
    _slamState.roomCaptureState = RoomCaptureStateScanning;
}

- (void)enterFinalizingState
{
    NSLog(@"enterFinalizingState() start");

    // Cannot be lost if not scanning anymore.
    [self hideTrackingErrorMessage];
    
    if (!uiHideFlag) {
        // Hide the Scan/Done/Reset button.
        self.scanButton.hidden = YES;
        self.doneButton.hidden = YES;
        self.resetButton.hidden = YES;
    }
    
    // Stop the sensors, we don't need them.
    [_sensorController stopStreaming];
    [self stopColorCamera];
    
    // Tell the mapper to compute a final triangle mesh from its data. Will also stop background processing.
    [_slamState.mapper finalizeTriangleMesh];
    
    _slamState.roomCaptureState = RoomCaptureStateFinalizing;
    
    // Colorize the mesh in a background queue.
    [self colorizeMeshInBackground];
}

- (void)colorizeMeshInBackground
{
    // Take a copy of the scene mesh to safely modify it.
    _colorizedMesh = [[STMesh alloc] initWithMesh:[_slamState.scene lockAndGetSceneMesh]];
    [_slamState.scene unlockSceneMesh];
    
    _appStatus.backgroundProcessingStatus = AppStatus::BackgroundProcessingStatusFinalizing;
    [self updateAppStatusMessage];
    
    STBackgroundTask* colorizeTask = [STColorizer
                                      newColorizeTaskWithMesh:_colorizedMesh
                                      scene:_slamState.scene
                                      keyframes:[_slamState.keyFrameManager getKeyFrames]
                                      completionHandler: ^(NSError *error)
                                      {
                                          if (error != nil) {
                                              NSLog(@"Error during colorizing: %@", [error localizedDescription]);
                                          }
                                          
                                          dispatch_async(dispatch_get_main_queue(), ^{
                                              
                                              _appStatus.backgroundProcessingStatus = AppStatus::BackgroundProcessingStatusIdle;
                                              _appStatus.statusMessageDisabled = true;
                                              [self updateAppStatusMessage];
                                              
                                              [self enterViewingState];
                                          });
                                      }
                                      options:@{kSTColorizerTypeKey: @(STColorizerTextureMapForRoom) }
                                      error:nil];
    
    [colorizeTask start];
}


// tanaka add ---------------------------------------------------------------------
- (void)colorizeMesh
{
    // Take a copy of the scene mesh to safely modify it.
    //_colorizedMesh = [_slamState.scene lockAndGetSceneMesh];//[[STMesh alloc] initWithMesh:[_slamState.scene lockAndGetSceneMesh]];
    _colorizedMesh = [[STMesh alloc] initWithMesh:[_slamState.scene lockAndGetSceneMesh]];  // very stable!

    //_appStatus.backgroundProcessingStatus = AppStatus::BackgroundProcessingStatusFinalizing;
    //[self updateAppStatusMessage];
    
    STBackgroundTask* colorizeTask = [STColorizer
                                      newColorizeTaskWithMesh:_colorizedMesh
                                      scene:_slamState.scene
                                      keyframes:[_slamState.keyFrameManager getKeyFrames]
                                      completionHandler: ^(NSError *error)
                                      {
                                          if (error != nil) {
                                              NSLog(@"Error during colorizing: %@", [error localizedDescription]);
                                          }
                                          
                                          /*
                                          dispatch_async(dispatch_get_main_queue(), ^{
                                              
                                              _appStatus.backgroundProcessingStatus = AppStatus::BackgroundProcessingStatusIdle;
                                              _appStatus.statusMessageDisabled = true;
                                              [self updateAppStatusMessage];
                                              
                                              //[self enterViewingState]; comment out by tanaka
                                          });
                                          */
                                      }
                                      options:@{kSTColorizerTypeKey: @(STColorizerTextureMapForRoom) }
                                      error:nil];
    
    [colorizeTask start];
    
    [colorizeTask waitUntilCompletion]; // add by tanaka カラー化が終わるまで処理まち
    [_slamState.scene unlockSceneMesh];
}
//------------------------------------------------------

- (void)enterViewingState
{
    // Place the camera in the center of the scanning volume.
    GLKVector3 cameraCenter = GLKVector3MultiplyScalar(_slamState.cameraPoseInitializer.volumeSizeInMeters, 0.5);
    GLKMatrix4 initialCameraPose = GLKMatrix4MakeTranslation(cameraCenter.x, cameraCenter.y, cameraCenter.z);

    STMesh *mesh = [_slamState.scene lockAndGetSceneMesh];   //original
    [_slamState.scene unlockSceneMesh];
    NSLog(@"enterViewingState set mesh end ?");          // この時点でsetMesh->uploadMeshが実行される？

    [self presentMeshViewer:_colorizedMesh withCameraPose:initialCameraPose];
    
    _slamState.roomCaptureState = RoomCaptureStateViewing;

    [self updateIdleTimer];
    
    /* for mem rec
     // 最初の２コマがなぜかゴミデータなので取り除いてから次に移動させる
     //[recordMeshList removeObjectsInRange:NSMakeRange(0, 2)];
    
    [_meshViewController setRecordMeshList:recordMeshList]; //add by tanaka
    [_meshViewController setScanFrameDateList:scanFrameDateList]; //add by tanaka add 2016.6
    [_meshViewController setRecordMeshNum:(int)[recordMeshList count]];//add by tanaka
    [_meshViewController setScanStartDate:scanStartDate ];//add by tanaka
    
     */

    
}

- (void)adjustVolumeSize:(GLKVector3)volumeSize
{
    _slamState.volumeSizeInMeters = volumeSize;
    _slamState.cameraPoseInitializer.volumeSizeInMeters = volumeSize;
}


-(BOOL)currentStateNeedsSensor
{
    switch (_slamState.roomCaptureState)
    {
        // Initialization and scanning need the sensor.
        case RoomCaptureStatePoseInitialization:
        case RoomCaptureStateScanning:
            return TRUE;
            
        // Other states don't need the sensor.
        default:
            return FALSE;
    }
}

- (void) loadUserDefaultSettingData {
    
    NSUserDefaults *defaults = [NSUserDefaults standardUserDefaults];
    udRecordToMemorySwitch = [defaults boolForKey:@"recordToMemorySwitch"];
    udNearModeSwitch = [defaults boolForKey:@"nearModeSwitch"];
    udRecordGpsSwitch = [defaults boolForKey:@"recordGpsSwitch"];
    udSaveToFileSwitch = [defaults boolForKey:@"saveToFileSwitch"];
    udColorScanSwitch = [defaults boolForKey:@"colorScanSwitch"];
    udDrawModeSwitch = [defaults boolForKey:@"drawModeSwitch"];
    udTrackingSmallObjectSwitch = [defaults boolForKey:@"trackingSmallObjectSwitch"];
    udTrackerQualityAccurate = [defaults boolForKey:@"trackerQualityAccurateSwitch"];
    udFixedTrackingSwitch = [defaults boolForKey:@"fixedTrackingSwitch"];
    
    udIntervalSlider = (int)[defaults integerForKey:@"intervalSlider"];
    udResolutionSlider = [defaults floatForKey:@"resolutionSlider"];
    udRoomSizeSlider = [defaults floatForKey:@"roomSizeSlider"];

    NSLog(@"udNearMode: %d", udRecordToMemorySwitch?1:0);
    NSLog(@"udRecordToMemory: %d", udNearModeSwitch?1:0);
    
}

- (void) countFps {
    fpsCount++;
    NSDate *tNowDate = [NSDate date];
    
    NSTimeInterval deltaTime = [tNowDate timeIntervalSinceDate:fpsBasetime];
    NSLog(@"countFps fpsBasetime:%@", fpsBasetime);
    NSLog(@"countFps tNowDate:%@", tNowDate);
    NSLog(@"countFps deltaTime:%.3f", deltaTime);
    if (deltaTime > 1.0) {
        NSLog(@"countFps Update");
        fpsFramerate = (float)fpsCount / (float)deltaTime;
        fpsBasetime = tNowDate;
        fpsCount = 0;
        _scanFpsLabel.text = [NSString stringWithFormat:@"%.2f fps", fpsFramerate];
    }
    
}

- (void)saveDataMemoryToFile {
    
    // Setup names and paths.
    NSString *modelDirPath = [NSString stringWithFormat:@"%s/%d/", [saveBaseDirPath UTF8String], nowSaveDirNum];
    
    for(int i=0; i<savedFrameCount; i++) {

        STMesh *sceneMesh = recordMeshList[i];
        STScene* localScene = sceneList[i];

        NSString* zipFilename = [NSString stringWithFormat:@"mesh_%d.obj", i]; //@"Model.zip";
        NSString* zipTemporaryFilePath = [modelDirPath stringByAppendingPathComponent:zipFilename];
        
        // We want a ZIP with OBJ, MTL and JPG inside.
        NSDictionary* fileWriteOptions = @{kSTMeshWriteOptionFileFormatKey: @(STMeshWriteOptionFileFormatObjFile) };            // need set same type file name ext(.zip/.obj)
        
        NSDate *scanNowDate = scanFrameDateList[i];
        
        NSString* scanDateListFilePath = [modelDirPath stringByAppendingPathComponent:scanDateListFileName];
        NSLog(@"scanDateListFilePath: %@", scanDateListFilePath);
        // self.debugInfoLabel.text = [NSString stringWithFormat:@"datePath: %@", scanDateListFilePath];
        
        /*
        // |||||||||||||||||||| 後工程 色付け ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
        if (_colorScanSwitch.isOn) {
            NSLog(@"afterColorizing start");
            _colorizedMesh = sceneMesh;
            
            NSLog(@"make keyFrame start");
            STKeyFrame* keyFrame = [[STKeyFrame alloc] initWithColorCameraPose:firstCameraPoseOnScan colorFrame:colorFrameList[i] depthFrame:nil];
            //NSArray *tKeyFrames = [NSArray arrayWithObjects:keyFrame, nil];
            NSArray *tKeyFrames = @[keyFrame];
            //NSArray *newArr = @[@"value1", @"value2", @"value3"];
            
            NSLog(@"keyFramesList: %@", keyFramesList);
            NSLog(@"sceneList: %@", sceneList);
            
            NSLog(@"make colorizeTask start");          // 落ちる
            STBackgroundTask* colorizeTask = [STColorizer
                                              newColorizeTaskWithMesh:_colorizedMesh
                                              scene:localScene
                                              //keyframes:keyFramesList[i]
                                              keyframes:tKeyFrames
                                              completionHandler: ^(NSError *error)
                                              {
                                                  if (error != nil) {
                                                      NSLog(@"Error during colorizing: %@", [error localizedDescription]);
                                                  }
                                              }
                                              options:@{kSTColorizerTypeKey: @(STColorizerTextureMapForRoom) }
                                              error:nil];
            
            NSLog(@"colorizeTask start");
            [colorizeTask start];
            
            [colorizeTask waitUntilCompletion]; // add by tanaka カラー化が終わるまで処理まち
            [_slamState.scene unlockSceneMesh];
            
            NSLog(@"colorizeTask end");
            
            sceneMesh = _colorizedMesh;
            NSLog(@"afterColorizing end");
        }
         */
        
        // |||||||||||||||||||| メッシュの保存 ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
        NSError* error;
        BOOL success = [sceneMesh writeToFile:zipTemporaryFilePath options:fileWriteOptions error:&error];
        if (!success) {
            return;
        }

        // ---
        
        
        dataTypeFileName = @"";
        NSDateFormatter *df = [[NSDateFormatter alloc] init];
        [df setLocale:[[NSLocale alloc] initWithLocaleIdentifier:@"ja_JP"]]; // Localeの指定
        [df setDateFormat:@"yyyy/MM/dd HH:mm:ss.SSS"];
        
        CLLocation* tempLocation = scanGpsDataList[i];
        
        NSDate *nsd = scanFrameDateList[i];
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
                             i,
                             timeFromScanStart,
                             strNow,
                             tempLocation.coordinate.latitude,
                             tempLocation.coordinate.longitude,
                             tempLocation.altitude,
                             tempLocation.horizontalAccuracy,
                             tempLocation.verticalAccuracy,
                             tempLocation.speed,
                             tempLocation.course,
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
    
    
    /*
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
     */
    
}


- (BOOL)createFile:(NSString *)localFilePath
{
    return [[NSFileManager defaultManager] createFileAtPath:localFilePath contents:[NSData data] attributes:nil];
}

#pragma mark - IMU

- (void)setupIMU
{
    NSLog(@"setupIMU. started");
    _lastCoreMotionGravity = GLKVector3Make (0,0,0);
    
    // 60 FPS is responsive enough for motion events.
    const float fps = 60.0;
    _motionManager = [[CMMotionManager alloc] init];
    _motionManager.accelerometerUpdateInterval = 1.0/fps;
    _motionManager.gyroUpdateInterval = 1.0/fps;
    
    // Limiting the concurrent ops to 1 is a simple way to force serial execution
    _imuQueue = [[NSOperationQueue alloc] init];
    [_imuQueue setMaxConcurrentOperationCount:1];
    
    __weak ViewController *weakSelf = self;
    CMDeviceMotionHandler dmHandler = ^(CMDeviceMotion *motion, NSError *error)
    {
        // Could be nil if the self is released before the callback happens.
        if (weakSelf) {
            [weakSelf processDeviceMotion:motion withError:error];
        }
    };
    
    [_motionManager startDeviceMotionUpdatesToQueue:_imuQueue withHandler:dmHandler];

    NSLog(@"setupIMU. ended");
}

- (void)processDeviceMotion:(CMDeviceMotion *)motion withError:(NSError *)error
{
    //NSLog(@"processDeviceMotion. started"); // testlog

    if (_slamState.roomCaptureState == RoomCaptureStatePoseInitialization)
    {
        // Update our gravity vector, it will be used by the cube placement initializer.
        _lastCoreMotionGravity = GLKVector3Make (motion.gravity.x, motion.gravity.y, motion.gravity.z);
    }
    
    if (_slamState.roomCaptureState == RoomCaptureStatePoseInitialization || _slamState.roomCaptureState == RoomCaptureStateScanning)
    {
        //NSLog(@"processDeviceMotion.updateCameraPoseWithMotion "); // testlog
        // The tracker is more robust to fast moves if we feed it with motion data.
        [_slamState.tracker updateCameraPoseWithMotion:motion];
    }
}

#pragma mark - Message Display

- (void)showTrackingMessage:(NSString*)message
{
    self.trackingMessageLabel.text = message;
    self.trackingMessageLabel.hidden = NO;
}

- (void)hideTrackingErrorMessage
{
    self.trackingMessageLabel.hidden = YES;
}

- (void)showAppStatusMessage:(NSString *)msg
{
    _appStatus.needsDisplayOfStatusMessage = true;
    [self.view.layer removeAllAnimations];
    
    [self.appStatusMessageLabel setText:msg];
    [self.appStatusMessageLabel setHidden:NO];
    
    // Progressively show the message label.
    [self.view setUserInteractionEnabled:false];
    [UIView animateWithDuration:0.5f animations:^{
        self.appStatusMessageLabel.alpha = 1.0f;
    }completion:nil];
}

- (void)hideAppStatusMessage
{
    if (!_appStatus.needsDisplayOfStatusMessage)
        return;
    
    _appStatus.needsDisplayOfStatusMessage = false;
    [self.view.layer removeAllAnimations];
    
    __weak ViewController *weakSelf = self;
    [UIView animateWithDuration:0.5f
                     animations:^{
                         weakSelf.appStatusMessageLabel.alpha = 0.0f;
                     }
                     completion:^(BOOL finished) {
                         // If nobody called showAppStatusMessage before the end of the animation, do not hide it.
                         if (!_appStatus.needsDisplayOfStatusMessage)
                         {
                             // Could be nil if the self is released before the callback happens.
                             if (weakSelf) {
                                 [weakSelf.appStatusMessageLabel setHidden:YES];
                                 [weakSelf.view setUserInteractionEnabled:true];
                             }
                         }
                     }];
}

-(void)updateAppStatusMessage
{
    // Skip everything if we should not show app status messages (e.g. in viewing state).
    if (_appStatus.statusMessageDisabled)
    {
        [self hideAppStatusMessage];
        return;
    }
    
    // First show sensor issues, if any.
    switch (_appStatus.sensorStatus)
    {
        case AppStatus::SensorStatusOk:
        {
            break;
        }
            
        case AppStatus::SensorStatusNeedsUserToConnect:
        {
            [self showAppStatusMessage:_appStatus.pleaseConnectSensorMessage];
            return;
        }
            
        case AppStatus::SensorStatusNeedsUserToCharge:
        {
            [self showAppStatusMessage:_appStatus.pleaseChargeSensorMessage];
            return;
        }
    }
    
    // Color camera without calibration (e.g. not iPad).
    if (!_appStatus.colorCameraIsCalibrated)
    {
        [self showAppStatusMessage:_appStatus.needCalibratedColorCameraMessage];
        return;
    }
    
    // Color camera permission issues.
    if (!_appStatus.colorCameraIsAuthorized)
    {
        [self showAppStatusMessage:_appStatus.needColorCameraAccessMessage];
        return;
    }
    
    // Finally background processing feedback.
    switch (_appStatus.backgroundProcessingStatus)
    {
        case AppStatus::BackgroundProcessingStatusIdle:
        {
            break;
        }
            
        case AppStatus::BackgroundProcessingStatusFinalizing:
        {
            [self showAppStatusMessage:_appStatus.finalizingMeshMessage];
            return;
        }
    }
    
    // If we reach this point, no status to show.
    [self hideAppStatusMessage];
}

#pragma mark - UI Callbacks

// Manages whether we can let the application sleep.
-(void)updateIdleTimer
{
    if ([self isStructureConnectedAndCharged] && [self currentStateNeedsSensor])
    {
        // Do not let the application sleep if we are currently using the sensor data.
        [[UIApplication sharedApplication] setIdleTimerDisabled:YES];
    }
    else
    {
        // Let the application sleep if we are only viewing the mesh or if no sensors are connected.
        [[UIApplication sharedApplication] setIdleTimerDisabled:NO];
    }
}

- (IBAction)saveToFIleSwitch:(id)sender {
}

- (IBAction)roomSizeSliderValueChanged:(id)sender
{
    [self actionOnRoomSizeSliderValueChanged];
}

- (IBAction)nearModeSwitchValueChanged:(id)sender {
    [_sensorController setHighGainEnabled:_nearModeSwitch.isOn?NO:YES];
    NSLog(@"Sensor gain changed. %d", _nearModeSwitch.isOn?NO:YES);
}

- (IBAction)mapperDepthThlesholdSliderValueChanged:(id)sender {
    self.mapperDepthThresholdSliderLabel.text = [NSString stringWithFormat:@"%.3f", self.mapperDepthThresholdSlider.value ];
}

- (IBAction)uiHideButton:(id)sender {
}

- (void)actionOnResolutionSliderValueChanged {
    _options.initialVolumeResolutionInMeters = self.resolutionSlider.value;
    
    self.resolutionSliderLabel.text = [NSString stringWithFormat:@"%.3f", self.resolutionSlider.value ];

}

- (void)actionOnRoomSizeSliderValueChanged {
    
    float scale = self.roomSizeSlider.value;
    
    GLKVector3 newVolumeSize = GLKVector3MultiplyScalar(_options.initialVolumeSizeInMeters, scale);
    //newVolumeSize.y = std::max (newVolumeSize.y, _options.minVerticalVolumeSize);
    
    // Helper function.
    auto keepInRange = [](float value, float minValue, float maxValue)
    {
        if (value > maxValue) return maxValue;
        if (value < minValue) return minValue;
        return value;
    };
    
    // Make sure the volume size remains between 3 meters and 10 meters.
    //newVolumeSize.x = keepInRange (newVolumeSize.x, 3.f, 10.f);
    //newVolumeSize.y = keepInRange (newVolumeSize.y, 3.f, 10.f);
    //newVolumeSize.z = keepInRange (newVolumeSize.z, 3.f, 10.f);
    // tanaka changed
    newVolumeSize.x = keepInRange (newVolumeSize.x, 0.3f, 20.f);
    newVolumeSize.y = keepInRange (newVolumeSize.y, 0.3f, 20.f);
    newVolumeSize.z = keepInRange (newVolumeSize.z, 0.3f, 20.f);
    
    [self.roomSizeLabel setText:[NSString stringWithFormat:@"%.3f x %.3f x %.3f meters", newVolumeSize.x, newVolumeSize.y, newVolumeSize.z]];
    
    [self adjustVolumeSize:newVolumeSize];
    
    
    self.roomSizeSliderLabel.text = [NSString stringWithFormat:@"%.3f", self.roomSizeSlider.value ];  // tanaka add

}


// スキャンボタンが押された場合の処理
- (IBAction)scanButtonPressed:(id)sender
{
    [self doSaveInitializeAction];
    
    [self enterScanningState];
}

// リセットボタンが押された場合の処理
- (IBAction)resetButtonPressed:(id)sender
{
    recordMeshNum = 0;
    savedFrameCount = 0;
    [recordMeshList removeAllObjects];
    [scanGpsDataList removeAllObjects];
    [scanFrameDateList removeAllObjects];       // add 2016.6
    [slamStateList removeAllObjects];
    NSUInteger elements = [recordMeshList count];
    //self.recFramesValueLabel.text = [NSString stringWithFormat:@"%lu", (unsigned long)elements];
    
    // Handles simultaneous press of Done & Reset.
    if(self.doneButton.hidden) return;
    
    [self resetSLAM];
    [self enterPoseInitializationState];
    [lm stopUpdatingLocation];
}

- (IBAction)doneButtonPressed:(id)sender
{
    NSLog(@"doneButtonPressed: start");
    
    // Handles simultaneous press of Done & Reset.
    if(self.doneButton.hidden) return;
    
    if (self.recordToMemorySwitch.isOn) {
        [self saveDataMemoryToFile];
    }
    
    [self resetButtonPressed:self.resetButton];
    
    //[self enterFinalizingState];  //default
    
}


- (IBAction)uiHideButtonPressed:(id)sender
{
    [self doUiHideAction];
}


- (void)doUiHideAction {
    if (!uiHideFlag) {
        
        uiHideFlag = true;
        
        self.scanButton.hidden = YES;
        self.doneButton.hidden = YES;
        self.resetButton.hidden = YES;
        
        self.roomSizeSlider.hidden = YES;
        self.resolutionSlider.hidden = YES;
        self.intervalSlider.hidden = YES;
        
        self.colorScanSwitch.hidden = YES;
        self.saveToFileSwitch.hidden = YES;
        self.drawModeSwitch.hidden = YES;
        
        self.roomSizeUiLabel.hidden = YES;
        self.resolutionLabel.hidden = YES;
        self.intervalLabel.hidden = YES;
        
        self.debugInfoLabel.hidden = YES;
        
        self.drawModeSwitchLabel.hidden = YES;
        self.colorScanSwitchLabel.hidden = YES;
        self.saveToFileSwitchLabel.hidden = YES;

        self.intervalSliderLabel.hidden = YES;
        self.resolutionSliderLabel.hidden = YES;
        self.roomSizeSliderLabel.hidden = YES;
        
        self.trackingSmallObjectSwitch.hidden = YES;
        self.trackerQualityAccurateSwitch.hidden = YES;

        // Cannot be lost in cube placement mode.
        _trackingMessageLabel.hidden = YES;
        
        // Hide the room size controls.
        self.roomSizeLabel.hidden = YES;
        self.roomSizeSlider.hidden = YES;

    } else {
        
        uiHideFlag = false;

        if (_slamState.roomCaptureState == RoomCaptureStatePoseInitialization) {
            self.scanButton.hidden = NO;
            self.doneButton.hidden = YES;
            self.resetButton.hidden = YES;
            
            self.roomSizeSlider.hidden = NO;
            self.resolutionSlider.hidden = NO;
            self.intervalSlider.hidden = NO;
            
            self.colorScanSwitch.hidden = NO;
            self.saveToFileSwitch.hidden = NO;
            self.drawModeSwitch.hidden = NO;
            
            self.roomSizeUiLabel.hidden = NO;
            self.resolutionLabel.hidden = NO;
            self.intervalLabel.hidden = NO;
            
            self.trackingSmallObjectSwitch.hidden = NO;
            self.trackerQualityAccurateSwitch.hidden = NO;
            
            self.debugInfoLabel.hidden = NO;
            
            self.drawModeSwitchLabel.hidden = NO;
            self.colorScanSwitchLabel.hidden = NO;
            self.saveToFileSwitchLabel.hidden = NO;
            
            self.intervalSliderLabel.hidden = NO;
            self.resolutionSliderLabel.hidden = NO;
            self.roomSizeSliderLabel.hidden = NO;
            
            // Cannot be lost in cube placement mode.
            _trackingMessageLabel.hidden = YES;
            
            // Hide the room size controls.
            self.roomSizeLabel.hidden = NO;
            self.roomSizeSlider.hidden = NO;
            
        } else if (_slamState.roomCaptureState == RoomCaptureStateScanning) {
            self.scanButton.hidden = YES;
            self.doneButton.hidden = NO;
            self.resetButton.hidden = NO;
            
            self.roomSizeSlider.hidden = NO;
            self.resolutionSlider.hidden = NO;
            self.intervalSlider.hidden = NO;
            
            self.colorScanSwitch.hidden = NO;
            self.saveToFileSwitch.hidden = NO;
            self.drawModeSwitch.hidden = NO;
            
            self.roomSizeUiLabel.hidden = NO;
            self.resolutionLabel.hidden = NO;
            self.intervalLabel.hidden = NO;
            
            self.trackingSmallObjectSwitch.hidden = NO;
            self.trackerQualityAccurateSwitch.hidden = NO;

            self.debugInfoLabel.hidden = NO;
            
            self.drawModeSwitchLabel.hidden = NO;
            self.colorScanSwitchLabel.hidden = NO;
            self.saveToFileSwitchLabel.hidden = NO;
            
            self.intervalSliderLabel.hidden = NO;
            self.resolutionSliderLabel.hidden = NO;
            self.roomSizeSliderLabel.hidden = NO;
            
            // Cannot be lost in cube placement mode.
            _trackingMessageLabel.hidden = YES;
            
            // Hide the room size controls.
            self.roomSizeLabel.hidden = NO;
            self.roomSizeSlider.hidden = NO;

        } else if (_slamState.roomCaptureState == RoomCaptureStateViewing) {
        
        }
        
    }
    
}


- (IBAction)uiHideButtonTouchDowned:(id)sender {
    [self doUiHideAction];
}

- (IBAction)uiSaveSettingButtonPressed:(id)sender {
    NSUserDefaults *defaults = [NSUserDefaults standardUserDefaults];
    [defaults setBool:_nearModeSwitch.isOn forKey:@"nearModeSwitch"];
    [defaults setBool:_recordToMemorySwitch.isOn forKey:@"recordToMemorySwitch"];
    [defaults setBool:_recordGpsSwitch.isOn forKey:@"recordGpsSwitch"];
    [defaults setBool:_drawModeSwitch.isOn forKey:@"drawModeSwitch"];
    [defaults setBool:_saveToFileSwitch.isOn forKey:@"saveToFileSwitch"];
    [defaults setBool:_colorScanSwitch.isOn forKey:@"colorScanSwitch"];
    [defaults setBool:_trackingSmallObjectSwitch.isOn forKey:@"trackingSmallObjectSwitch"];
    [defaults setBool:_trackerQualityAccurateSwitch.isOn forKey:@"trackerQualityAccurateSwitch"];
    [defaults setBool:_fixedTrackingSwitch.isOn forKey:@"fixedTrackingSwitch"];
    
    [defaults setInteger:_intervalSlider.value forKey:@"intervalSlider"];
    [defaults setFloat:_resolutionSlider.value  forKey:@"resolutionSlider"];
    [defaults setFloat:_roomSizeSlider.value  forKey:@"roomSizeSlider"];
    BOOL successful = [defaults synchronize];
    if (successful) {
        NSLog(@"%@", @"ユーザデフォルト設定データの保存に成功しました。");
    } else {
        NSLog(@"%@", @"ユーザデフォルト設定データの保存に失敗しました。");
    }
}

- (IBAction)roomSizeSliderTouchDown:(id)sender {
    self.roomSizeLabel.hidden = NO;
}

- (IBAction)roomSizeSliderTouchUpInside:(id)sender {
    self.roomSizeLabel.hidden = YES;
}

- (IBAction)roomSizeSliderTouchUpOutside:(id)sender {
    self.roomSizeLabel.hidden = YES;
}


// tanaka add
- (IBAction)resolutionSliderValueChanged:(id)sender
{
    [self actionOnResolutionSliderValueChanged];
}

- (IBAction)intervalSliderValueChanged:(id)sender {
    self.intervalSliderLabel.text = [NSString stringWithFormat:@"%d", (int)self.intervalSlider.value ];
}
/*
- (IBAction)roomSizeSliderValueChanged:(id)sender {

}*/





#pragma mark - MeshViewController delegates

- (void)presentMeshViewer:(STMesh *)mesh withCameraPose:(GLKMatrix4)cameraPose
{
    [(EAGLView *)_meshViewController.view setContext:_display.context];
    [EAGLContext setCurrentContext:_display.context];
    
    // Horizontal field of view.
    float fovXRadians = 90.f/180.f * M_PI;
    float aspectRatio = 4.f/3.f;
    
    if (mesh)
    {
        [_meshViewController uploadMesh:mesh];
    }
    else
    {
        NSLog(@"Error: no mesh!");
    }
    [_meshViewController setHorizontalFieldOfView:fovXRadians aspectRatio:aspectRatio];
    [_meshViewController setCameraPose:cameraPose];
    
    [self presentViewController:_meshViewNavigationController animated:YES completion:^{}];
}

- (void)meshViewWillDismiss
{
    [_meshViewController hideMeshViewerMessage:_meshViewController.meshViewerMessageLabel];
    
    // Make sure we don't keep background task running.
    if (_holeFillingTask)
    {
        [_holeFillingTask cancel];
        _holeFillingTask = nil;
    }
    if (_colorizeTask)
    {
        [_colorizeTask cancel];
        _colorizeTask = nil;
    }
}

- (void)meshViewDidDismiss
{
    // Restart the sensor.
    [self connectToStructureSensorAndStartStreaming];
    
    _appStatus.statusMessageDisabled = false;
    [self updateAppStatusMessage];
    
    // Reset the tracker, mapper, etc.
    [self resetSLAM];
    [self enterPoseInitializationState];
}

- (void)meshViewDidRequestHoleFilling
{
    if (_holeFilledMesh)
    {
        // If already available, just re-upload it.
        [_meshViewController uploadMesh:_holeFilledMesh];
    }
    else
    {
        // Otherwise compute it now.
        [_meshViewController.holeFillingSwitch setEnabled:NO];
        [self startHoleFillingAndColorizingTasks];
    }
}

- (void)meshViewDidRequestRegularMesh
{
    [_meshViewController uploadMesh:_colorizedMesh];
}

-(void) backgroundTask:(STBackgroundTask*)sender didUpdateProgress:(double)progress
{
    if(sender == _holeFillingTask)
    {
        dispatch_async(dispatch_get_main_queue(), ^{
            [_meshViewController showMeshViewerMessage:_meshViewController.meshViewerMessageLabel msg:[NSString stringWithFormat:@"Hole filling: % 3d%%", int(progress*80)]];
        });
    }
    else if(sender == _colorizeTask)
    {
        dispatch_async(dispatch_get_main_queue(), ^{
            [_meshViewController showMeshViewerMessage:_meshViewController.meshViewerMessageLabel msg:[NSString stringWithFormat:@"Coloring Mesh: % 3d%%", int(progress*20)+80]];
        });
    }
}


- (void)startHoleFillingAndColorizingTasks
{
    // Safely copy the scene mesh so we can operate on it while it is being used for rendering.
    STMesh* sceneMesh = [_slamState.scene lockAndGetSceneMesh];
    STMesh* sceneMeshCopy = [[STMesh alloc] initWithMesh:sceneMesh];
    [_slamState.scene unlockSceneMesh];
    
    // If an old task is still running, wait until it completes.
    if (_holeFillingTask)
    {
        [_holeFillingTask waitUntilCompletion];
        _holeFillingTask = nil;
    }
    
    STBackgroundTask* holeFillingTask = [STMesh newFillHolesTaskWithMesh:sceneMeshCopy completionHandler:^(STMesh* result, NSError *error) {
        
        // We must execute on main thread to check if the operation was canceled right before completion.
        dispatch_async(dispatch_get_main_queue(), ^{
            
            _holeFilledMesh = result;
            
            if (_holeFillingTask.isCancelled)
            {
                // If we reach this point, it means the task was cancelled after we already had the result ready.
                NSLog(@"Error! %s", [[error localizedDescription] UTF8String]);
                return;
            }
            
            // Release the handle on the completed task.
            _holeFillingTask = nil;
            
            // Now colorize the hole filled mesh.
            STBackgroundTask* colorizeTask = [STColorizer
                                              newColorizeTaskWithMesh:_holeFilledMesh
                                              scene:_slamState.scene
                                              keyframes:[_slamState.keyFrameManager getKeyFrames]
                                              completionHandler:^(NSError *error)
                                              {
                                                  if (error == nil)
                                                  {
                                                      [self performSelectorOnMainThread:@selector(holeFillingDone) withObject:nil waitUntilDone:NO];
                                                  }
                                                  else
                                                  {
                                                      NSLog(@"Error! %s", [[error localizedDescription] UTF8String]);
                                                  }
                                                  
                                                  _colorizeTask = nil; // release the handle on the completed task.
                                              }
                                              options:@{kSTColorizerTypeKey: @(STColorizerTextureMapForRoom) }
                                              error:nil];
            _colorizeTask = colorizeTask;
            _colorizeTask.delegate = self;
            [_colorizeTask start];
        });
    }];
    
    // Keep a reference so we can monitor progress
    _holeFillingTask = holeFillingTask;
    _holeFillingTask.delegate = self;
    
    [_holeFillingTask start];
}

- (void)holeFillingDone
{
    [_meshViewController uploadMesh:_holeFilledMesh];
    [_meshViewController hideMeshViewerMessage:_meshViewController.meshViewerMessageLabel];
    [_meshViewController.holeFillingSwitch setEnabled:YES];
}



-(void) doSaveInitializeAction {
    
    savedFrameCount = 0;
    trackingOkCounter = 0;
    allFrameCounter = 0;
    //_recordMeshNum;
    
    //mvcRecordMeshList;

    scanFrameTime = 1;      // add by tanaka
    scanFrameCount = 0;      // add by tanaka
    recordMeshNum = 0;      // add by tanaka
    [recordMeshList removeAllObjects];       // add 2016.6
    [scanFrameDateList removeAllObjects];       // add 2016.6
    [scanGpsDataList removeAllObjects];       // add 2016.6
    
    ownKeyframeCounts = 0;
    scanStartDate = [NSDate date];

    if (_recordGpsSwitch.isOn) {
        [lm startUpdatingLocation];         // デリゲートに通知が来るようになる
    }
    
    //---------------------------------
    int newSaveNum = 0;
    
    NSLog(@"mvcSaveButtonPressed saveWithData start2");
    
    if (_saveToFileSwitch.isOn) {
    
        // Documentsフォルダを得る
        NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *DocumentsDirPath = [paths objectAtIndex:0];
        
        NSLog(@"Documents folder path: %s", [DocumentsDirPath UTF8String]);
        
        saveBaseDirPath = [NSString stringWithFormat:@"%s/%s", [DocumentsDirPath UTF8String], [saveBaseDirName UTF8String]];
        
        NSLog(@"artDkt folder path: %s", [saveBaseDirPath UTF8String]);
        
        // 新しい保存のための番号を作るため、現在一番新しいセーブの番号を取得する
        NSFileManager *tFileManager = [NSFileManager defaultManager];
        NSLog(@"filePathScan: %s", [saveBaseDirPath UTF8String] );
        for (NSString *content in [tFileManager contentsOfDirectoryAtPath:saveBaseDirPath error:nil ]) {
            const char *chars = [content UTF8String];
            
            NSString *str = [NSString stringWithCString: chars encoding:NSUTF8StringEncoding];
            
            NSLog(@"filePathList: %s", chars);
            
            //NSString型をInt型に
            int oldSaveId = [ str intValue ];
            if (oldSaveId >= newSaveNum) {
                newSaveNum = oldSaveId;
            }
        }
        newSaveNum += 1;
        nowSaveDirNum = newSaveNum;
    
        NSString *modelDirPath = [NSString stringWithFormat:@"%s/%d", [saveBaseDirPath UTF8String], nowSaveDirNum];
        
        NSLog(@"make folder  proccess!");
        // 一撮影ごとのフォルダを作る
        {
            NSFileManager *tFileManager = [NSFileManager defaultManager];
            NSError *error = nil;
            BOOL created = [tFileManager createDirectoryAtPath:modelDirPath
                                  withIntermediateDirectories:YES
                                                   attributes:nil
                                                        error:&error];
            
            if (!error){
                NSLog(@"error: %@", error);
            } else {
                NSLog(@"folder make success!");
            }
            
        }
    }
    
    
    NSString *scanTimeRecordList = @"";
    
    
    
    NSLog(@"saveWithData end2");
    
    
}

- (void)startLocation {
    [lm startUpdatingLocation];
}

- (void)stopLocation {
    [lm stopUpdatingLocation];
}

- (void)locationManager:(CLLocationManager *)manager didUpdateLocations:(NSArray *)locations {
   
    
    CLLocation* location = [locations lastObject];
    
    // locationManager sometimes return cache object, so i shold check newLocation is row or cache.
    NSTimeInterval howRecent = [location.timestamp timeIntervalSinceNow];
    if (std::abs(howRecent) < 15.0) {
        NSDate* timestamp = location.timestamp;
        //NSTimeInterval howRecent = [timestamp timeIntervalSinceNow];
        //if (abs(howRecent) < 15.0) {
        NSLog(@"緯度 %+.6f, 経度 %+.6f\n", location.coordinate.latitude, location.coordinate.longitude);
        
        _latitudeLabel.text = [NSString stringWithFormat:@"%+.6f", location.coordinate.latitude];
        _longitudeLabel.text = [NSString stringWithFormat:@"%+.6f", location.coordinate.longitude];
        _gpsAltitudeLabel.text = [NSString stringWithFormat:@"%.6f \n", location.altitude];
        _gpsDescriptionLabel.text = [NSString stringWithFormat:@"%@ \n", [location description]];

        _gpsHorizontalAccuracy.text = [NSString stringWithFormat:@"%6.f \n", [location horizontalAccuracy]];
        _gpsVerticalAccuracy.text = [NSString stringWithFormat:@"%6.f \n", [location verticalAccuracy]];
        _gpsSpeed.text = [NSString stringWithFormat:@"%.6f \n", [location speed]];
        _gpsCourse.text = [NSString stringWithFormat:@"%.6f \n", [location course]];

        NSDateFormatter *df = [[NSDateFormatter alloc] init];
        [df setDateFormat:@"yyyy/MM/dd HH:mm:ss"];
        _gpsTimestampLabel.text = [df stringFromDate:timestamp];

        recentLocation = location;

    }
}



@end
