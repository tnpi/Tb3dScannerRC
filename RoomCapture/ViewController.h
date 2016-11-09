/*
 This file is part of the Structure SDK.
 Copyright © 2016 Occipital, Inc. All rights reserved.
 http://structure.io
 */

#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>
#define HAS_LIBCXX
#import <Structure/Structure.h>

#import "CalibrationOverlay.h"
#import "MeshViewController.h"

#include <vector>
//#include <stdio.h>
#import <CoreLocation/CoreLocation.h>

struct Options
{
    // The initial scanning volume size will be 6.0 x 4.0 x 6.0 meters
    GLKVector3 initialVolumeSizeInMeters = GLKVector3Make (10.0f, 10.f, 10.0f);
    // tanaka changed
    //GLKVector3 initialVolumeSizeInMeters = GLKVector3Make (6.f, 6.f, 6.f);
    
    // Resolution for the initialVolumeSizeInMeters. Will get scaled if we change the volume size.
    // 0.05 初期値：ラフだが高速
    // 0.01 高精細だがやや低速
    float initialVolumeResolutionInMeters = 0.05; // coarse, but gives good performance
    
    // The minimal vertical volume size is fixed, since the ceiling is not likely to be very low.
    float minVerticalVolumeSize = 1.f;
        
    // The maximum of keyframes for keyFrameManager. More won't fit in a single OpenGL texture.
    int maxNumKeyframes = 48;
    
    // Take a new keyframe in the rotation difference is higher than 20 degrees.
    float maxKeyFrameRotation = 20.0f * (M_PI / 180.f);
    
    // Take a new keyframe if the translation difference is higher than 30 cm.
    float maxKeyFrameTranslation = 0.3;
    
    // Threshold to consider that the rotation motion was small enough for a frame to be accepted
    // as a keyframe. This avoids capturing keyframes with strong motion blur / rolling shutter.
    float maxKeyframeRotationSpeedInDegreesPerSecond = 1.f;
    
    // Threshold to pop a warning to the user if he's exploring too far away since this demo is optimized
    // for a rotation around oneself.
    float maxDistanceFromInitialPositionInMeters = 1.f;
    
    // Fixed focus position of the color camera.
    float colorCameraLensPosition = 0.75f; // 0.75 gives pretty good focus for a room scale.
    
    // Whether we should use depth aligned to the color viewpoint when Structure Sensor was calibrated.
    // This setting may get overwritten to false if no color camera can be used.
    //bool useHardwareRegisteredDepth = false;
    bool useHardwareRegisteredDepth = true;     // tanaka add
    
    // Whether to enable an expensive per-frame depth accuracy refinement.
    // Note: this option requires useHardwareRegisteredDepth to be set to false.
    //const bool applyExpensiveCorrectionToDepth = true;
    const bool applyExpensiveCorrectionToDepth = false;     // tanaka add for get fast speed
};

enum RoomCaptureState
{
    // Defining the volume to scan
    RoomCaptureStatePoseInitialization = 0,
    
    // Scanning
    RoomCaptureStateScanning,
    
    // Finalizing the mesh
    RoomCaptureStateFinalizing,
    
    // Visualizing the mesh
    RoomCaptureStateViewing,
    
    RoomCaptureStateNumStates
};

// SLAM-related members.
struct SlamData
{
    RoomCaptureState roomCaptureState = RoomCaptureStatePoseInitialization;
    bool initialized = false;
    
    NSTimeInterval prevFrameTimeStamp = -1.0;
    
    STScene *scene = NULL;
    STTracker *tracker = NULL;
    STMapper *mapper = NULL;
    STCameraPoseInitializer *cameraPoseInitializer = NULL;
    STKeyFrameManager *keyFrameManager = NULL;
    
    GLKVector3 volumeSizeInMeters = GLKVector3Make (NAN, NAN, NAN);
};

struct AppStatus
{
    NSString* const pleaseConnectSensorMessage = @"Please connect Structure Sensor.";
    NSString* const pleaseChargeSensorMessage = @"Please charge Structure Sensor.";
    
    NSString* const needColorCameraAccessMessage = @"This app requires camera access to capture rooms.\nAllow access by going to Settings → Privacy → Camera.";
    NSString* const needCalibratedColorCameraMessage = @"This app requires an iOS device with a supported bracket.";
    
    NSString* const finalizingMeshMessage = @"Finalizing model...";
    
    enum SensorStatus
    {
        SensorStatusOk,
        SensorStatusNeedsUserToConnect,
        SensorStatusNeedsUserToCharge,
    };
    
    enum BackgroundProcessingStatus
    {
        BackgroundProcessingStatusIdle,
        BackgroundProcessingStatusFinalizing
    };
    
    SensorStatus sensorStatus = SensorStatusOk;
    
    // Whether iOS camera access was granted by the user.
    bool colorCameraIsAuthorized = true;
    
    // Whether the current iOS device has a supported bracket and thus a calibrated color camera.
    bool colorCameraIsCalibrated = true;
    
    BackgroundProcessingStatus backgroundProcessingStatus = BackgroundProcessingStatusIdle;
    
    // Whether there is currently a message to show.
    bool needsDisplayOfStatusMessage = false;
    
    // Flag to disable entirely status message display.
    bool statusMessageDisabled = false;
};

// Display related members.
struct DisplayData
{
    ~DisplayData ()
    {
        if (lumaTexture)
        {
            CFRelease (lumaTexture);
            lumaTexture = NULL;
        }
        
        if (chromaTexture)
        {
            CFRelease(chromaTexture);
            chromaTexture = NULL;
        }
        
        if (videoTextureCache)
        {
            CFRelease(videoTextureCache);
            videoTextureCache = NULL;
        }
    }
    
    // OpenGL context.
    EAGLContext *context = nil;
    
    // OpenGL Texture reference for y images.
    CVOpenGLESTextureRef lumaTexture = NULL;
    
    // OpenGL Texture reference for color images.
    CVOpenGLESTextureRef chromaTexture = NULL;
    
    // OpenGL Texture cache for the color camera.
    CVOpenGLESTextureCacheRef videoTextureCache = NULL;
    
    // Shader to render a GL texture as a simple quad. YCbCr version.
    STGLTextureShaderYCbCr *yCbCrTextureShader = nil;
    
    // Shader to render a GL texture as a simple quad. RGBA version.
    STGLTextureShaderRGBA *rgbaTextureShader = nil;
    
    // Used during initialization to show which depth pixels lies in the scanning volume boundaries.
    std::vector<uint8_t> scanningVolumeFeedbackBuffer;
    GLuint scanningVolumeFeedbackTexture = -1;
    
    // OpenGL viewport.
    GLfloat viewport[4] = {0,0,0,0};
    
    // OpenGL projection matrix for the color camera.
    GLKMatrix4 colorCameraGLProjectionMatrix = GLKMatrix4Identity;
};

@interface ViewController : UIViewController <STBackgroundTaskDelegate, MeshViewDelegate, AVCaptureVideoDataOutputSampleBufferDelegate, UIPopoverControllerDelegate, UIGestureRecognizerDelegate, CLLocationManagerDelegate>
{
    Options _options;
    
    // Manages the app status messages.
    AppStatus _appStatus;
    
    DisplayData _display;
    SlamData _slamState;
    
    STMesh *_colorizedMesh;
    STMesh *_holeFilledMesh;
    
    // Most recent gravity vector from IMU.
    GLKVector3 _lastCoreMotionGravity;
    
    // Structure Sensor controller.
    STSensorController *_sensorController;
    
    // Mesh viewer controllers.
    UINavigationController *_meshViewNavigationController;
    MeshViewController *_meshViewController;
    
    // IMU handling.
    CMMotionManager *_motionManager;
    NSOperationQueue *_imuQueue;
    
    // Handles on background tasks which may be running.
    STBackgroundTask* _holeFillingTask;
    STBackgroundTask* _colorizeTask;
    
    CalibrationOverlay* _calibrationOverlay;
    
    
    // ---------------------------------------------------------------
    int scanFrameCount; // add by tanaka
    int savedFrameCount; // add by tanaka
    int recordMeshNum; // add by tanaka
    
    int scanFrameTime; // add by tanaka
    
    int nowSaveDirNum;
    int ownKeyframeCounts;
    
    NSFileManager *fileManager;
    NSMutableArray *fileList;
    NSString *filePath;
    NSString *basePath;
    NSString *saveBaseDirPath;
    NSString *saveBaseDirName;
    
    NSDate *scanStartDate;
    NSDate *scanNowDate;
    
    NSMutableArray *recordMeshList; // add by tanaka
    NSMutableArray *keyFramesList;
    NSMutableArray *sceneList;

    NSMutableArray *scanFrameDateList;
    NSMutableArray *scanGpsDataList;
    NSMutableArray *scanDoFDataList;
    NSMutableArray *slamStateList;
    NSMutableArray *depthCameraPoseList;
    NSMutableArray *depthFrameList;
    NSMutableArray *colorFrameList;
    NSDate *getSceneMeshDate;
    
    //FILE *fpScanDateListFile;
    NSString *scanDateListFileName;
    NSString *dataTypeFileName;
    
    bool uiHideFlag;
    
    CLLocationManager *lm;
    CLLocation* recentLocation;
    
    bool udNearModeSwitch;
    bool udRecordToMemorySwitch;
    bool udRecordGpsSwitch;
    bool udSaveToFileSwitch;
    bool udColorScanSwitch;
    bool udDrawModeSwitch;
    bool udTrackingSmallObjectSwitch;
    bool udFixedTrackingSwitch;
    bool udTrackerQualityAccurate;
    
    int udIntervalSlider;
    float udResolutionSlider;
    float udRoomSizeSlider;
    
    NSDate *fpsBasetime;   //測定基準時間
    int fpsCount;      //フレーム数
    float fpsFramerate;  //フレームレート
    
    // tanaka rec to memory --------------------------------------------------

    
    NSTimer *clockTimer;
    
    int trackingOkCounter;
    int allFrameCounter;
    
    STDepthFrame *firstGetDepthFrame;
    STColorFrame *firstGetColorFrame;
    GLKMatrix4 firstCameraPoseOnScan;
    
    NSArray *lastKeyFrames;
    STScene *lastScene;
    
}

@property (nonatomic, retain) AVCaptureSession *avCaptureSession;
@property (nonatomic, retain) AVCaptureDevice *videoDevice;

@property (weak, nonatomic) IBOutlet UILabel *appStatusMessageLabel;
@property (weak, nonatomic) IBOutlet UIButton *scanButton;
@property (weak, nonatomic) IBOutlet UIButton *resetButton;
@property (weak, nonatomic) IBOutlet UIButton *doneButton;
@property (weak, nonatomic) IBOutlet UILabel *trackingMessageLabel;
@property (weak, nonatomic) IBOutlet UILabel *roomSizeLabel;
@property (weak, nonatomic) IBOutlet UISlider *roomSizeSlider;
@property (weak, nonatomic) IBOutlet UISlider *resolutionSlider;
@property (weak, nonatomic) IBOutlet UISlider *mapperDepthThresholdSlider;
@property (weak, nonatomic) IBOutlet UISwitch *drawModeSwitch;
@property (weak, nonatomic) IBOutlet UISwitch *colorScanSwitch;
@property (weak, nonatomic) IBOutlet UISwitch *saveToFileSwitch;
@property (weak, nonatomic) IBOutlet UISwitch *recordToMemorySwitch;
@property (weak, nonatomic) IBOutlet UISwitch *nearModeSwitch;
@property (weak, nonatomic) IBOutlet UISwitch *recordGpsSwitch;
@property (weak, nonatomic) IBOutlet UISwitch *trackingSmallObjectSwitch;
@property (weak, nonatomic) IBOutlet UISwitch *liveWireframeSwitch;
@property (weak, nonatomic) IBOutlet UISwitch *trackerQualityAccurateSwitch;
@property (weak, nonatomic) IBOutlet UISwitch *fixedTrackingSwitch;
@property (weak, nonatomic) IBOutlet UISlider *intervalSlider;

@property (weak, nonatomic) IBOutlet UILabel *resolutionLabel;
@property (weak, nonatomic) IBOutlet UILabel *intervalLabel;

@property (weak, nonatomic) IBOutlet UILabel *debugInfoLabel;
@property (weak, nonatomic) IBOutlet UILabel *roomSizeUiLabel;

@property (weak, nonatomic) IBOutlet UIButton *uiHideButton;
@property (weak, nonatomic) IBOutlet UIButton *saveSettingsButton;
@property (weak, nonatomic) IBOutlet UILabel *mapperDepthThresholdSliderLabel;

@property (weak, nonatomic) IBOutlet UILabel *drawModeSwitchLabel;
@property (weak, nonatomic) IBOutlet UILabel *colorScanSwitchLabel;
@property (weak, nonatomic) IBOutlet UILabel *saveToFileSwitchLabel;
@property (weak, nonatomic) IBOutlet UILabel *recordToMemorySwitchLabel;
@property (weak, nonatomic) IBOutlet UILabel *nearModeSwitchLabel;
@property (weak, nonatomic) IBOutlet UILabel *recordGpsSwitchLabel;

@property (weak, nonatomic) IBOutlet UILabel *intervalSliderLabel;
@property (weak, nonatomic) IBOutlet UILabel *resolutionSliderLabel;
@property (weak, nonatomic) IBOutlet UILabel *roomSizeSliderLabel;
@property (weak, nonatomic) IBOutlet UILabel *latitudeLabel;
@property (weak, nonatomic) IBOutlet UILabel *longitudeLabel;
@property (weak, nonatomic) IBOutlet UILabel *gpsTimestampLabel;
@property (weak, nonatomic) IBOutlet UILabel *gpsAltitudeLabel;
@property (weak, nonatomic) IBOutlet UILabel *gpsDescriptionLabel;
@property (weak, nonatomic) IBOutlet UILabel *gpsHorizontalAccuracy;
@property (weak, nonatomic) IBOutlet UILabel *gpsVerticalAccuracy;
@property (weak, nonatomic) IBOutlet UILabel *gpsSpeed;
@property (weak, nonatomic) IBOutlet UILabel *gpsCourse;

@property (weak, nonatomic) IBOutlet UILabel *scanFpsLabel;

@property (weak, nonatomic) IBOutlet UILabel *clockLabel;



- (IBAction)scanButtonPressed:(id)sender;
- (IBAction)resetButtonPressed:(id)sender;
- (IBAction)doneButtonPressed:(id)sender;
- (IBAction)uiHideButtonPressed:(id)sender;
- (IBAction)uiHideButtonTouchDowned:(id)sender;
- (IBAction)uiSaveSettingButtonPressed:(id)sender;

- (IBAction)roomSizeSliderTouchDown:(id)sender;
- (IBAction)roomSizeSliderTouchUpInside:(id)sender;
- (IBAction)roomSizeSliderTouchUpOutside:(id)sender;
//- (IBAction)resolutionSliderValueChanged:(id)sender;
//- (IBAction)drawModeSwitchPressed:(id)sender;
//- (IBAction)colorScanSwitchPressed:(id)sender;
- (IBAction)resolutionSliderValueChanged:(id)sender;
- (IBAction)intervalSliderValueChanged:(id)sender;
- (IBAction)roomSizeSliderValueChanged:(id)sender;
- (IBAction)nearModeSwitchValueChanged:(id)sender;
- (IBAction)mapperDepthThlesholdSliderValueChanged:(id)sender;



- (void)enterPoseInitializationState;
- (void)enterScanningState;
- (void)enterViewingState;
- (void)adjustVolumeSize:(GLKVector3)volumeSize;
- (void)updateAppStatusMessage;
- (BOOL)currentStateNeedsSensor;
- (void)updateIdleTimer;
- (void)showTrackingMessage:(NSString*)message;
- (void)hideTrackingErrorMessage;

// tanaka add -----
- (void)colorizeMesh;
- (void)countFps;
- (void)saveDataMemoryToFile;
- (BOOL)createFile:(NSString *)localFilePath;
- (void)updateClockLabel;

@end
