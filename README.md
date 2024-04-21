## NOTICE

This repository contains the public FTC SDK for the CENTERSTAGE (2023-2024) competition season.

## Welcome!
This GitHub repository contains the source code that is used to build an Android app to control a *FIRST* Tech Challenge competition robot.  To use this SDK, download/clone the entire project to your local computer.

## Requirements
To use this Android Studio project, you will need Android Studio 2021.2 (codename Chipmunk) or later.

To program your robot in Blocks or OnBot Java, you do not need Android Studio.

## Getting Started
If you are new to robotics or new to the *FIRST* Tech Challenge, then you should consider reviewing the [FTC Blocks Tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html) to get familiar with how to use the control system:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Blocks Online Tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html)

Even if you are an advanced Java programmer, it is helpful to start with the [FTC Blocks tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html), and then migrate to the [OnBot Java Tool](https://ftc-docs.firstinspires.org/programming_resources/onbot_java/OnBot-Java-Tutorial.html) or to [Android Studio](https://ftc-docs.firstinspires.org/programming_resources/android_studio_java/Android-Studio-Tutorial.html) afterwards.

## Downloading the Project
If you are an Android Studio programmer, there are several ways to download this repo.  Note that if you use the Blocks or OnBot Java Tool to program your robot, then you do not need to download this repository.

* If you are a git user, you can clone the most current version of the repository:

<p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;git clone https://github.com/FIRST-Tech-Challenge/FtcRobotController.git</p>

* Or, if you prefer, you can use the "Download Zip" button available through the main repository page.  Downloading the project as a .ZIP file will keep the size of the download manageable.

* You can also download the project folder (as a .zip or .tar.gz archive file) from the Downloads subsection of the [Releases](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases) page for this repository.

* The Releases page also contains prebuilt APKs.

Once you have downloaded and uncompressed (if needed) your folder, you can use Android Studio to import the folder  ("Import project (Eclipse ADT, Gradle, etc.)").

## Getting Help
### User Documentation and Tutorials
*FIRST* maintains online documentation with information and tutorials on how to use the *FIRST* Tech Challenge software and robot control system.  You can access this documentation using the following link:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FIRST Tech Challenge Documentation](https://ftc-docs.firstinspires.org/index.html)

Note that the online documentation is an "evergreen" document that is constantly being updated and edited.  It contains the most current information about the *FIRST* Tech Challenge software and control system.

### Javadoc Reference Material
The Javadoc reference documentation for the FTC SDK is now available online.  Click on the following link to view the FTC SDK Javadoc documentation as a live website:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Javadoc Documentation](https://javadoc.io/doc/org.firstinspires.ftc)

### Online User Forum
For technical questions regarding the Control System or the FTC SDK, please visit the FIRST Tech Challenge Community site:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FIRST Tech Challenge Community](https://ftc-community.firstinspires.org/)

### Sample OpModes
This project contains a large selection of Sample OpModes (robot code examples) which can be cut and pasted into your /teamcode folder to be used as-is, or modified to suit your team's needs.

Samples Folder: &nbsp;&nbsp; [/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples)

The readme.md file located in the [/TeamCode/src/main/java/org/firstinspires/ftc/teamcode](TeamCode/src/main/java/org/firstinspires/ftc/teamcode) folder contains an explanation of the sample naming convention, and instructions on how to copy them to your own project space.

# Release Information

## Version 9.1 (20240215-115542)

### Enhancements
* Fixes a problem with Blocks: if the user closes a Block's warning balloon, it will still be closed next time the project is opened in the Blocks editor.
* In the Blocks editor, an alert concerning missing hardware devices is not shown if all the Blocks that use the missing hardware devices are disabled.
* Adds Blocks to support comparing property values CRServo.Direction, DCMotor.Direction, DCMotor.Mode, DCMotor.ZeroPowerBehavior, DigitalChannel.Mode, GyroSensor.HeadingMode, IrSeekerSensor.Mode, and Servo.Direction, to the corresponding enum Block.
* Improves OnBotJava auto-import to correctly import classes when used in certain situations.
* Improves OnBotJava autocomplete to provide better completion options in most cases.
  * This fixes an issue where autocomplete would fail if a method with two or more formal parameters was defined.
* In OnBotJava, code folding support was added to expand and collapse code sections
* In OnBotJava, the copyright header is now automatically collapsed loading new files
* For all Blocks OpMode samples, intro comments have been moved to the RunOpMode comment balloon.
* The Clean up Blocks command in the Blocks editor now positions function Blocks so their comment balloons don't overlap other function Blocks.
* Added Blocks OpMode sample SensorTouch.
* Added Java OpMode sample SensorDigitalTouch.
* Several improvements to VisionPortal
  * Adds option to control whether the stream is automatically started following a `.build()` call on a VisionPortal Builder
  * Adds option to control whether the vision processing statistics overlay is rendered or not
  * VisionPortals now implement the `CameraStreamSource` interface, allowing multiportal users to select which portal is routed to the DS in INIT by calling CameraStreamServer.getInstance().setSource(visionPortal). Can be selected via gamepad, between Camera Stream sessions.
  * Add option to `AprilTagProcessor` to suppress calibration warnings
  * Improves camera calibration warnings
    * If a calibration is scaled, the resolution it was scaled from will be listed
    * If calibrations exist with the wrong aspect ratio, the calibrated resolutions will be listed
  * Fixes race condition which caused app crash when calling `stopStreaming()` immediately followed by `close()` on a VisionPortal
  * Fixes IllegalStateException when calling `stopStreaming()` immediately after building a VisionPortal
  * Added FTC Blocks counterparts to new Java methods:
    * VisionPortal.Builder.setAutoStartStreamOnBuild
    * VisionPortal.Builder.setShowStatsOverlay
    * AprilTagProcessor.Builder.setSuppressCalibrationWarnings
    * CameraStreamServer.setSource​

### Bug Fixes
* Fixes a problem where OnBotJava does not apply font size settings to the editor.
* Updates EasyOpenCV dependency to v1.7.1
  * Fixes inability to use EasyOpenCV CameraFactory in OnBotJava
  * Fixes entire RC app crash when user pipeline throws an exception
  * Fixes entire RC app crash when user user canvas annotator throws an exception
  * Use the modern stacktrace display when handling user exceptions instead of the legacy ESTOP telemetry message

## Version 9.0.1 (20230929-083754)

### Enhancements
* Updates AprilTag samples to include Decimation and additional Comments.  Also corrects misleading tag ID warnings
* Increases maximum size of Blocks inline comments to 140 characters
* Adds Blocks sample BasicOmniOpMode.
* Updated CENTERSTAGE library AprilTag orientation quaternions
    * Thanks [@FromenActual](https://github.com/FromenActual)
* Updated Java Sample ConceptTensorFlowObjectDetection.java to include missing elements needed for custom model support.

### Bug Fixes
* Fixes a problem where after October 1 the Driver Station will report as obsolete on v9.0 and prompt the user to update.

## Version 9.0 (20230830-154348)

### Breaking Changes
* Removes Vuforia
* Fields in `AprilTagDetection` and `AprilTagPose(ftc/raw)` objects are now `final`
* VisionPortal builder method `setCameraMonitorViewId()` has been renamed to `setLiveViewContainerId()` and `enableCameraMonitoring()` has been renamed to `enableLiveView()`

### Enhancements
* Adds support for the DFRobot HuskyLens Vision Sensor.
* Blocks teams can now perform webcam calibration.
    * Added a Block for System.currentTimeMillis (under Utilities/Time)
    * Added a Block for VisionPortal.saveNextFrameRaw (under Vision/VisionPortal)
    * Added a new sample Blocks OpMode called UtilityCameraFrameCapture.
* The RobotDriveByGyro sample has been updated to use the new universal IMU interface.  It now supports both IMU types.
* Removed some error-prone ElapsedTime Blocks from the Blocks editor's toolbox. This is not a
  breaking change: old Blocks OpModes that use these Blocks will still function, both in the
  Blocks editor and at runtime.
* Standardizes on the form "OpMode" for the term OpMode.
    * The preferred way to refer to OpModes that specifically extend `LinearOpMode` (including Blocks OpModes) is "linear OpMode".
    * The preferred way to refer to OpModes that specifically extend `OpMode` directly is "iterative OpMode".
* Overhauls `OpMode` and `LinearOpMode` Javadoc comments to be easier to read and include more detail.
* Makes minor enhancements to Java samples
    * Javadoc comments in samples that could be rendered badly in Android Studio have been converted to standard multi-line comments
    * Consistency between samples has been improved
    * The SensorDigitalTouch sample has been replaced with a new SensorTouch sample that uses the `TouchSensor` interface instead of `DigitalChannel`.
    * The ConceptCompassCalibration, SensorMRCompass, and SensorMRIRSeeker samples have been deleted, as they are not useful for modern FTC competitions.

### Bug Fixes
* Fixes a bug which prevented PlayStation gamepads from being used in bluetooth mode. Bluetooth is NOT legal for competition but may be useful to allow a DS device to be used while charging, or at an outreach event.
* Fixes a bug where a Blocks OpMode's Date Modified value can change to December 31, 1969, if the Control Hub is rebooted while the Blocks OpMode is being edited.
* Fixes the automatic TeleOp preselection feature (was broken in 8.2)
* Fixes a bug where passing an integer number such as 123 to the Telemetry.addData block that takes a number shows up as 123.0 in the telemetry.
* Fixes OnBotJava autocomplete issues:
  * Autocomplete would incorrectly provide values for the current class when autocompleting a local variable
  * `hardwareMap` autocomplete would incorrectly include lambda class entries
* Fixes OnBotJava not automatically importing classes.
* Fixes OnBotJava tabs failing to close when their file is deleted.
* Fixes a project view refresh not happening when a file is renamed in OnBotJava.
* Fixes the "Download" context menu item for external libraries in the OnBotJava interface.
* Fixes issue where Driver Station telemetry would intermittently freeze when set to Monospace mode.
* Fixes performance regression for certain REV Hub operations that was introduced in version 8.2.
* Fixes TagID comparison logic in DriveToTag samples.
