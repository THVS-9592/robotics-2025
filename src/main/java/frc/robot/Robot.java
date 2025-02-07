// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this proj

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import java.util.ArrayList;
import org.opencv.core.*;
import org.opencv.imgproc.*;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends TimedRobot 
{
  // Declare controllers and timer

  private XboxController controller;
  private final Timer m_timer = new Timer(); 

  // Declare motors

  WPI_VictorSPX victorLeftLeader = new WPI_VictorSPX(1);
  WPI_VictorSPX victorLeftFollower = new WPI_VictorSPX(3);

  WPI_VictorSPX victorRightLeader = new WPI_VictorSPX(2);
  WPI_VictorSPX victorRightFollower = new WPI_VictorSPX(4);

  // Declare drivetrain

  DifferentialDrive drivetrain = new DifferentialDrive(victorLeftLeader, victorRightLeader);

  // Set variables to default

  double kP2 = 1;
  double kP = 0.05;
  boolean shoot = false;
  boolean reverse = false;
  boolean low = false;
  boolean restart = false;
  boolean reversed = false;
  boolean shooting = false;
  boolean move = false;
  boolean drive = false;

  // Change the safety settings of all the motors
  public void setSafety(boolean safety)
  {
    victorRightLeader.setSafetyEnabled(safety);
    victorRightFollower.setSafetyEnabled(safety);

    victorLeftLeader.setSafetyEnabled(safety);
    victorLeftFollower.setSafetyEnabled(safety);

    drivetrain.setSafetyEnabled(safety);
  }

  public void robotInit() 
  {
    victorLeftLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    
    controller = new XboxController(0);
    victorLeftFollower.follow(victorLeftLeader);
    victorRightFollower.follow(victorRightLeader);

    // Invert the right side of the drivetrain, and make sure the left is not inverted

    victorLeftLeader.setInverted(false);
    victorRightLeader.setInverted(true);

    // Make the followers the same as their leaders

    victorLeftFollower.setInverted(InvertType.FollowMaster);
    victorRightFollower.setInverted(InvertType.FollowMaster);

    // Safeties shouldn't on, but we will still set them to on at the start of the program

    setSafety(true);
  }

  public void teleopInit()
  {
    setSafety(false);
  }

public void teleopPeriodic()
{
  drivetrain.arcadeDrive(controller.getLeftY(), -controller.getRightX(), true);
}

  public void autonomousInit()
  {
    setSafety(false);
    
    m_timer.restart();

    shooting = false;
    move = false;
  }

  public void autonomousPeriodic()
  {

  }

  void apriltagVisionThreadProc() {
    var detector = new AprilTagDetector();

    // look for tag36h11, correct 3 error bits
    detector.addFamily("tag36h11", 3);

    // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
    // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
    var poseEstConfig =
        new AprilTagPoseEstimator.Config(
            0.1651, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
    var estimator = new AprilTagPoseEstimator(poseEstConfig);

    // Get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture();

    // Set the resolution
    camera.setResolution(640, 480);

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo();

    // Setup a CvSource. This will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("Detected", 640, 480);
    CvSource outputStreamGrey = CameraServer.putVideo("Grey", 640, 480);

    // Mats are very memory expensive. Lets reuse these.
    var mat = new Mat();
    var grayMat = new Mat();

    // Instantiate once
    ArrayList<Long> tags = new ArrayList<>();
    var outlineColor = new Scalar(0, 255, 0);
    var crossColor = new Scalar(0, 0, 255);

    // We'll output to NT
    NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
    IntegerArrayPublisher pubTags = tagsTable.getIntegerArrayTopic("tags").publish();

    // This cannot be 'true'. The program will never exit if it is. This
    // lets the robot stop this thread when restarting robot code or
    // deploying.
    while (!Thread.interrupted()) {

      // Tell the CvSink to grab a frame from the camera and put it
      // in the source mat.  If there is an error notify the output.
      if (cvSink.grabFrame(mat) == 0) {

        // Send the output the error.
        outputStream.notifyError(cvSink.getError());

        // skip the rest of the current iteration
        continue;
      }

      Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

      AprilTagDetection[] detections = detector.detect(grayMat);

      // have not seen any tags yet
      tags.clear();

      for (AprilTagDetection detection : detections) {

        // remember we saw this tag
        tags.add((long) detection.getId());

        // draw lines around the tag
        for (var i = 0; i <= 3; i++) {
          var j = (i + 1) % 4;
          var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
          var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
          Imgproc.line(mat, pt1, pt2, outlineColor, 2);
        }

        // mark the center of the tag
        var cx = detection.getCenterX();
        var cy = detection.getCenterY();
        var ll = 10;
        Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), crossColor, 2);
        Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), crossColor, 2);

        // identify the tag
        Imgproc.putText(
            mat,
            Integer.toString(detection.getId()),
            new Point(cx + ll, cy),
            Imgproc.FONT_HERSHEY_SIMPLEX,
            1,
            crossColor,
            3);

        // determine pose
        Transform3d pose = estimator.estimate(detection);

        // put pose into dashboard
        Rotation3d rot = pose.getRotation();
        tagsTable
            .getEntry("pose_" + detection.getId())
            .setDoubleArray(
                new double[] {
                  pose.getX(), pose.getY(), pose.getZ(), rot.getX(), rot.getY(), rot.getZ()
                });
      }

      // put list of tags onto dashboard
      pubTags.set(tags.stream().mapToLong(Long::longValue).toArray());

      // Give the output stream a new image to display
      outputStream.putFrame(mat);
      outputStreamGrey.putFrame(grayMat);
    }

    pubTags.close();
    detector.close();
  }
}