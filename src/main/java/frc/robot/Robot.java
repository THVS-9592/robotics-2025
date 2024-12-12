// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this proj

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;

import java.util.ArrayList;
import java.util.Optional;

import org.opencv.core.*;
import org.opencv.imgproc.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.kauailabs.navx.frc.*;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends TimedRobot 
{
  // Declare controllers and timer

  private XboxController controller;
  private XboxController operator;
  private final Timer m_timer = new Timer(); 

  // Declare motors

  WPI_TalonSRX talonLeftLeader = new WPI_TalonSRX(1);
  WPI_TalonSRX talonLeftFollower = new WPI_TalonSRX(3);

  WPI_TalonSRX talonRightLeader = new WPI_TalonSRX(2);
  WPI_TalonSRX talonRightFollower = new WPI_TalonSRX(4);

  WPI_TalonSRX talonTopWheel = new WPI_TalonSRX(5);
  WPI_TalonSRX talonBottomWheel = new WPI_TalonSRX(6);


   DoubleSolenoid firstSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
   //Solenoid secondSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 9);
   Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  // Declare drivetrain

  DifferentialDrive drivetrain = new DifferentialDrive(talonLeftLeader, talonRightLeader);

  // Declare gyroscope

  AHRS gyroscope = new AHRS(SPI.Port.kMXP);

  // Set variables to default, these should probably be booleans

  double kP2 = 1;
  double kP = 0.05;
  int shoot = 0;
  int reverse = 0;
  int low = 0;
  int restart = 0;
  int reversed = 0;
  int shooting = 0;
  int move = 0;
  int drive = 0;

  public void robotInit() 
  {
    // Start Apriltag detection

    var visionThread = new Thread(this::apriltagVisionThreadProc);
    visionThread.setDaemon(true);
    visionThread.start();

    talonLeftLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    m_compressor.enableDigital();

    // Add a gyroscope to the Shuffleboard

    Shuffleboard.getTab("Gyro").add(gyroscope);
    
    controller = new XboxController(0);
    operator = new XboxController(1);
    talonLeftFollower.follow(talonLeftLeader);
    talonRightFollower.follow(talonRightLeader);

    // Invert the right side of the drivetrain, and make sure the left is not inverted

    talonLeftLeader.setInverted(false);
    talonRightLeader.setInverted(true);

    // Make the followers the same as their leaders

    talonLeftFollower.setInverted(InvertType.FollowMaster);
    talonRightFollower.setInverted(InvertType.FollowMaster);

    // Safeties are never on, but still set them to on at the start of the program

    talonRightLeader.setSafetyEnabled(true);
    talonRightFollower.setSafetyEnabled(true);

    talonLeftLeader.setSafetyEnabled(true);
    talonLeftFollower.setSafetyEnabled(true);

    //drivetrain.setDeadband(0.03);

    //robot too fast for turning!! see what drivers think
    //drivetrain.setMaxOutput(0.7);
  }

  public void teleopInit()
  {

    // Safeties make the robot yell in pain, turn them off

    talonRightLeader.setSafetyEnabled(false);
    talonRightFollower.setSafetyEnabled(false);

    talonLeftLeader.setSafetyEnabled(false);
    talonLeftFollower.setSafetyEnabled(false);

    drivetrain.setSafetyEnabled(false);

    // If the robot is facing the opposite direction from
    // where it woke up, set the "reversed" variable to 1
    // so turning is reversed

/*  if(gyroscope.getYaw() < -90 || gyroscope.getAngle() > 90)
    {
      reversed = 1;
      System.out.println("reversed");
    }
    else
    {
      reversed = 0;
      System.out.println("not reversed");
     } */

    m_timer.restart();

    // Make sure variables are set to default

    shoot = 0;
    reverse = 0;
    low = 0;
    restart = 0;
    drive = 0;

    firstSolenoid.set(Value.kReverse);

  }

  public void teleopPeriodic()
  {
    // Shoot with full power upon an A press

    if(operator.getAButton() || shoot == 1)
    {
      if (restart == 0)
      {
        m_timer.restart();
        restart = 1;
      }
      else
      {
        //System.out.println(m_timer.get());
      }

      //System.out.println("a pressed");
      //System.out.println(talonBottomWheel.getMotorOutputPercent());
      shoot = 1;
      if(m_timer.get() <= 0.5)
      {
        talonTopWheel.set(1);
        talonBottomWheel.set(0);

        // current fix works, for now! use this if it stops working

        //try {
        //  System.out.println("waiting");
        //  wait(500);
        //} catch (InterruptedException e) {
        //  System.out.print("interrupted ");
        //  e.printStackTrace();
        //  System.out.println();
        //}
      }
      else if(m_timer.get() <= 1 && m_timer.get() > 0.5)
      {
        talonTopWheel.set(1);
        talonBottomWheel.set(1);
      }
      else
      {
        m_timer.restart();
        shoot = 0;
        restart = 0;
        talonBottomWheel.set(0);
        talonTopWheel.set(0);
      }
    }

    // Suck in the note for shooting upon a B press

    if(operator.getBButtonPressed() || reverse == 1)
    {
      if (restart == 0)
      {
        m_timer.restart();
        restart = 1;
      }
      else
      {
        //System.out.println(m_timer.get());
      }

      //System.out.println("b pressed");
      //System.out.println(talonBottomWheel.getMotorOutputPercent());
      reverse = 1;
      if(m_timer.get() <= 1)
      {
        talonBottomWheel.set(-1);
        talonTopWheel.set(-1);
      }
      else
      {
        m_timer.restart();
        reverse = 0;
        restart = 0;
        talonBottomWheel.set(0);
        talonTopWheel.set(0);
      }
    }

    // Shoot the note at a lower power upon an X press

    if(operator.getXButtonPressed() || low == 1)
    {
      if (restart == 0)
      {
        m_timer.restart();
        restart = 1;
      }

      low = 1;
      if(m_timer.get() <= 3)
      {
        talonTopWheel.set(0.36);
        talonBottomWheel.set(0.36);
      }
      else
      {
        m_timer.restart();
        low = 0;
        restart = 0;
        talonTopWheel.set(0);
        talonBottomWheel.set(0);
      }
    }

    // Check if Y button on the operator controller is pressed
if (operator.getYButtonPressed()) {
  // Toggle the compressor state
  //if (m_compressor.isEnabled()) {
  //    m_compressor.disable();
  //    System.out.println("Compressor turned off");
  //} else {
      //m_compressor.enableDigital();
      //System.out.println("Compressor turned on");
      //System.out.println(m_compressor.isEnabled());
  //}

  // Wait for button release to avoid repeated toggling
  //while (operator.getYButtonPressed()) {
    //System.out.println("waiting");
      // Wait until the button is released
  //}
}

if(operator.getLeftStickButtonPressed()) {
  firstSolenoid.toggle();
}
   

    // Reverse steering if robot is facing opposite direction
    // from where it was turned on

    //if(reversed == 0)
    //{
      drivetrain.arcadeDrive(controller.getLeftY(), -controller.getRightX(), true);
    //}
    //else
    //{
    //  drivetrain.arcadeDrive(controller.getLeftY(), controller.getRightX(), true);
    //}
}

  public void autonomousInit()
  {
    // Safeties make the robot yell in pain, turn them off

    talonRightLeader.setSafetyEnabled(false);
    talonRightFollower.setSafetyEnabled(false);

    talonLeftLeader.setSafetyEnabled(false);
    talonLeftFollower.setSafetyEnabled(false);

    drivetrain.setSafetyEnabled(false);
    
    m_timer.restart();

    shooting = 0;
    move = 0;
  }

  public void autonomousPeriodic()
  {
    Optional<Alliance> ally = DriverStation.getAlliance();

    if(shooting == 0)
    {
     /*  if(m_timer.get() < 1 && move == 0)
      {
        drivetrain.tankDrive(kP, -kP);
      }
      else if(move != 1)
      {
        move = 1;
        m_timer.restart();
      } */
      if(m_timer.get() <= 1/* && move == 1*/)
      {
        //talonTopWheel.set(1);
        //talonBottomWheel.set(0);
      }
      else if(m_timer.get() <= 3 && m_timer.get() > 1)
      {
        //talonTopWheel.set(1);
        //talonBottomWheel.set(1);
      }
      else
      {
        m_timer.restart();
        shooting = 1;
        //talonBottomWheel.set(0);
        //talonTopWheel.set(0);
      }
    }
    else
    {
      //if(move == 1)
      //{
        if(m_timer.get() < 2)
        {
          // move backwards

          drivetrain.arcadeDrive(0.8, 0);
        }
        else if(m_timer.get() < 4)
        {
          // turn

          drivetrain.arcadeDrive(0, -0.5);
        }
        else if(m_timer.get() < 7 && m_timer.get() > 4)
        {
          // move backwards

          drivetrain.arcadeDrive(0.8, 0);
        }
        else{
          // stop

          drivetrain.arcadeDrive(0, 0);
        }
      /*}
      else
      {
        if(m_timer.get() < 0.5)
        {
          drivetrain.tankDrive(kP, -kP);
        }
        else
        {
          move = 1;
          m_timer.restart();
        }
      } 

      drivetrain.tankDrive(kP, -kP);
      
      if(m_timer.get() > 4)
        // Find the heading error; setpoint is 45
        if(ally.get() == Alliance.Red)
        {
          double error = -30 - gyroscope.getAngle();

          // Turns the robot to face the desired direction
          drivetrain.tankDrive(.5 + kP * error, .5 - kP * error);
        }
        else if(ally.get() == Alliance.Red)
        {
          double error = 30 - gyroscope.getAngle();

          
          drivetrain.tankDrive(.5 + kP * error, .5 - kP * error);
        }
        else
        {
          double error = 30 - gyroscope.getAngle();

          drivetrain.tankDrive(.5 + kP * error, .5 - kP * error);
        }
        if(m_timer.get() < 7)
        {
        drivetrain.tankDrive(kP, kP);
        }
        else if(m_timer.get() < 10)
        {
          double error = 90 - gyroscope.getAngle();

          drivetrain.tankDrive(-.5 - kP * error, .5 + kP * error);
        }
      }*/
    }
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
    
    // UsbCamera camera = CameraServer.startAutomaticCapture();

    // Set the resolution

    // camera.setResolution(640, 480);

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