// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this proj

package frc.robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;

import org.opencv.core.*;
import org.opencv.imgproc.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Robot extends TimedRobot 
{
  // Declare controllers and timer
  private XboxController driver;
  private XboxController operator;

  private final Timer m_timer = new Timer(); 

  DigitalInput limit = new DigitalInput(0);

  // Declare motors
  WPI_TalonSRX talonLeftLeader = new WPI_TalonSRX(1);
  WPI_TalonSRX talonLeftFollower = new WPI_TalonSRX(3);

  WPI_TalonSRX talonRightLeader = new WPI_TalonSRX(2);
  WPI_TalonSRX talonRightFollower = new WPI_TalonSRX(4);

  WPI_VictorSPX victorRollerElevator = new WPI_VictorSPX(0);
  WPI_TalonSRX talonRollers = new WPI_TalonSRX(6);
  SparkMax sparkElevator = new SparkMax(7, MotorType.kBrushless);
  WPI_TalonSRX talonCoralIntake = new WPI_TalonSRX(8);

  SparkClosedLoopController closedLoopController = sparkElevator.getClosedLoopController();

  // Declare encoder
  RelativeEncoder sparkEncoder = sparkElevator.getEncoder();
  // Declare drivetrain
  DifferentialDrive drivetrain = new DifferentialDrive(talonLeftLeader, talonRightLeader);

  //Set variables
  boolean rollerDown = false;
  boolean elevatorLowered = false;
  int setpoint;
  boolean go = true;

  UsbCamera camera = CameraServer.startAutomaticCapture();

  PIDController pid = new PIDController(0.5, 0, 0);

  // Change the safety settings of all the motors
  public void setSafety(boolean safety)
  {
    talonRightLeader.setSafetyEnabled(safety);
    talonRightFollower.setSafetyEnabled(safety);

    talonLeftLeader.setSafetyEnabled(safety);
    talonLeftFollower.setSafetyEnabled(safety);

    drivetrain.setSafetyEnabled(safety);
  }

  public void robotInit() 
  {    
    driver = new XboxController(0);
    operator = new XboxController(1);

    talonLeftFollower.follow(talonLeftLeader);
    talonRightFollower.follow(talonRightLeader);

    // Invert the right side of the drivetrain, and make sure the right is not inverted
    talonLeftLeader.setInverted(false);
    talonRightLeader.setInverted(true);

    talonLeftFollower.setInverted(talonLeftLeader.getInverted());
    talonRightFollower.setInverted(talonRightLeader.getInverted());
    
    // Safeties shouldn't on, but we will still set them to on at the start of the program
    setSafety(true);

    //Configure the encoders
    //talonLeftLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    //talonLeftLeader.setSensorPhase(false);

    //talonRightLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    //talonRightLeader.setSensorPhase(true);

    SparkMaxConfig encoderConfig = new SparkMaxConfig();
    encoderConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    encoderConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .p(0.4)
      .i(0)
      .d(0)
      .outputRange(-1, 1)
      // Set PID values for velocity control in slot 1
      .p(0.0001, ClosedLoopSlot.kSlot1)
      .i(0, ClosedLoopSlot.kSlot1)
      .d(0, ClosedLoopSlot.kSlot1)
      .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
      .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    encoderConfig.closedLoop.maxMotion
      // Set MAXMotion parameters for position control. We don't need to pass
      // a closed loop slot, as it will default to slot 0.
      .maxVelocity(1000)
      .maxAcceleration(1000)
      .allowedClosedLoopError(1)
      // Set MAXMotion parameters for velocity control in slot 1
      .maxAcceleration(500, ClosedLoopSlot.kSlot1)
      .maxVelocity(6000, ClosedLoopSlot.kSlot1)
      .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    sparkElevator.configure(encoderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultNumber("Calculate", 0);
    SmartDashboard.setDefaultNumber("LLeaderSpeed", 0);
    SmartDashboard.setDefaultNumber("RLeaderSpeed", 0);
    SmartDashboard.setDefaultNumber("LFollowSpeed", 0);
    SmartDashboard.setDefaultNumber("RFollowSpeed", 0);
    SmartDashboard.setDefaultNumber("Robot Position L", 0);
    SmartDashboard.setDefaultNumber("Robot Position R", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
  }

  //Move the rollers to take a ball
  public void turnRollers(boolean out)
  {
    if(out == false)
    {
      if(operator.getAButton())
      {
        talonRollers.set(-1);
      }
      else
      {
        talonRollers.set(0);
      }
    }
    else
    {
      if(operator.getBButton())
      {
        talonRollers.set(1);
      }
      else
      {
        talonRollers.set(0);
      }
    }
  }

  public void shootCoral(boolean intake)
  {
    talonCoralIntake.set(1);
    talonCoralIntake.set(0);
  }

  public void moveBottom()
  {
    setpoint = 54;
    while(pid.calculate(sparkEncoder.getPosition(), setpoint) > 2 || pid.calculate(sparkEncoder.getPosition(), setpoint) < -2)
    {
      sparkElevator.set(pid.calculate(sparkEncoder.getPosition(), setpoint));
    }
    setpoint = 0;
  }

  public void moveMiddle()
  {
    setpoint = 110;
    if(pid.calculate(sparkEncoder.getPosition(), setpoint) > 2 || pid.calculate(sparkEncoder.getPosition(), setpoint) < -2)
    {
      sparkElevator.set(pid.calculate(sparkEncoder.getPosition(), setpoint));
    }
    else
    {
      sparkElevator.set(0);
    }
    setpoint = 0;
  }

  public void robotPeriodic() {
    // Display encoder position and velocity
    SmartDashboard.putNumber("Actual Position", sparkEncoder.getPosition());
    SmartDashboard.putNumber("Actual Velocity", sparkEncoder.getVelocity());
    SmartDashboard.putNumber("Calculate", pid.calculate(sparkEncoder.getPosition(), setpoint));
    SmartDashboard.putNumber("Robot Position L", talonLeftLeader.getSelectedSensorPosition());
    SmartDashboard.putNumber("Robot Position R", talonRightLeader.getSelectedSensorPosition());

    if (SmartDashboard.getBoolean("Reset Encoder", false))
    {
      SmartDashboard.putBoolean("Reset Encoder", false);
      // Reset the encoder position to 0
      sparkEncoder.setPosition(0);
    }

    if(limit.get())
    {
      sparkEncoder.setPosition(0);
    }
  }

  public void teleopInit()
  {
    CvSink cvSink = CameraServer.getVideo();
    CvSource outputStream = CameraServer.putVideo("Detected", 640, 480);
    setpoint = 0;
    m_timer.start();
    setSafety(false);
  }

  public void teleopPeriodic()
  {
    drivetrain.arcadeDrive(-driver.getLeftY(), -driver.getRightX(), true);
    if(operator.getLeftTriggerAxis() > 0)
    {
      victorRollerElevator.set(operator.getLeftTriggerAxis());
    }
    else if(operator.getRightTriggerAxis() > 0)
    {
      victorRollerElevator.set(-operator.getRightTriggerAxis());
    }
    sparkElevator.set(-operator.getLeftY());

    if(operator.getRightBumperButton())
    {
      talonRollers.set(1);
    }
    else if(!operator.getLeftBumperButton())
    {
      talonRollers.set(0);
    } 
    
    if(operator.getLeftBumperButton())
    {
      talonRollers.set(-1);
    }
    else if(!operator.getRightBumperButton())
    {
      talonRollers.set(0);
    }

    if(operator.getXButton())
    {
      talonCoralIntake.set(0.8);
    }
    else if(operator.getYButton())
    {
      talonCoralIntake.set(0.3);
    }
    else
    {
      talonCoralIntake.set(0);
    }
    
    
    if(operator.getPOV() == 0)
    {
      moveBottom();
    }

    if(operator.getPOV() == 90)
    {
      moveMiddle();
    }

    SmartDashboard.putNumber("LLeaderSpeed", talonLeftLeader.getMotorOutputPercent());
    SmartDashboard.putNumber("RLeaderSpeed", talonLeftLeader.getMotorOutputPercent());
    SmartDashboard.putNumber("LFollowSpeed", talonLeftLeader.getMotorOutputPercent());
    SmartDashboard.putNumber("RFollowSpeed", talonLeftLeader.getMotorOutputPercent());
  }

  public void autonomousInit()
  {
    go = true;
    setpoint = 0;
    setSafety(false);
    m_timer.restart();
  }

  public void autonomousPeriodic()
  {
    if(m_timer.get() < 5 && go == true)
    {
      talonRightLeader.set(0.2);
      talonLeftLeader.set(0.2);
    }
    else
    {
      talonRightLeader.set(0);
      talonLeftLeader.set(0);
      if(go == true)
      {
        setpoint = 110;
        if(pid.calculate(sparkEncoder.getPosition(), setpoint) > 2 || pid.calculate(sparkEncoder.getPosition(), setpoint) < -2)
        {
          sparkElevator.set(pid.calculate(sparkEncoder.getPosition(), setpoint));
        }
        else
        {
          sparkElevator.set(0);
          setpoint = 0;
          m_timer.restart();
          go = false;
        }
      }
      
      if(m_timer.get() < 2)
      {
        sparkElevator.set(0);
        talonCoralIntake.set(0.8);
      }
      else
      {
        talonCoralIntake.set(0);
      }
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