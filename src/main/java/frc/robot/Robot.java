// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this proj

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

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
}