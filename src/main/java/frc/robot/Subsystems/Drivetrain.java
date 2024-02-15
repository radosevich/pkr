package frc.robot.Subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Drivetrain extends SubsystemBase {

  // Motors
  private final CANSparkMax m_rightLead = 
    new CANSparkMax(Constants.kRightLeadID, MotorType.kBrushless);
  private final CANSparkMax m_leftLead = 
    new CANSparkMax(Constants.kLeftLeadID, MotorType.kBrushless);
  private final CANSparkMax m_rightFollow = 
    new CANSparkMax(Constants.kRightFollowID, MotorType.kBrushless);
  private final CANSparkMax m_leftFollow = 
    new CANSparkMax(Constants.kLeftFollowID, MotorType.kBrushless);
  
  // Differential Drive
  private final DifferentialDrive m_robotDrive = 
    new DifferentialDrive(m_rightLead, m_leftLead);

  // Kinematics and odometry

  public Drivetrain() {
  //Creates a new drivetrain
    m_leftFollow.follow(m_leftLead);
    m_rightFollow.follow(m_rightLead);
  }

  public void manualDrive(double move, double turn) {
    if (Math.abs(move) < 0.1) move = 0;
    if (Math.abs(turn) < 0.1) turn = 0;

    m_robotDrive.arcadeDrive(move, turn);
  } 
}