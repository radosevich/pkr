// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Climb Subsystem

    // Drivetrain Subsystem
    public static int kRightLeadID = 1; // CAN value of Right Lead Motor
    public static int kLeftLeadID = 2; // CAN value of Left Lead Motor
    public static int kRightFollowID = 3; // CAN value of Right Follow Motor
    public static int kLeftFollowID = 4; // CAN value of Left Follow Motor
    
    // Intake Subsystem
    public static int kHighIntakeID = 5; // CAN value of kHighIntakeID
    public static int kLowIntakeID = 6; // CAN value of kLowIntakeID
  
    public static final double kIntakeSpeed = -.75; // Update this with your desired intake speed

    // Shooter Subsystem
    public static int kRightShooterID = 7; // CAN value of Right Shooter
    public static int kLeftShooterID = 8; // CAN value of Left Shooter

    public static final double kShooterSpeed = 0.25; // Update this with your desired shooter speed
    
    // Controllers
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    
}