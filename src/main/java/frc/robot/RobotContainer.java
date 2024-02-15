// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Commands.AutoDoNothing;
import frc.robot.Commands.LaunchNoteAuto;
import frc.robot.Commands.LaunchNoteTeleop;

import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class RobotContainer {
  // The robot's subsystems
  public final Drivetrain m_driveSubsystem = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Climb m_climb = new Climb();

  // The robot's controllers
  private final CommandXboxController m_driverController = 
    new CommandXboxController(Constants.kDriverControllerPort);
  private final Joystick m_operatorController = 
    new Joystick(Constants.kOperatorControllerPort);
  private final JoystickButton m_triggerButton =
    // 1 is usually the trigger 
    new JoystickButton(m_operatorController, 1);

  // The robot's autonomous commands
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser;

  // Representation of game field
  public final Field2d m_field = new Field2d();

  public RobotContainer() {
    configureBindings();

    // Teleop default command
    // Control the drive with split-stick arcade controls
    m_driveSubsystem.setDefaultCommand(
      new RunCommand(() -> m_driveSubsystem.manualDrive(
        -m_driverController.getLeftY(), 
        -m_driverController.getRightX()), 
        m_driveSubsystem));
    
    // Setup autonomous select commands
    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("Do nothing", new AutoDoNothing());
    m_chooser.addOption("LaunchNoteAuto", new LaunchNoteAuto(m_shooter, m_intake));
    //m_chooser.addOption("Autonomous Distance", new AutonomousDistance(m_drivetrain, m_Tebo));
    SmartDashboard.putData(m_chooser);

    if (RobotBase.isReal()) {
      System.out.println("Running in a real environment");
    } else {
      System.out.println("Running in a simulated environment");
      SmartDashboard.putData("Field", m_field);
    }
  }

  private void configureBindings() {
  // Configure the button bindings

    m_driverController.rightBumper()
    // Intake note into the robot
      .whileTrue(new RunCommand(() -> m_intake.IntakeIn(Constants.kIntakeSpeed)))
      .onFalse(new RunCommand(() -> m_intake.IntakeStop()));

    m_driverController.leftBumper()
    // Reverse the intake direction
      .whileTrue(new RunCommand(() -> m_intake.IntakeIn(-Constants.kIntakeSpeed)))
      .onFalse(new RunCommand(() -> m_intake.IntakeStop()));

    m_driverController.a()
    // Launch the note
      .whileTrue(new RunCommand(() -> m_shooter.ShooterStart(Constants.kShooterSpeed)))
      .onFalse(new RunCommand(() -> m_shooter.ShooterStop()));

    m_driverController.b()
    // Reverse the shooter in case of a jam
      .whileTrue(new RunCommand(() -> m_shooter.ShooterBack(-Constants.kShooterSpeed)))
      .onFalse(new RunCommand(() -> m_shooter.ShooterStop()));

    // Bind the operator joystick trigger button to the shoot command
    m_triggerButton
      .onTrue(new LaunchNoteTeleop(m_shooter, m_intake));
  }

  public Command getAutonomousCommand() {
    // The selected chooser command will run in autonomous
    m_autonomousCommand = m_chooser.getSelected();
    return m_autonomousCommand;
  }
}
