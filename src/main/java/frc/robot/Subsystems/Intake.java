package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
  CANSparkMax m_highintake;
  CANSparkMax m_lowintake;

  public Intake() {
  /** Creates a new Intake. */
  System.out.println("Intake()");
    m_highintake = new CANSparkMax(Constants.kHighIntakeID, MotorType.kBrushless);
    m_lowintake = new CANSparkMax(Constants.kLowIntakeID, MotorType.kBrushless);
  }
  
  public void IntakeRun(double speed) {
  //start motors to intake or eject note depending on sign of speed
    m_highintake.set(-speed);
    m_lowintake.set(speed);
  }

  public void IntakeStop() {
    m_highintake.set(0);
    m_lowintake.set(0);
  }
}
   
