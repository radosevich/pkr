package frc.robot.Subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase {

  CANSparkMax m_highintake;
  CANSparkMax m_lowintake;

  public Intake() {
  /** Creates a new Intake. */
    m_highintake = new CANSparkMax(Constants.kHighIntakeID, MotorType.kBrushless);
    m_lowintake = new CANSparkMax(Constants.kLowIntakeID, MotorType.kBrushless);
  }
  
  public void IntakeIn(double speed) {
    m_highintake.set(-speed);
    m_lowintake.set(speed);
  }

  public void IntakeOut(double speed) {
    m_highintake.set(speed);
    m_lowintake.set(-speed);
  }
  
  public void IntakeStop() {
  m_highintake.set(0);
  m_lowintake.set(0);
  }
}
   
