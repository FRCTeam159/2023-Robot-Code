// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel;

public class Claw extends SubsystemBase {

  // private static MotorController m_clawMotor1;
  // private static MotorController m_clawMotor2;
  public RelativeEncoder m_encoderLeft;
  public RelativeEncoder m_encoderRight;
  public static CANSparkMax m_clawMotor1;
  public static CANSparkMax m_clawMotor2;

  private static DoubleSolenoid m_clawSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  private static DoubleSolenoid m_clawSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);

  /** Creates a new Claw. */
  public Claw() {
    m_clawMotor1 = new CANSparkMax(kClawMotorID1, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_clawMotor2 = new CANSparkMax(kClawMotorID2, CANSparkMaxLowLevel.MotorType.kBrushless); // can't measure
    m_clawMotor1.setInverted(true);
    m_encoderLeft = m_clawMotor1.getEncoder();
    m_encoderRight = m_clawMotor2.getEncoder();
  }

  public void clawMotorState(double state) {
    if(state == 0){
      m_clawMotor1.set(0);
      m_clawMotor2.set(0);
    } else if(state == 1){
      m_clawMotor1.set(-.3); // big suck
      m_clawMotor2.set(-.3);
    } else if(state == 2){
      m_clawMotor1.set(-0.02); //small suck
      m_clawMotor2.set(-0.02);
    } else if(state == 3){
      m_clawMotor1.set(1); //big throw
      m_clawMotor2.set(1);
    } else if(state == 4){
      m_clawMotor1.set(0.25);
      m_clawMotor2.set(0.25);
    }
  }

  public void clawSolenoidState(boolean grab) {
    if (grab) {
      m_clawSolenoid1.set(Value.kForward);
      m_clawSolenoid2.set(Value.kForward);
    } else {
      m_clawSolenoid1.set(Value.kReverse);
      m_clawSolenoid2.set(Value.kReverse);
    }
  }

  public void log() {
    SmartDashboard.putNumber("clawLeft", m_encoderLeft.getPosition());
    SmartDashboard.putNumber("clawRight", m_encoderRight.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //log();
  }
}
