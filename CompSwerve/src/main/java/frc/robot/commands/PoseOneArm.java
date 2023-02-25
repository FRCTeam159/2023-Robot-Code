// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.OneArm;

public class PoseOneArm extends CommandBase {
  public OneArm m_Arm;
  public XboxController m_Controller;
  public Claw m_Claw;
  public enum m{
    pickup,
    hold,
    drop,
    eject,
    none
  };
  public enum a{
    arm,
    wrist
  };
  public m claw_mode = m.none;
  public a arm_mode = a.arm;

  public Timer tim = new Timer();
  public double k = 0; //set to 113;
  /** Creates a new PoseArm. */
  public PoseOneArm(OneArm arm, XboxController controller, Claw claw) {
    m_Arm = arm;
    m_Controller = controller;
    m_Claw = claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Arm.posHolding();
    tim.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Controller.getYButtonPressed()){
      if(arm_mode==a.arm)
        arm_mode=a.wrist;
      else if(arm_mode==a.wrist)
        arm_mode=a.arm;
    }
    double up = m_Controller.getRightTriggerAxis();
    double down = m_Controller.getLeftTriggerAxis();
    if(arm_mode==a.arm){
      if(up>0)
        m_Arm.setArm(0.01*up);
      if(down>0)
        m_Arm.setArm(-0.01*down);
    }
    else{
      if(up>0)
        m_Arm.setWrist(0.01*up);
      if(down>0)
        m_Arm.setWrist(-0.01*down);
    }
    wristTest();
  }

  public void wristTest() {
    if (m_Controller.getBButtonPressed()) {
      System.out.print("B button");
      if (claw_mode == m.none || claw_mode == m.eject) {
        m_Claw.clawSolenoidState(false);
        claw_mode = m.pickup;
      } else if (claw_mode == m.pickup) {
        m_Claw.clawSolenoidState(true);
        claw_mode = m.hold;
      } else if (claw_mode == m.hold) {
        m_Claw.clawSolenoidState(false);
        claw_mode = m.drop;
        tim.reset();
      }
    }

    if(m_Controller.getXButtonPressed()){
      m_Claw.clawSolenoidState(false);
      claw_mode = m.eject;
      tim.reset();
    }

    switch(claw_mode){
      case none:
        m_Claw.clawMotorState(0);
        break;
      case pickup:
        m_Claw.clawMotorState(1);
        break;
      case hold:
        m_Claw.clawMotorState(2);
        break;
      case drop:
        m_Claw.clawMotorState(0);
        break;
      case eject:
        m_Claw.clawMotorState(3);
        break;
    }

    if(claw_mode == m.drop && tim.get() > 3){
      claw_mode = m.none;
      m_Claw.clawSolenoidState(true);
    }

    if(claw_mode == m.eject && tim.get() > 0.5){
      claw_mode = m.none;
      m_Claw.clawSolenoidState(true);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
