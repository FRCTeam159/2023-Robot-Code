// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.RunnableFuture;

import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class PoseArm extends CommandBase {
  public Arm m_Arm;
  public XboxController m_Controller;
  public Claw m_Claw;
  public m mode = m.none;
  public enum m{
    pickup,
    hold,
    drop,
    eject,
    none
  };
  public Timer tim = new Timer();
  public double k = 0; //set to 113;
  /** Creates a new PoseArm. */
  public PoseArm(Arm arm, XboxController controller, Claw claw) {
    m_Arm = arm;
    m_Controller = controller;
    m_Claw = claw;
    //addRequirements(arm, claw);
    //SmartDashboard.putNumber("joystick", -2);
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
    double up = m_Controller.getRightTriggerAxis();
    double down = m_Controller.getLeftTriggerAxis();
    if((m_Arm.encoderOne.getPosition()-k)<0){
      m_Arm.stageOne.set(up - down);
    } else if((m_Arm.encoderOne.getPosition()-k)>0){
      m_Arm.stageOne.set(up);
    }

    double joy = m_Controller.getRightY();
    m_Arm.wrist.set(joy/2);

    m_Arm.log(); 
    wristTest();

    
    //m_Arm.stageOne.set((up-down)*0.001);
    //System.out.println(up-down);
    //SmartDashboard.putNumber("joystick", m_Controller.getRightY());
    // int direction = m_Controller.getPOV(1);
    // m_Arm.posTrim(Units.degreesToRadians(direction));
    // m_Arm.runFeed();
  }

  public void wristTest() {
    if (m_Controller.getBButtonPressed()) {
      System.out.print("B button");
      if (mode == m.none || mode == m.eject){
        m_Claw.clawSolenoidState(false);
        mode = m.pickup;
      }else if(mode == m.pickup){
        m_Claw.clawSolenoidState(true);
        mode = m.hold;
      }else if (mode == m.hold){
        m_Claw.clawSolenoidState(false);
        mode = m.drop;
        tim.reset();
    }
    }

    if(m_Controller.getXButtonPressed()){
      m_Claw.clawSolenoidState(false);
      mode = m.eject;
      tim.reset();
    }

    switch(mode){
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

    if(mode == m.drop && tim.get() > 3){
      mode = m.none;
      m_Claw.clawSolenoidState(true);
    }

    if(mode == m.eject && tim.get() > 0.5){
      mode = m.none;
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
