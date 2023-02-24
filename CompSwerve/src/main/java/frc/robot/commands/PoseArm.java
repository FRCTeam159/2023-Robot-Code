// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.RunnableFuture;

import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class PoseArm extends CommandBase {
  public Arm m_Arm;
  public XboxController m_Controller;
  public Claw m_Claw;
  public boolean m_ClawIn = false;
  /** Creates a new PoseArm. */
  public PoseArm(Arm arm, XboxController controller, Claw claw) {
    m_Arm = arm;
    m_Controller = controller;
    m_Claw = claw;
    addRequirements(arm, claw);
    //SmartDashboard.putNumber("joystick", -2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Arm.posHolding();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double up = m_Controller.getRightTriggerAxis();
    double down = m_Controller.getLeftTriggerAxis();
    //double wristmotorspin = 
    wristTest();

    
    //m_Arm.stageOne.set((up-down)*0.001);
    //System.out.println(up-down);
    //SmartDashboard.putNumber("joystick", m_Controller.getRightY());
    // int direction = m_Controller.getPOV(1);
    // m_Arm.posTrim(Units.degreesToRadians(direction));
    // m_Arm.runFeed();
  }

  public void wristTest() {
    System.out.print(".");
    if (m_Controller.getBButtonPressed()) {
      System.out.print("B button");
      if (m_ClawIn){
        //m_Claw.clawSolenoidState(false);
        m_ClawIn = false;
        System.out.println("opening claw");
      }
      else{
       // m_Claw.clawSolenoidState(true);
        m_ClawIn = true;
        System.out.println("closing claw");
      }


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
