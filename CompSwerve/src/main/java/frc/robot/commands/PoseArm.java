// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.security.KeyStore;
import java.util.concurrent.RunnableFuture;

import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BNO055;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import static frc.robot.Constants.*;


public class PoseArm extends CommandBase {

  public Arm m_Arm;
  public XboxController m_Controller;
  public Claw m_Claw;
  double armController2;
  double armController1;

  double stageOneOffset;
  
  public boolean lastStageOneForwardLimitState = true;
  /** Creates a new PoseArm. */
  public PoseArm(Arm arm, XboxController controller, Claw claw) {
    m_Arm = arm;
    m_Controller = controller;
    m_Claw = claw;
    armController2 = 0;
    armController1 = 0;
    //addRequirements(arm, claw);
    //SmartDashboard.putNumber("joystick", -2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armController1 = m_Arm.getStageOneAngle();
    m_Arm.posHolding();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double up = m_Controller.getRightTriggerAxis();
    double down = m_Controller.getLeftTriggerAxis();
    //m_Arm.stageOne.set(up - down);
    //m_Arm.stageTwo.set(m_Controller.getLeftX());
    //System.out.println(m_Controller.getLeftX());

    if(m_Controller.getPOV() == 0){
      armController1 = armController1 + 0.01;
    }
    if(m_Controller.getPOV() == 180){
      armController1 = armController1 - 0.01;
    }

    if(m_Controller.getPOV() == 90){
      armController2= armController2 + 0.01;
    }
    if(m_Controller.getPOV() == 270){
      armController2= armController2 - 0.01;
    }
    
    m_Arm.armPIDtest(armController1);
    m_Arm.armPIDtesttwo(armController2);

    m_Arm.log(); 
    m_Claw.clawControl();
    if(m_Controller.getYButtonPressed()){
      m_Arm.posSetpoint1();
      //System.out.println("y pressed");
    }
    if(m_Controller.getBButtonPressed()){
      m_Arm.posHolding();
      System.out.println("b pressed");
    }
    
  }


  public void setArmOffsets(BNO055 gyro){
    double theta = Math.atan(gyro.getVector()[2]/gyro.getVector()[0]);
    stageOneOffset = -theta/(2*Math.PI);
    System.out.println("fancy thing: " + theta/(2*Math.PI));
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
