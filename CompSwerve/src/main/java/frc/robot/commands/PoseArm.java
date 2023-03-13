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
  double stageTwoOffset;
  double wristOffset;
  
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
    SmartDashboard.putString("offsets", "default");
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
    // double up = m_Controller.getRightTriggerAxis();
    // double down = m_Controller.getLeftTriggerAxis();
    //m_Arm.stageOne.set(up - down);
    //m_Arm.stageTwo.set(m_Controller.getLeftX());
    //System.out.println(m_Controller.getLeftX());

    // if(m_Controller.getPOV() == 0){
    //   armController1 = armController1 + 0.01;
    // }
    // if(m_Controller.getPOV() == 180){
    //   armController1 = armController1 - 0.01;
    // }

    // if(m_Controller.getPOV() == 90){
    //   armController2= armController2 + 0.01;
    // }
    // if(m_Controller.getPOV() == 270){
    //   armController2= armController2 - 0.01;
    // }
    
    //m_Arm.armPIDtest(armController1);
    //m_Arm.armPIDtesttwo(armController2);

    m_Arm.log(); 
    m_Claw.clawControl();
    if(m_Controller.getYButtonPressed()){
      m_Arm.posPickupGround();
      System.out.println("y pressed");
    }
    if(m_Controller.getBButtonPressed()){
      m_Arm.posHolding();
      System.out.println("b pressed");
    }
    if(m_Controller.getXButtonPressed()){
      m_Arm.posDropHighCone();
      System.out.println("x pressed");
    }
    if(m_Controller.getAButtonPressed()){
      //m_Arm.posAuto();
      m_Arm.posDropHighCube();
      System.out.println("a pressed");
    }
    
    SmartDashboard.putString("offsets", "two: " + stageTwoOffset + " wrist: " + wristOffset);
  }


  public void setArmOffsets(BNO055 elbow, BNO055 wrist){
    double theta = Math.atan(elbow.getVector()[1]/elbow.getVector()[0]);
    System.out.println("y: " + elbow.getVector()[1]+"x: " + elbow.getVector()[0]);
    stageTwoOffset = theta/(2*Math.PI);
    m_Arm.offset1 = stageTwoOffset;
    System.out.println("two offset: " + theta/(2*Math.PI));
    double tau = Math.atan(wrist.getVector()[2]/wrist.getVector()[0]);
    System.out.println("z: " + wrist.getVector()[1]+"x: " + wrist.getVector()[0]);
    wristOffset = tau/(2*Math.PI);
    m_Arm.offsetW = wristOffset;
    System.out.println("wrist offset: " + tau/(2*Math.PI));
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
