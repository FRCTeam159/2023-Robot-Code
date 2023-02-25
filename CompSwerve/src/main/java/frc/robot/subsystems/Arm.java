// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

public class Arm extends Thread{
  public CANSparkMax stageOne;
  public RelativeEncoder encoderOne;
  public CANSparkMax stageTwo;
  public RelativeEncoder encoderTwo;
  public CANSparkMax wrist;
  public RelativeEncoder encoderWrist;
  //TODO tune PID
  public PIDController onePID = new PIDController(1, 0, 0);
  public PIDController twoPID = new PIDController(1, 0, 0);
  public PIDController wristPID = new PIDController(1, 0, 0);

  public ArrayList<ArmPosition> targetFeeder = new ArrayList<ArmPosition>();
  XboxController m_Controller;

  /** Creates a new Arm. 
   @param m_Limelight **/
  public Arm() {
    stageOne = new CANSparkMax(kStageOneChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoderOne = stageOne.getEncoder();
    stageTwo = new CANSparkMax(kStageTwoChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoderTwo = stageTwo.getEncoder();
    wrist = new CANSparkMax(kWristChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoderWrist = wrist.getEncoder();
    onePID.setTolerance(1);
    twoPID.setTolerance(1);
    wristPID.setTolerance(1);
    SmartDashboard.putNumber("armboi", -10000);
    SmartDashboard.putNumber("wristy", 0);
  }

  public void setAngle(double x, double y) {
    ArmPosition target =  new ArmPosition(x, y, ArmPosition.consType.pose);
    double oneOut = onePID.calculate(encoderOne.getPosition(), target.oneAngle/(2*Math.PI));
    double twoOut = twoPID.calculate(encoderTwo.getPosition(), target.twoAngle/(2*Math.PI));
    double wristOut = wristPID.calculate(encoderWrist.getPosition(), target.wristAngle);
    stageOne.setVoltage(oneOut);
    stageTwo.setVoltage(twoOut);
    wrist.setVoltage(wristOut);
  }

  public void setAngle(double x, double y, double a) {
    ArmPosition target =  new ArmPosition(x, y, a);
    double oneOut = onePID.calculate(encoderWrist.getPosition(), target.oneAngle/(2*Math.PI));
    double twoOut = twoPID.calculate(encoderTwo.getPosition(), target.oneAngle/(2*Math.PI));
    double wristOut = wristPID.calculate(encoderWrist.getPosition(), target.wristAngle);
    stageOne.setVoltage(oneOut);
    stageTwo.setVoltage(twoOut);
    wrist.setVoltage(wristOut);
  }

  public boolean armAtSetPoint(){
    return onePID.atSetpoint() && twoPID.atSetpoint() && wristPID.atSetpoint();
  }

  private void runFeed(){
    if(!targetFeeder.get(0).hasWrist){
      setAngle(targetFeeder.get(0).oneAngle, targetFeeder.get(0).twoAngle);
      //System.out.println("going to: " + targetFeeder.get(0));
      if(armAtSetPoint() && targetFeeder.size() > 1){
        targetFeeder.remove(0);
        System.out.println("popping feed");
      }
    } else {
      setAngle(targetFeeder.get(0).oneAngle, targetFeeder.get(0).twoAngle, targetFeeder.get(0).wristAngle);
      if(armAtSetPoint() && targetFeeder.size() > 1){
        targetFeeder.remove(0);
        System.out.println("popping feed");
      }
    }
  }

  //various position codes

  // holding position
  public void posHolding(){
    targetFeeder.set(0, new ArmPosition(0.1, 0.1, ArmPosition.consType.pose));
    System.out.println("pos is holding");
  }

  public void posTrim(double r){
    ArmPosition pos = new ArmPosition(encoderOne.getPosition(), encoderTwo.getPosition(), encoderWrist.getPosition());
    System.out.println("trimming");
    targetFeeder.add(new ArmPosition(pos.xPos + Math.cos(r), pos.yPos + Math.sin(r), ArmPosition.consType.pose));
  }

  public void log() {
    SmartDashboard.putNumber("armboi", encoderOne.getPosition());
    SmartDashboard.putNumber("wristy", encoderWrist.getPosition());
  }

  public void run(){
    posHolding();
    while (!Thread.interrupted()){
      try{
        Thread.sleep(50);
        log();
        //runFeed();
      }
      catch(Exception ex){
        System.out.println("exception: " + ex);
      }
    }
  }


}
