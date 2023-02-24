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

public class Arm extends SubsystemBase {
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
  public Arm(Limelight limelight) {
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
  }

  // // Input x and y (relative to bas of stage 1), returns 2 angles for the 2 stages of the arm
  // public double[] calculateAngle(double x, double y) {
  //   double[] point = {x, y};
  //   double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  //   double alpha = Math.acos((Math.pow(kStageOneLength, 2)+Math.pow(distance, 2)-Math.pow(kStageTwoLength, 2))/(2*kStageOneLength*distance))+Math.atan(point[0]/point[1]); // Stage 1 to ground angle
  //   double beta = Math.acos((Math.pow(kStageOneLength, 2)+Math.pow(kStageTwoLength, 2)-Math.pow(distance, 2))/(2*kStageTwoLength*kStageOneLength)); // Top angle

  //   double[] angles = {alpha, beta};
  //   return angles;
  // }

  // public double[] getPosition(){
  //   double alpha = encoderOne.getPosition();
  //   double beta = encoderTwo.getPosition();
  //   return new double[] {kStageOneLength*Math.cos(alpha) + kStageTwoLength*Math.cos(beta - Math.PI + alpha), 
  //     kStageOneLength*Math.sin(alpha) + kStageTwoLength*Math.sin(beta - Math.PI + alpha)};
  // }

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

  public void runFeed(){
    if(!targetFeeder.get(0).hasWrist){
      setAngle(targetFeeder.get(0).oneAngle, targetFeeder.get(0).twoAngle);
      System.out.println("going to: " + targetFeeder.get(0));
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
    targetFeeder.add(new ArmPosition(0.1, 0.1, ArmPosition.consType.pose));
  }

  public void posTrim(double r){
    ArmPosition pos = new ArmPosition(encoderOne.getPosition(), encoderTwo.getPosition(), encoderWrist.getPosition());
    System.out.println("trimming");
    targetFeeder.add(new ArmPosition(pos.xPos + Math.cos(r), pos.yPos + Math.sin(r), ArmPosition.consType.pose));
  }

  public void log() {
    SmartDashboard.putNumber("armboi", encoderOne.getPosition());
    SmartDashboard.putNumber("volty", stageOne.getAppliedOutput());
    SmartDashboard.putNumber("wristy", encoderWrist.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }
}
