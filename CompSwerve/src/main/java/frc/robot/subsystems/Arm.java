// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmPosition.consType;

import static frc.robot.Constants.*;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

public class Arm extends Thread{
  public CANSparkMax stageOne;
  public RelativeEncoder encoderOne;
  public CANSparkMax stageTwo;
  public RelativeEncoder encoderTwo;
  public CANSparkMax wrist;
  public RelativeEncoder encoderWrist;
  public SparkMaxLimitSwitch stageOneForwardLimit;

  public double offset1;
  public double offset2;
  public double offsetW;
  //TODO tune PID
  public ProfiledPIDController onePID = new ProfiledPIDController(15, 0, 0, new Constraints(Math.PI, Math.PI/2));
  public ProfiledPIDController twoPID = new ProfiledPIDController(15, 0, 0, new Constraints(Math.PI, Math.PI/2));
  public ProfiledPIDController wristPID = new ProfiledPIDController(6, 0, 0, new Constraints(Math.PI, Math.PI/2));

  public ArrayList<ArmPosition> targetFeeder = new ArrayList<ArmPosition>(10);

  public enum pos{
    holding,
    dropMid,
    dropHigh,
    pickupGround,
    dropHighFw,
    none
  };

  public pos currentPos = pos.holding;
  public pos prevPos = pos.none;

  public double c =0;

  /** Creates a new Arm. 
   @param m_Limelight **/
  public Arm() {
    stageOne = new CANSparkMax(kStageOneChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoderOne = stageOne.getEncoder();
    encoderOne.setPositionConversionFactor(kEncoderOnePosConversionFactor);
    stageTwo = new CANSparkMax(kStageTwoChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoderTwo = stageTwo.getEncoder();
    encoderTwo.setPositionConversionFactor(kEncoderTwoPosConversionFactor);
    wrist = new CANSparkMax(kWristChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoderWrist = wrist.getEncoder();
    encoderWrist.setPositionConversionFactor(kEncoderWristPosConversionFactor);

    onePID.setTolerance(0.05);
    twoPID.setTolerance(0.05);
    wristPID.setTolerance(0.05);

    SmartDashboard.putString("simulationtest", "default");
    SmartDashboard.putString("encoders", "default");

    encoderOne.setPosition(0);
    encoderTwo.setPosition(0);
    encoderWrist.setPosition(0);
  }

  public double getStageOneAngle() {
    // return the absolute (robot-relative) angle of the first stage
    return encoderOne.getPosition()+0.29;
  }

  public double getStageTwoAngle() {
    // return the absolute (robot-relative) angle of the first stage
    //System.out.println(encoderTwo.getPositionConversionFactor());
    return -encoderTwo.getPosition()+0.08;//offset2;
  }

  public double getWristAngle(){
    return -encoderWrist.getPosition()+0.5;
  }

  public void setStageOneZero() {
    // Called when the forward limit switch is pressed
    // Sets the angular reference for the first stage
    if (stageOneForwardLimit.isPressed()) {
      encoderOne.setPosition(kStageOneForwardLimitOffset);
      System.out.println("Setting stage one forward reference.");
    }
  }  

  public void setAngle(ArmPosition pos) {
    double oneOut = onePID.calculate(getStageOneAngle(), pos.oneAngle/(2*Math.PI));
    double twoOut = twoPID.calculate(getStageTwoAngle(), pos.twoAngle/(2*Math.PI));
    double wristOut = wristPID.calculate(getWristAngle(), pos.wristAngle/(2*Math.PI));
    //TODO RENENABLE
    stageOne.set(oneOut);
    stageTwo.set(-twoOut);
    wrist.set(-wristOut);
  }

  public boolean armAtSetPoint(){
    double error1 = Math.abs((targetFeeder.get(targetFeeder.size()-1).oneAngle/(2*Math.PI))-getStageOneAngle());
    double error2 = Math.abs((targetFeeder.get(targetFeeder.size()-1).twoAngle/(2*Math.PI))-getStageTwoAngle());
    double errorW = Math.abs((targetFeeder.get(targetFeeder.size()-1).wristAngle/(2*Math.PI))-getWristAngle()); 
    String s = String.format("err1: %-1.3f err2: %-1.3f stpt1: %-3.3f stpt2: %-3.3f ang1: %-3.3f ang2: %3.3f", 
    error1, error2, onePID.getSetpoint().position, twoPID.getSetpoint().position, getStageOneAngle(), getStageTwoAngle());
    System.out.println(s);
    return (error1 < 0.02 && error2 <0.02 && errorW < 0.02);
  }

  private void runFeed(){
      setAngle(targetFeeder.get(targetFeeder.size()-1));
      //System.out.println("going to: " + targetFeeder.get(0));
      if(armAtSetPoint() && targetFeeder.size() > 1){
        targetFeeder.remove(targetFeeder.size()-1);
        System.out.println("popping feed");
      }
  }

  //testing
  public void wristPIDtest(double setpoint){
    double output = wristPID.calculate(encoderWrist.getPosition()/63, setpoint);
    wrist.set(output);
    //System.out.println("pos " + encoderWrist.getPosition()/63 + "setpt" + setpoint + "output" + output);
  }

  public void armPIDtest(double setpoint){
    double output = onePID.calculate(getStageOneAngle(), setpoint);
    //stageOne.set(output);
  }

  public void armPIDtesttwo(double setpoint){
    double output = twoPID.calculate(getStageOneAngle(), setpoint);
    //stageTwo.set(output);
  }

  public void armveloPID(double setpoint){
    double output = wristPID.calculate(encoderOne.getVelocity()/612, setpoint*3.75); //setpoint*3.75
    stageOne.set(output);
    System.out.println("vel " + encoderWrist.getVelocity()/612 + "err " + wristPID.getVelocityError() + "output" + output);
  }

  //various position codes

  // holding position
  public void posHolding(){
    prevPos = currentPos;
    currentPos = pos.holding;
    targetFeeder.clear();
    if(prevPos == pos.dropHigh || prevPos == pos.dropMid || prevPos == pos.dropHighFw){
      targetFeeder.add(new ArmPosition(0.4, 0.4, consType.pose));
      targetFeeder.add(new ArmPosition(1.5, 0.4, consType.pose));
    } else {
      targetFeeder.add(new ArmPosition(0.4, 0.4, consType.pose));
    }
    
    System.out.println("pos is holding");
  }

  public void posPickupGround(){
    prevPos = currentPos;
    currentPos = pos.pickupGround;
    targetFeeder.clear();
    targetFeeder.add(new ArmPosition(0.07, 1, consType.pose));
  }

  public void posDropMid(){
    prevPos = currentPos;
    currentPos = pos.dropMid;
    targetFeeder.clear();
    targetFeeder.add(new ArmPosition(1.1, 1, consType.pose));
    targetFeeder.add(new ArmPosition(0.75, 0.5, 0.27, consType.pose));
  }

  public void posDropHigh(){
    prevPos = currentPos;
    currentPos = pos.dropHigh;
    targetFeeder.clear();
    targetFeeder.add(new ArmPosition(0.747, Math.PI, Math.PI, consType.angle));
    targetFeeder.add(new ArmPosition(1.5, 0.4, consType.pose));
  }

  public void posDropHighFw(){
    prevPos = currentPos;
    currentPos = pos.dropHighFw;
    targetFeeder.clear();
    targetFeeder.add(new ArmPosition(0.857, Math.PI, Math.PI/4, consType.angle));
    targetFeeder.add(new ArmPosition(1.5, 0.4, consType.pose));
  }

  public void posTrim(double r){
    targetFeeder.clear();
    ArmPosition pos = new ArmPosition(getStageOneAngle(), encoderTwo.getPosition(), encoderWrist.getPosition(), consType.angle);
    System.out.println("trimming");
    targetFeeder.set(0, new ArmPosition(pos.xPos + Math.cos(r), pos.yPos + Math.sin(r), ArmPosition.consType.pose));
  }

  //pos code end

  public void log() {
    c++;
    String s = targetFeeder.size() > 0?String.format("alpha: %-3.2f beta: %-3.2f wrist: %-3.2f", targetFeeder.get(0).oneAngle/(2*Math.PI), targetFeeder.get(0).twoAngle/(2*Math.PI), targetFeeder.get(0).wristAngle/(2*Math.PI)): "no things";
    SmartDashboard.putString("simulationtest", s);
    String t = String.format("encoder one: %-3.2f encoder two: %-3.2f encoder wrist: %-3.2f", getStageOneAngle(), getStageTwoAngle(), getWristAngle());
    SmartDashboard.putString("encoders", t);
    if(c%100 == 0){
      // System.out.println("AT SET POINT: "+ armAtSetPoint());
      // System.out.println("FIRST STAGE: " + onePID.atSetpoint());
      // System.out.println("SETPOINT ONE: " + onePID.getSetpoint());
      // System.out.println();
    }
  }

  public void run(){
    posHolding();
    while (!Thread.interrupted()){
      try{
        Thread.sleep(50);
        log();
        runFeed();
      }
      catch(Exception ex){
        System.out.println("exception: " + ex);
      }
    }
  }


}
