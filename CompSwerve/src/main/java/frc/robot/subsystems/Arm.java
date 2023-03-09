// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  public PIDController onePID = new PIDController(20, 0, 0);
  public PIDController twoPID = new PIDController(20, 0, 0);
  public PIDController wristPID = new PIDController(3, 0, 0);

  public ArrayList<ArmPosition> targetFeeder = new ArrayList<ArmPosition>(10);

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

    onePID.setTolerance(0.5);
    twoPID.setTolerance(0.5);
    wristPID.setTolerance(0.5);

    SmartDashboard.putString("simulationtest", "default");
    SmartDashboard.putString("encoders", "default");

    encoderOne.setPosition(0);
    encoderTwo.setPosition(0);
    encoderWrist.setPosition(0);
  }

  public double getStageOneAngle() {
    // return the absolute (robot-relative) angle of the first stage
    return encoderOne.getPosition()+0.25;
  }

  public double getStageTwoAngle() {
    // return the absolute (robot-relative) angle of the first stage
    //System.out.println(encoderTwo.getPositionConversionFactor());
    return -encoderTwo.getPosition()+0.5;//offset2;
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
    double wristOut = wristPID.calculate(getWristAngle(), pos.wristAngle/(2*Math.PI)-0.042);
    stageOne.set(oneOut);
    stageTwo.set(-twoOut);
    wrist.set(-wristOut);
    //System.out.println(oneOut);
  }

  public boolean armAtSetPoint(){
    return onePID.atSetpoint() && twoPID.atSetpoint() && wristPID.atSetpoint();
  }

  private void runFeed(){
      setAngle(targetFeeder.get(0));
      //System.out.println("going to: " + targetFeeder.get(0));
      if(armAtSetPoint() && targetFeeder.size() > 1){
        ArrayList<ArmPosition> temp = new ArrayList<ArmPosition>(10);
        temp.addAll(targetFeeder.subList(1, targetFeeder.size()-1));
        targetFeeder.clear();
        targetFeeder.addAll(temp);
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
    targetFeeder.clear();
    targetFeeder.add(new ArmPosition(0.4, 0.5, ArmPosition.consType.pose));
    System.out.println("pos is holding");
  }

  public void posPickupGround(){
    targetFeeder.clear();
    targetFeeder.add(new ArmPosition(0.07, 1, ArmPosition.consType.pose));
  }

  public void posDropMid(){
    targetFeeder.clear();
    targetFeeder.add(new ArmPosition(1.1, 1, ArmPosition.consType.pose));
  }

  public void posDropHigh(){
    targetFeeder.clear();
    targetFeeder.add(new ArmPosition(Math.PI/2, Math.PI, Math.PI));
    targetFeeder.add(new ArmPosition(0.8, Math.PI, Math.PI));
  }

  public void posTrim(double r){
    targetFeeder.clear();
    ArmPosition pos = new ArmPosition(getStageOneAngle(), encoderTwo.getPosition(), encoderWrist.getPosition());
    System.out.println("trimming");
    targetFeeder.set(0, new ArmPosition(pos.xPos + Math.cos(r), pos.yPos + Math.sin(r), ArmPosition.consType.pose));
  }

  //pos code end

  public void log() {
    String s = targetFeeder.size() > 0?String.format("alpha: %-3.2f beta: %-3.2f wrist: %-3.2f", targetFeeder.get(0).oneAngle/(2*Math.PI), targetFeeder.get(0).twoAngle/(2*Math.PI), targetFeeder.get(0).wristAngle/(2*Math.PI)): "no things";
    SmartDashboard.putString("simulationtest", s);
    String t = String.format("encoder one: %-3.2f encoder two: %-3.2f encoder wrist: %-3.2f", getStageOneAngle(), getStageTwoAngle(), getWristAngle());
    SmartDashboard.putString("encoders", t);
    //System.out.println("AT SET POINT: "+ armAtSetPoint());
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
