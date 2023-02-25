// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

public class OneArm extends Thread{

  public static final double kArmAngleOffset = Math.toRadians(90); // starting angle
  public static final double kWristAngleOffset = Math.toRadians(45); // starting angle

  public CANSparkMax stageOne;
  public RelativeEncoder encoderOne;
  public CANSparkMax wrist;
  public RelativeEncoder encoderWrist;
  //TODO tune PID
  public PIDController onePID = new PIDController(1, 0, 0);
  public PIDController wristPID = new PIDController(1, 0, 0);

  XboxController m_Controller;

  public static final double[] kinit =  {0,kStageOneLength,Math.toRadians(45)}; // x,y,wrist

  double X=0;
  double Y=0;
  double arm_angle=0;
  double wrist_angle=0;

  static double maxX=kStageOneLength;
  static double maxY=kStageOneLength;
  static double minX=0;
  static double minY=0;

  static double minA=0;
  static double maxA=Math.toRadians(90);

  static double minW=Math.toRadians(0);
  static double maxW=Math.toRadians(90);

  public OneArm() {
    stageOne = new CANSparkMax(kStageOneChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoderOne = stageOne.getEncoder();
    wrist = new CANSparkMax(kWristChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoderWrist = wrist.getEncoder();
    onePID.setTolerance(1);
    wristPID.setTolerance(1);
    SmartDashboard.putString("Arm","..");
  }

  public double wristAngle() {
    return encoderWrist.getPosition() * 2 * Math.PI + kWristAngleOffset;
  }
  public double armAngle() {
    return encoderOne.getPosition() * 2 * Math.PI + kArmAngleOffset;
  }

  void setAngle(){
    double oneOut = onePID.calculate(armAngle(), arm_angle);
    double wristOut = wristPID.calculate(encoderWrist.getPosition(), wrist_angle);
    stageOne.setVoltage(oneOut);
    wrist.setVoltage(wristOut);
  }

  public boolean armAtSetPoint(){
    return onePID.atSetpoint() && wristPID.atSetpoint();
  }
 
  void calculatePose(double alpha){  
    X=kStageOneLength*Math.cos(alpha);
    Y=kStageOneLength*Math.sin(alpha);
  }
  double calculateAngle(double x, double y){  
    return Math.atan2(y,x);
  }
  double[] getPosition() {
    double alpha = armAngle();
    double x=kStageOneLength*Math.cos(alpha);
    double y=kStageOneLength*Math.sin(alpha);
    double[] tmp={x,y};
    return tmp;
  }
 
  void setX(double x){
    X=x;
    X=X>maxX?maxX:X;
    X=X<minX?minX:X;
  }
  void setY(double y){
    Y=y;
    Y=Y>maxY?maxY:Y;
    Y=Y<minY?minY:Y;
  }
  void setA(double a){
    arm_angle=a;
    arm_angle=arm_angle>maxA?maxA:arm_angle;
    arm_angle=arm_angle<minA?minA:arm_angle;
    calculatePose(arm_angle);
  }
  void setW(double a){
    wrist_angle=a;
    wrist_angle=wrist_angle>maxW?maxW:wrist_angle;
    wrist_angle=wrist_angle<minW?minW:wrist_angle;
  }
  void setPose(double[]p){
    setX(p[0]);
    setY(p[1]);
    wrist_angle=p[2];
    arm_angle=calculateAngle(X,Y);
  }
  // holding position
  public void posHolding(){
    setPose(kinit);
    System.out.println("pos is holding");
  }

  public void setArm(double r){  
    setA(arm_angle+r);
  }
  public void setWrist(double r){  
    setW(wrist_angle+r);
  }

  public void log() {
    double d[]=getPosition();
    String s=String.format("X:%-3.2f(%-3.2f) Y:%-3.1f(%-3.2f) A:%-3.1f(%-3.1f) W:%-3.1f(%-3.1f)",
      d[0],X,
      d[1],Y,
      Math.toDegrees(armAngle()),Math.toDegrees(arm_angle),
      Math.toDegrees(wristAngle()),Math.toDegrees(wrist_angle)
      );
    SmartDashboard.putString("Arm",s);
  }
 
  public void run(){
    posHolding();
    while (!Thread.interrupted()){
      try{
        Thread.sleep(20);
        setAngle();
        log();
      }
      catch(Exception ex){
        System.out.println("exception: " + ex);
      }
    }
  }
}
