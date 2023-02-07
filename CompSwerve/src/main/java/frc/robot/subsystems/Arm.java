// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
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

  public ArrayList<double[]> targetFeeder = new ArrayList<double[]>();
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
  }

  // Input x and y (relative to bas of stage 1), returns 2 angles for the 2 stages of the arm
  public double[] calculateAngle(double x, double y) {
    double[] point = {x, y};
    double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    double alpha = Math.acos((Math.pow(kStageOneLength, 2)+Math.pow(distance, 2)-Math.pow(kStageTwoLength, 2))/(2*kStageOneLength*distance))+Math.atan(point[1]/point[2]); // Stage 1 to ground angle
    double beta = Math.acos((Math.pow(kStageOneLength, 2)+Math.pow(kStageTwoLength, 2)-Math.pow(distance, 2))/(2*kStageTwoLength*kStageOneLength)); // Top angle

    double[] angles = {alpha, beta};
    return angles;
  }

  public double[] getPosition(){
    double alpha = encoderOne.getPosition();
    double beta = encoderTwo.getPosition();
    return new double[] {kStageOneLength*Math.cos(alpha) + kStageTwoLength*Math.cos(beta - Math.PI + alpha), 
      kStageOneLength*Math.sin(alpha) + kStageTwoLength*Math.sin(beta - Math.PI + alpha)};
  }

  public void setAngle(double x, double y) {
    double[] target = calculateAngle(x, y);
    double wristTarget = target[0]+target[1];
    double oneOut = onePID.calculate(encoderOne.getPosition(), target[0]);
    double twoOut = twoPID.calculate(encoderTwo.getPosition(), target[1]);
    double wristOut = wristPID.calculate(encoderWrist.getPosition(), wristTarget);
    stageOne.setVoltage(oneOut);
    stageTwo.setVoltage(twoOut);
    wrist.setVoltage(wristOut);
  }

  public void setAngle(double x, double y, double a) {
    double[] target = calculateAngle(x, y);
    double wristTarget = a;
    double oneOut = onePID.calculate(encoderWrist.getPosition(), target[0]);
    double twoOut = twoPID.calculate(encoderTwo.getPosition(), target[1]);
    double wristOut = wristPID.calculate(encoderWrist.getPosition(), wristTarget);
    stageOne.setVoltage(oneOut);
    stageTwo.setVoltage(twoOut);
    wrist.setVoltage(wristOut);
  }

  public boolean armAtSetPoint(){
    return onePID.atSetpoint() && twoPID.atSetpoint() && wristPID.atSetpoint();
  }

  public void runFeed(){
    if(targetFeeder.get(0).length < 2){
      setAngle(targetFeeder.get(0)[0], targetFeeder.get(0)[1]);
      if(armAtSetPoint() && targetFeeder.size() > 1){
        targetFeeder.remove(0);
      }
    } else {
      setAngle(targetFeeder.get(0)[0], targetFeeder.get(0)[1], targetFeeder.get(0)[2]);
      if(armAtSetPoint() && targetFeeder.size() > 1){
        targetFeeder.remove(0);
      }
    }
  }

  //various position codes

  // holding position
  public void posHolding(){
    targetFeeder.add(new double[] {0.1, 0.1});
  }

  public void posTrim(double r){
    double x = getPosition()[0];
    double y = getPosition()[1];
    targetFeeder.add(new double[] {x + Math.cos(r), y + Math.sin(r)});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
