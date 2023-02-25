// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class ArmPosition extends SubsystemBase {

  public static enum consType{
    angle,
    pose
  };

  public double xPos;
  public double yPos;
  public double oneAngle;
  public double twoAngle;
  public double wristAngle;
  public boolean hasWrist;
  
  /** Creates a new ArmPosition. */
  public ArmPosition(double one, double two, consType m) {
    if(m == consType.pose){
      xPos = one;
      yPos = two;
      System.out.println("one = " + one + "two = " + two);
      double[] temp = calculateAngle(one, two);
      oneAngle = temp[0];
      twoAngle = temp[1];
    }
    if(m == consType.angle){
      oneAngle = one;
      twoAngle = two;
      double[] temp = calculatePos(one, two);
      xPos = temp[0];
      yPos = temp[1];
    }
    wristAngle = oneAngle + twoAngle;
    hasWrist = false;
  }

  public ArmPosition(double one, double two, double wrist){
    oneAngle = one;
    twoAngle = two;
    double[] temp = calculatePos(one, two);
    xPos = temp[0];
    yPos = temp[1];
    wristAngle = wrist;
    hasWrist = true;
  }

  public double[] calculateAngle(double x, double y) {
    double[] point = {x, y};
    double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    double alpha = Math.acos((Math.pow(kStageOneLength, 2)+Math.pow(distance, 2)-Math.pow(kStageTwoLength, 2))/(2*kStageOneLength*distance))+Math.atan(point[0]/point[1]); // Stage 1 to ground angle
    double beta = Math.acos((Math.pow(kStageOneLength, 2)+Math.pow(kStageTwoLength, 2)-Math.pow(distance, 2))/(2*kStageTwoLength*kStageOneLength)); // Top angle
    System.out.println(x + " " + y);
    double[] angles = {alpha, beta};
    return angles;
  }

  public double[] calculatePos(double alpha, double beta){
    return new double[] {kStageOneLength*Math.cos(alpha) + kStageTwoLength*Math.cos(beta - Math.PI + alpha), 
      kStageOneLength*Math.sin(alpha) + kStageTwoLength*Math.sin(beta - Math.PI + alpha)};
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
