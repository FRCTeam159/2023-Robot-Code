// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Arm extends SubsystemBase {
  public CANSparkMax stageOne;
  public CANSparkMax stageTwo;
  public CANSparkMax wrist;
  //TODO tune PID
  public PIDController onePID = new PIDController(1, 0, 0);
  public PIDController twoPID = new PIDController(1, 0, 0);
  public PIDController wristPID = new PIDController(1, 0, 0);

  /** Creates a new Arm. */
  public Arm() {
    stageOne = new CANSparkMax(kStageOneChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    stageTwo = new CANSparkMax(kStageTwoChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    wrist = new CANSparkMax(kWristChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
  }

  // Input x and y, returns 3 angles for the 3 parts of the arm
  public double[] calculateAngle(double x, double y) {
    double[] point = {x, y};
    double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    double alpha = Math.acos((Math.pow(kStageOneLength, 2)+Math.pow(distance, 2)-Math.pow(kStageTwoLength, 2))/(2*kStageOneLength*distance))+Math.atan(point[1]/point[2]); // Stage 1 to ground angle
    double beta = Math.acos((Math.pow(kStageOneLength, 2)+Math.pow(kStageTwoLength, 2)-Math.pow(distance, 2))/(2*kStageTwoLength*kStageOneLength)); // Top angle

    double[] angles = {alpha, beta};
    return angles;
  }

  public void setAngle(double x, double y, double wrist) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
