// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.RunnableFuture;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class PoseArm extends CommandBase {
  public Arm m_Arm;
  public XboxController m_Controller;
  /** Creates a new PoseArm. */
  public PoseArm(Arm arm, XboxController controller) {
    m_Arm = arm;
    m_Controller = controller;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // DISABLED FOR TESTING 
    // m_Arm.posHolding();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // for testing: gentle velocity control of the arm
    double testVelocity = -1.0 * MathUtil.applyDeadband(m_Controller.getRightY(), 0.2);
    System.out.println("Setting arm speed: " + testVelocity);
    m_Arm.stageOne.set(testVelocity);

    //int direction = m_Controller.getPOV(1);
    //m_Arm.posTrim(Units.degreesToRadians(direction));
    //m_Arm.runFeed();
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
