// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.m;

public class AutoPlace extends CommandBase {
  Arm m_arm;
  Claw m_claw;
  /** Creates a new AutoPlace. */
  public AutoPlace(Arm arm, Claw claw) {
    m_arm = arm;
    m_claw = claw;

    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_claw.clawSolenoidState(true);
    m_arm.posAuto();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arm.targetFeeder.size() == 3){
      m_claw.clawSolenoidState(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.armAtSetPoint() && (m_arm.targetFeeder.size() == 1);
  }
}
