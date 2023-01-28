// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends Thread {
  /** Creates a new LimeLight. */
  public Limelight() {
   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    //public NetworkTableEntry tx = table.getEntry("tx");
  }

}
