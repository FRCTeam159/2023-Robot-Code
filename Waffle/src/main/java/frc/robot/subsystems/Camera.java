// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import utils.MJpegClient;

public class Camera extends SubsystemBase {
  /** Creates a new Camera. */
private String limelightURL = "http://limelight.local:5800";
private MJpegClient limelight;
private CvSource sink;


  public Camera() {
    limelight = new MJpegClient(limelightURL);
    sink = CameraServer.putVideo("limelight", 640, 480);
    }
  public Mat getLimelightFrame() {
    Mat mat = new Mat();
    mat = limelight.read();
    return mat;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sink.putFrame(getLimelightFrame());
  }
}
