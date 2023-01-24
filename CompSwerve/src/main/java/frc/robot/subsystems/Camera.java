// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.highgui.ImageWindow;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;

public class Camera extends SubsystemBase {

  public UsbCamera camera;
  public CvSink UsbCameraSink;
  protected static CvSource source = CameraServer.putVideo("Camera Output", kImageWidth, kImageHeight);

  

  /** Creates a new Camera. */
  public Camera() {
    camera.setResolution(kImageWidth, kImageHeight);
    camera.setFPS(20);
    UsbCameraSink = CameraServer.getVideo(camera);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
