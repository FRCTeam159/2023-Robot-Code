// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.highgui.ImageWindow;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.apriltag.jni.AprilTagJNI;
import edu.wpi.first.apriltag.jni.DetectionResult;

public class AprilTagDetector extends Thread {
  /** Creates a new AprilTagDetector. */
  static int imageWidth = 640;
  static int imageHeight = 480;
  private UsbCamera camera;
  private CvSink UsbCameraSink;
  static Mat mat;
  static Mat graymat = new Mat();
  protected static CvSource source = CameraServer.putVideo("Camera Output", imageWidth, imageHeight);
  protected long wpi_detector = 0;

  public double h_FOV = 50;
  public double aspect = ((double)imageWidth ) / imageHeight;
  public double v_FOV = 50;
  public double cx = imageWidth/2;
  public double cy = imageHeight/2;
  public double fx = cx / Math.tan(0.05*Math.toRadians(h_FOV));
  public double fy = cy / Math.tan(0.05*Math.toRadians(v_FOV));

  public static double targetSize = Units.inchesToMeters(6);


  public AprilTagDetector() {
    wpi_detector = AprilTagJNI.aprilTagCreate("tag16h5", 2.0, 0.0, 1, false, true);
    camera = CameraServer.startAutomaticCapture(0);
    camera.setResolution(imageWidth, imageHeight);
    camera.setFPS(20);

    UsbCameraSink = CameraServer.getVideo(camera);

  }

  private DetectionResult[] getTags(Mat mat) { 
    Imgproc.cvtColor(mat, graymat, Imgproc.COLOR_RGB2GRAY);
    DetectionResult[] result = AprilTagJNI.aprilTagDetect(wpi_detector, mat, true, targetSize, fx, fy, cx, cy, 1);
    return result;
  }

  private Mat getUsbCameraFrame() {
    Mat mat = new Mat();
    UsbCameraSink.grabFrame(mat);
    return mat;
  }

  public void putFrame(CvSource source, Mat m) {
    if (m != null) {
      source.putFrame(m);
    }
  }
  @Override
  public void run() {
    while (true) {
    try {
      Thread.sleep(50);

      UsbCameraSink.grabFrame(mat);
      DetectionResult[] detections = getTags(mat);
      for (int i = 0;i<detections.length;i++) {
        DetectionResult det = detections[i];
      }
      putFrame(source, mat);

    } catch (Exception e) {
      System.out.println("apriltag exception: " + e);
    }
  }
}

}
