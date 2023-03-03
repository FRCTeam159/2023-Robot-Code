// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.I2C;

public class DriveGyro implements Gyro{
  static public enum gyros {
    FRC450,
    NAVX,
    BNO55  
  } ;
  gyros gyro_type=gyros.FRC450;
  String gyro_name;
  Gyro gyro;
  /** Creates a new Gyros. */
  public DriveGyro(gyros type) {
    gyro_type=type;
    switch(type){
      default:
      case FRC450:
        gyro= new ADXRS450_Gyro();
        gyro_name="FRC450";
        break;
      case NAVX:
        gyro= new AHRS();
        gyro_name="NAVX";
        break;
      case BNO55:
        {
          int[] bnoOffsets = {0, -42, -8, -24, -3, 0, 2, 299, -59, -25, 523};
          gyro=BNO055.getInstance(
            BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
            BNO055.vector_type_t.VECTOR_EULER,
            I2C.Port.kMXP,
            BNO055.BNO055_ADDRESS_A,
            bnoOffsets
            );
        }
        gyro_name="BN055";
      break;
    }
  }
  public gyros getType(){
    return gyro_type;
  }
  public String toString(){
    return gyro_name;
  }
  @Override
  public void close() throws Exception {
    gyro.close(); 
  }
  @Override
  public void calibrate() {
   gyro.calibrate();
  }
  @Override
  public void reset() {
   gyro.reset();
  }
  @Override
  public double getAngle() {
    return gyro.getAngle();
  }
  @Override
  public double getRate() {
    return gyro.getRate();
  }
}
