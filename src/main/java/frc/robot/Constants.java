// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final int kRetractIntakeButton = 2;
    public static final int kExtendIntakeButton = 4;
    
    public static final int kGetAlgea = 1;
    public static final int kRemoveAlgea = 3;
    
    public static final int kShootPvc = 3;
    public static final int kReloadPvc = 5;
    
    public static final int kFeederButton = 8;
    public static final int kL1Button = 8;
    public static final int kL2Button = 8;
    public static final int kL3Button = 8;
    public static final int kL4Button = 8;
  }

  public static class DriveConstants{
    public static final double maxSpeedConstrain = 1;

    public static final int kLeftDriveMotorPort = 0;
    public static final int kRightDriveMotorPort = 2;

    public static final int kRightEncoderAChannel = 0;
    public static final int kRightEncoderBChannel = 1;
    public static final boolean kRightEncoderReversed = false;

    public static final int kLeftEncoderAChannel = 2;
    public static final int kLeftEncoderBChannel = 3;
    public static final boolean kLeftEncoderReversed = false;

    public static final double kWheelDiameter = 8.0 / 1000; //In meters
    public static final double kPulsePerRevolution = 1000.0;
    public static final double kDistancePerRevolution = Math.PI * kWheelDiameter / kPulsePerRevolution;
  }

  public static class ElevatorConstants{
    //[TODO] CHANGE THE VALUES OF THIS SHIT
    public static final double kFeederStationPosition = 0.0;
    public static final double kL1Position = 0.0;
    public static final double kL2Position = 0.0;
    public static final double kL3Position = 100.0;
    public static final double kL4Position = 150.0;

    //ROBORIO
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0.01;

    public static final double kDt = 0.02;
    public static final double kMaxVelocity = 1.75;
    public static final double kMaxAcceleration = 0.75;

    public static final double kS = 1.1;
    public static final double kG = 1.2;
    public static final double kV = 1.3;

    //SPARK MAX
    public static final boolean motorInvert = true;
    public static final IdleMode motorIdleMode = IdleMode.kBrake;

    public static final int kMasterMotorPwmPort = 4;
    public static final int kMasterMotorCANID = 1;

    public static final int kSlaveMotorPwmPort = 6;
    public static final int kSlaveMotorCANID = 2;

    public static final int kEncoderAChannel = 4;
    public static final int kEncoderBChannel = 5;
    public static final boolean kEncoderReversed = false;

    public static final double kWheelDiameter = 8.0 / 1000; //In meters
    public static final int kPulsePerRevolution = 1000;
    public static final int kCountsPerRevolution = kPulsePerRevolution * 4;
    public static final double kDistancePerRevolution = Math.PI * kWheelDiameter / kPulsePerRevolution;
  }

  public static class IntakeConstants{
    public static final int kRotationDeviceNumber= 4;
    public static final int kSpeedDeviceNumber = 5; 

    //[TODO] Change
    //In radians
    /* 
    public static final double kRetractedRotation = -0.440214;
    public static final double kExtendedRotation =   0.066366; 
    */

    public static final double kRetractedRotation = 0.701752;
    public static final double kExtendedRotation =  1.271558; 

    public static final double ksafetyMin = 0.0;
    public static final double kSafetyMax = 0.0;

    public static final double getAlgeaSpeed = 0.5;
    public static final double removeAlgeaSpeed = -0.5;

    public static final double reloadSpeed = 0.35;
    public static final double shooterSpeed = 0.8;

    public static final double reloadTime = 1.5;
    public static final double shooterTime = 2;

    public static final int kEncoderAChannel = 6;
    public static final int kEncoderBChannel = 7;
    public static final boolean kEncoderReversed = true;
    public static final double encoderOffset = 0.574518;

    public static final double kP = 7.5;
    public static final double kI = 0.01;
    public static final double kD = 1.0;

    public static final double kGetAlgeaP = 3.5;
    public static final double kGetAlgeaI = 0.01;
    public static final double kGetAlgeaD = 2.0;

    /* 
    public static final double kRemoveAlgeaP = 7.5;
    public static final double kRemoveAlgeaI = 0.01;
    public static final double kRemoveAlgeaD = 1.0;
    */

    public static final double kRemoveAlgeaP = 8.5;
    public static final double kRemoveAlgeaI = 0.01;
    public static final double kRemoveAlgeaD = 4.5;

    public static final double kDt = 0.02;
    public static final double kMaxVelocity = 5.0;
    public static final double kMaxAcceleration = 10.0;

    /*
     public static double kS = 0.1;
    public static double kG = 0.31;
    public static double kV = 0.07;
    public static double kA = 0.21;
    */

    public static double kS = 3.55;
    public static double kG = -4.3;
    public static double kV = 2.0;
    public static double kA = 3.0;

    public static final double kWheelDiameter = 6.54 / 1000; //In meters
    public static final int kPulsePerRevolution = 1000;
    public static final int kCountsPerRevolution = kPulsePerRevolution * 4;
    public static final double kDistancePerRevolution = Math.PI * kWheelDiameter / kPulsePerRevolution;
    public static final double kRadianPerRevolution = 2 * Math.PI / kCountsPerRevolution;
    public static final double kRotationPerRevolution = 1 / kCountsPerRevolution;
  }
}
