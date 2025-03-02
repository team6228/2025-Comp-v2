// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OmniDriveSubsystem extends SubsystemBase {
  public final VictorSP leftDriveMotor = new VictorSP(DriveConstants.kLeftDriveMotorPort);
  public final VictorSP rightDriveMotor = new VictorSP(DriveConstants.kRightDriveMotorPort);

  private final Encoder rightEncoder = new Encoder(
    DriveConstants.kRightEncoderAChannel,
    DriveConstants.kRightEncoderBChannel,
    DriveConstants.kRightEncoderReversed);

  private final Encoder leftEncoder = new Encoder(
    DriveConstants.kLeftEncoderAChannel,
    DriveConstants.kLeftEncoderBChannel,
    DriveConstants.kLeftEncoderReversed);

  public final DifferentialDrive robotDifferentialDrive = new DifferentialDrive(leftDriveMotor::set,rightDriveMotor::set);

  /** Creates a new ExampleSubsystem. */
  public OmniDriveSubsystem() {
    setInitEncoders();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("Right Encoder Distance Travelled: " + rightEncoder.getDistance());
    System.out.println("Left Encoder Distance Travelled: " + leftEncoder.getDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void driveArcade(double driveX,double driveY){
    robotDifferentialDrive.arcadeDrive(driveX, driveY);
  }

  private void setInitEncoders(){
    rightEncoder.setDistancePerPulse(DriveConstants.kDistancePerRevolution);
    leftEncoder.setDistancePerPulse(DriveConstants.kDistancePerRevolution);

    rightEncoder.reset();
    leftEncoder.reset();
  }
}
