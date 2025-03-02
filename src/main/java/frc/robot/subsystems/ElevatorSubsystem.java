// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAcceleration;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax masterElevatorMotor = new SparkMax(ElevatorConstants.kMasterMotorCANID, MotorType.kBrushed);
  private final SparkMax slaveElevatorMotor = new SparkMax(ElevatorConstants.kSlaveMotorCANID, MotorType.kBrushed);
  
  //[TODO] Pretty sure you can use an existing configs.Change that and add it to file system
  SparkMaxConfig masterConfig = new SparkMaxConfig();
  SparkMaxConfig slaveConfig = new SparkMaxConfig();

  //This is for plugging directly into roborio
  private final Encoder Encoder = new Encoder(
    ElevatorConstants.kEncoderAChannel,
    ElevatorConstants.kEncoderBChannel,
    ElevatorConstants.kEncoderReversed);

  private TrapezoidProfile.Constraints mConstraints =
      new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration);

  private ProfiledPIDController mController =
      new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,
         mConstraints, ElevatorConstants.kDt);

  private ElevatorFeedforward mFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG,
    ElevatorConstants.kV);

  private double controllerOutput = 0;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage mAppliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  //private final MutVelocity mMeter = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  //private final MutAcceleration mVelocity = RadiansPerSecond.mutable(0);

   /* 
  SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      masterElevatorMotor::setVoltage,
      log -> {
        log.motor("Master Elevator Motor")
          .voltage(mAppliedVoltage.mut_replace(masterElevatorMotor.get() * RobotController.getBatteryVoltage(), Volts))
          .linearPosition(mAngle.mut_replace(Encoder.getDistance(), Rotations))
          .linearVelocity(null)
          .LinearAcceleration();
      },
      this
    ));
  */

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    setInitRoborio();
    setSparkMaxConfig();
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
    SmartDashboard.putNumber("Elevator applied output", masterElevatorMotor.getAppliedOutput());
    SmartDashboard.putNumber("Elevator controller output", controllerOutput);
    SmartDashboard.putNumber("Elevator encoder distance",Encoder.getDistance());
    SmartDashboard.putBoolean("Elevator encoder invert",ElevatorConstants.kEncoderReversed);
    SmartDashboard.putBoolean("Elevator motor invert", ElevatorConstants.motorInvert);
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void setSparkMaxConfig(){
    masterConfig
      .inverted(ElevatorConstants.motorInvert)
      .idleMode(ElevatorConstants.motorIdleMode);

    slaveConfig
      .inverted(ElevatorConstants.motorInvert)
      .idleMode(ElevatorConstants.motorIdleMode)
      .follow(ElevatorConstants.kMasterMotorCANID);

    masterElevatorMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    slaveElevatorMotor.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void setInitRoborio(){
    Encoder.setDistancePerPulse(ElevatorConstants.kDistancePerRevolution);
    Encoder.reset();
  }

  public void testElevator(){
    masterElevatorMotor.set(0.6);
    slaveElevatorMotor.set(0.6);
  }

  public void setElevatorLevelRoborio(double setpoint){
    mController.setGoal(setpoint);

    System.out.println("Setpoint: " + setpoint);

    controllerOutput = mController.calculate(Encoder.getDistance())
      + mFeedforward.calculate(mController.getSetpoint().velocity);

    SmartDashboard.putNumber("Elevator controller output", controllerOutput);

    masterElevatorMotor.setVoltage(controllerOutput);
  }
}
