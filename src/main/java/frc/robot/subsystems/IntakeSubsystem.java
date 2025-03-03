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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  private final VictorSPX rotationMotor = new VictorSPX(IntakeConstants.kRotationDeviceNumber);
  private final VictorSPX intakeMotor = new VictorSPX(IntakeConstants.kSpeedDeviceNumber);

  public final PIDController mGetController =
    new PIDController(IntakeConstants.kGetAlgeaP, IntakeConstants.kGetAlgeaI, IntakeConstants.kGetAlgeaD);

  public final PIDController mRemoveController =  
    new PIDController(IntakeConstants.kRemoveAlgeaP, IntakeConstants.kRemoveAlgeaI, IntakeConstants.kRemoveAlgeaD);

  public final PIDController mController =
    new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
  
    public final ArmFeedforward mFeedforward = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG,
   IntakeConstants.kV, IntakeConstants.kA);
   
  private final Encoder Encoder = new Encoder(
    IntakeConstants.kEncoderAChannel,
    IntakeConstants.kEncoderBChannel,
    IntakeConstants.kEncoderReversed);

  //[TODO] Change

  public Timer shooterTimer = new Timer();
  public Timer reloadTimer = new Timer();

  private double motorOutput = 0;
  private double controllerOutput = 0;

  private double getMotorOutput = 0;
  private double getControllerOutput = 0;

  private double removeMotorOutput = 0;
  private double removeControllerOutput = 0;

  //Shooting
  public boolean isExtended = false;
  public boolean isRetracted = true;
  public boolean isReloading = false;
  public boolean isRotating = false;
  public boolean isAtMax = false;
  public boolean isAtMin = false;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage mAppliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle mAngle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity mVelocity = RadiansPerSecond.mutable(0);

  //look getIntakeRadiansPosition() detaily

  /* 
  SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      //DONT FORGET ROBOT BATTERY FOR ROTATION MOTOR
      rotationMotor::set,
        log -> {
          log.motor("Rotation Motor")
            .voltage(mAppliedVoltage.mut_replace(rotationMotor.getMotorOutputPercent() * RobotController.getBatteryVoltage(),Volts))
            .angularPosition(mAngle.mut_replace(getIntakeRadiansPosition(), Rotations))
            .angularVelocity(mVelocity.mut_replace(Encoder.getRate(), RotationsPerSecond));
        },
        this
    ));
  */

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    shooterTimer.reset();
    reloadTimer.reset();

    setInitEncoders();

    rotationMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setNeutralMode(NeutralMode.Brake);

    intakeMotor.setInverted(true);
    rotationMotor.setInverted(false);

    isExtended = false;
    isReloading = true;

    
    //Uuuhhh might check this one
    //factor setting stuff coming in here
  }

  /**
   * Example command fassctory method.
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
    SmartDashboard.putNumber("Remove Clamped Output", motorOutput);
    SmartDashboard.putNumber("Remove Output", controllerOutput);
    SmartDashboard.putNumber("Get Clamped Output", getMotorOutput);
    SmartDashboard.putNumber("Get Output", getControllerOutput);
    SmartDashboard.putNumber("Remove Clamped Output", removeMotorOutput);
    SmartDashboard.putNumber("Remove Output", removeControllerOutput);
    SmartDashboard.putNumber("Rotation encoder distance", Encoder.getDistance() + IntakeConstants.encoderOffset);
    SmartDashboard.putNumber("Rotation encoder radian", getIntakeRadiansPosition());
    SmartDashboard.putBoolean("Rotation encoder invert", IntakeConstants.kEncoderReversed);
    SmartDashboard.putBoolean("Rotation motor invert", intakeMotor.getInverted());
    SmartDashboard.putBoolean("Intake motor invert", rotationMotor.getInverted());
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Rotation voltage",rotationMotor.getMotorOutputVoltage());
    SmartDashboard.putBoolean("Intake isExtend", isExtended);
    SmartDashboard.putBoolean("Intake reloading", isReloading);
    SmartDashboard.putBoolean("Intake at min", isAtMin);
    SmartDashboard.putBoolean("Intake at max", isAtMax);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void setInitEncoders(){
    Encoder.setDistancePerPulse(IntakeConstants.kRadianPerRevolution);
    
    Encoder.reset();
  }

  public void testIntake(double intakeSpeed){
    intakeMotor.set(ControlMode.PercentOutput,intakeSpeed);
  }

  public void testRotation(double rotationSpeed){
    //[TODO] Put these to constants
    rotationMotor.set(ControlMode.PercentOutput,rotationSpeed);
  }

  public void reloadShooter(){
    reloadTimer.start();

    if(isExtended){
      System.out.println("CANT RELOAD INTAKE EXTENDED");
      setIntakeRotation(IntakeConstants.kRadianPerRevolution);
      return;
    }

    intakeMotor.set(ControlMode.PercentOutput,IntakeConstants.reloadSpeed);

    if(reloadTimer.get() > IntakeConstants.reloadTime){
      intakeMotor.set(ControlMode.PercentOutput,0);
      return;
    }
  }

  public void shootPvc(){
    shooterTimer.start();
    
    if(isExtended){
      System.out.println("CANT SHOOT INTAKE EXTENDED");
      setIntakeRotation(IntakeConstants.kRetractedRotation);
      return;
    }

    intakeMotor.set(ControlMode.PercentOutput,0.8);

    if(shooterTimer.get() > 1.5){
      intakeMotor.set(ControlMode.PercentOutput,0);
      return;
    }
  }

  public void getAlgea(){
    intakeMotor.set(ControlMode.PercentOutput,IntakeConstants.getAlgeaSpeed);
    isRotating = true;
  }

  public void removeAlgea(){
    intakeMotor.set(ControlMode.PercentOutput,IntakeConstants.removeAlgeaSpeed);
    isRotating = true;
  }

  public void stopIntake(){
    intakeMotor.set(ControlMode.PercentOutput,0.0);
    isRotating = false;
  }

  public void removeAlgeaRotation(){ 
    /* 
    if(IntakeConstants.ksafetyMin > Encoder.getDistance() || Encoder.getDistance() > IntakeConstants.kSafetyMax){
      rotationMotor.set(ControlMode.PercentOutput,0);
    }
    */

    removeControllerOutput = mRemoveController.calculate(Encoder.getDistance() + IntakeConstants.encoderOffset,IntakeConstants.kRetractedRotation)
      + mFeedforward.calculate(IntakeConstants.kRetractedRotation,2);

    //Maaaaybe I can remove this
    removeMotorOutput = MathUtil.clamp(removeControllerOutput / RobotController.getBatteryVoltage(), -1, 1);

    SmartDashboard.putNumber("Rotation Clamped Output", removeMotorOutput);
    SmartDashboard.putNumber("Rotation Output", removeControllerOutput);
    SmartDashboard.putNumber("Rotation error",IntakeConstants.kRetractedRotation - Encoder.getDistance() + IntakeConstants.encoderOffset);

    rotationMotor.set(ControlMode.PercentOutput,removeMotorOutput);
    //rotationMotor.set(ControlMode.PercentOutput,IntakeConstants.kS / RobotController.getBatteryVoltage());
  }

  public void getAlgeaRotation(){ 
    /* 
    if(IntakeConstants.ksafetyMin > Encoder.getDistance() || Encoder.getDistance() > IntakeConstants.kSafetyMax){
      rotationMotor.set(ControlMode.PercentOutput,0);
    }
    */

    getControllerOutput = mGetController.calculate(Encoder.getDistance() + IntakeConstants.encoderOffset,IntakeConstants.kExtendedRotation)
      + mFeedforward.calculate(IntakeConstants.kExtendedRotation,2);

    //Maaaaybe I can remove this
    getMotorOutput = MathUtil.clamp(getControllerOutput / RobotController.getBatteryVoltage(), -1, 1);

    SmartDashboard.putNumber("Rotation Clamped Output", getMotorOutput);
    SmartDashboard.putNumber("Rotation Output", getControllerOutput);
    SmartDashboard.putNumber("Rotation error",IntakeConstants.kExtendedRotation - Encoder.getDistance() + IntakeConstants.encoderOffset);

    rotationMotor.set(ControlMode.PercentOutput,getMotorOutput);
    //rotationMotor.set(ControlMode.PercentOutput,IntakeConstants.kS / RobotController.getBatteryVoltage());
  }
  

  public void setIntakeRotation(double setpoint){ 
    /* 
    if(IntakeConstants.ksafetyMin > Encoder.getDistance() || Encoder.getDistance() > IntakeConstants.kSafetyMax){
      rotationMotor.set(ControlMode.PercentOutput,0);
    }
    */

    controllerOutput = mController.calculate(Encoder.getDistance() + IntakeConstants.encoderOffset,setpoint)
      + mFeedforward.calculate(Encoder.getDistance() + IntakeConstants.encoderOffset,Encoder.getRate());

    //Maaaaybe I can remove this
    motorOutput = MathUtil.clamp(controllerOutput / RobotController.getBatteryVoltage(), -1, 1);

    SmartDashboard.putNumber("Rotation Clamped Output", motorOutput);
    SmartDashboard.putNumber("Rotation Output", controllerOutput);

    rotationMotor.set(ControlMode.PercentOutput,motorOutput);
    //rotationMotor.set(ControlMode.PercentOutput,IntakeConstants.kS / RobotController.getBatteryVoltage());
  }

  private double getIntakeRadiansPosition(){
    //return (Encoder.getDistance() / IntakeConstants.kCountsPerRevolution) * 2 * Math.PI;
    return (Encoder.getDistance() / IntakeConstants.kWheelDiameter) * 2 * Math.PI; 
  }

  public void rotateIntake(double speed){
    rotationMotor.set(ControlMode.PercentOutput,speed);
  }
}
