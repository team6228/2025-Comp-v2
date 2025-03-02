// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.OmniDriveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArcadeDriveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrixvateField", "PMD.SingularField"})
  private final OmniDriveSubsystem mOmniDriveSubsystem;
  private final Supplier<Double> speedFunction, rotationFunction;

  double realTimeSpeed = 0;
  double realTimeRotation = 0;  

  double driveX = 0;
  double driveY = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDriveCommand(OmniDriveSubsystem driveSubsystem,
    Supplier<Double> speedFunction,Supplier<Double> rotationFunction) {
    this.mOmniDriveSubsystem = driveSubsystem;
    this.speedFunction = speedFunction;
    this.rotationFunction = rotationFunction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    realTimeSpeed = speedFunction.get();
    realTimeRotation = rotationFunction.get();  

    driveX = 0;
    driveY = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Kod #1");

    realTimeSpeed = speedFunction.get();
    realTimeRotation = rotationFunction.get();

    driveX = 1 * realTimeSpeed * DriveConstants.maxSpeedConstrain;
    driveY = -1 * realTimeRotation * DriveConstants.maxSpeedConstrain;

    System.out.println("X: " + driveX + " Y: " + driveY);
    mOmniDriveSubsystem.driveArcade(driveX, driveY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
