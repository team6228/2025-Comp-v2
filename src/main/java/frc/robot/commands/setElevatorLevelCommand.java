// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class setElevatorLevelCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem mElevatoraSubsystem;
  private final Supplier<Double> mSetPoint;

  double realSetPoint;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public setElevatorLevelCommand(ElevatorSubsystem elevatorSubsystem,Supplier<Double> setPoint) {
    this.mElevatoraSubsystem = elevatorSubsystem;
    this.mSetPoint = setPoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    realSetPoint = mSetPoint.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    realSetPoint = mSetPoint.get();

    mElevatoraSubsystem.setElevatorLevelRoborio(realSetPoint);
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