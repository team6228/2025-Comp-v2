// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class test_intakeLevel extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem mIntakeSubsystem;
  private final Supplier<Double> speed;

  double realSpeed = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public test_intakeLevel(IntakeSubsystem intakeSubsystem,Supplier<Double> speed) {
    this.mIntakeSubsystem = intakeSubsystem;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    realSpeed = speed.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //realIntakeSpeed = intakeSpeedFunction.get() < 0 ? 0 : intakeSpeedFunction.get();
    realSpeed = speed.get();

    System.out.println("Uhuh");

    mIntakeSubsystem.testRotation(realSpeed);
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
