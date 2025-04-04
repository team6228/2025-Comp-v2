// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.OmniDriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class driveWithTimer extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final OmniDriveSubsystem mOmniDriveSubsystem;
  private final Timer driveTimer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public driveWithTimer(OmniDriveSubsystem OmniDriveSubsystem) {
    this.mOmniDriveSubsystem = OmniDriveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(OmniDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTimer.reset();
    driveTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driveTimer.get() < 1.5){
      System.out.println("Yes king");
      mOmniDriveSubsystem.driveArcade(0, 0.8);
    }else if(driveTimer.get() > 1.5){
      System.out.println("No king");
      mOmniDriveSubsystem.driveArcade(0, 0);
    }
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
