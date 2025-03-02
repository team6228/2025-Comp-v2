// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.OmniDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(OmniDriveSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ArcadeDriveCommand(subsystem,null,null));
  }

  /* 
  public static Command simpleAuto(OmniDriveSubsystem subsystem){
    return Commands.run(new ArcadeDriveCommand(subsystem.robotDifferentialDrive,0.8,0.8), subsystem);
  }
  */

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
