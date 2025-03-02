// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.test_intakeLevel;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.setElevatorLevelCommand;
import frc.robot.commands.shootCommand;
import frc.robot.commands.shooterReloadCommand;
import frc.robot.commands.getAlgea;
import frc.robot.commands.removeAlgea;
import frc.robot.commands.stopIntake;
import frc.robot.commands.test_elevatorCommand;
import frc.robot.commands.ExtendIntakeCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OmniDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final OmniDriveSubsystem mOmniSubsystem = new OmniDriveSubsystem();
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();

  // The driver's controller
  private final CommandJoystick mDriverController =
      new CommandJoystick(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
     
     
    mOmniSubsystem.setDefaultCommand(
      new ArcadeDriveCommand(mOmniSubsystem, () -> mDriverController.getRawAxis(4), () -> mDriverController.getRawAxis(1))
    );
    
    mElevatorSubsystem.setDefaultCommand(
      new test_elevatorCommand(mElevatorSubsystem)
    );

    /* 
    mIntakeSubsystem.setDefaultCommand(
      new test_intakeLevel(mIntakeSubsystem, () -> mDriverController.getRawAxis(4))
    );
    */
  }

  
  private void configureButtonBindings() {
     
    //Extend 
    mDriverController.button(OperatorConstants.kExtendIntakeButton)
      .toggleOnTrue(new ExtendIntakeCommand(mIntakeSubsystem));

    //Retract
    mDriverController.button(OperatorConstants.kRetractIntakeButton)
      .toggleOnTrue(new RetractIntakeCommand(mIntakeSubsystem));

    //Remove Algea
    mDriverController.button(OperatorConstants.kRemoveAlgea)
      .toggleOnFalse(new stopIntake(mIntakeSubsystem))
      .toggleOnTrue(new removeAlgea(mIntakeSubsystem));

    //Get Algea
    mDriverController.button(OperatorConstants.kGetAlgea)
      .toggleOnFalse(new stopIntake(mIntakeSubsystem))
      .toggleOnTrue(new getAlgea(mIntakeSubsystem));

    //Stop Intake
    /* 
    mDriverController.button(OperatorConstants.kStopButton)
      .toggleOnTrue(new stopIntake(mIntakeSubsystem));
    */

    /* 
    //Shoot PVC
    mDriverController.button(OperatorConstants.kShootPvc)
      .toggleOnTrue(new shootCommand(mIntakeSubsystem));

    //Reload PVC
    mDriverController.button(OperatorConstants.kReloadPvc)
      .toggleOnTrue(new shooterReloadCommand(mIntakeSubsystem));
    
    //Feeder
    mDriverController.button(OperatorConstants.kFeederButton)
      .toggleOnTrue(new setElevatorLevelCommand(mElevatorSubsystem,() -> ElevatorConstants.kFeederStationPosition));

    //L1
    mDriverController.button(OperatorConstants.kL1Button)
      .toggleOnTrue(new setElevatorLevelCommand(mElevatorSubsystem,() -> ElevatorConstants.kL1Position));
    
    //L2
    mDriverController.button(OperatorConstants.kL2Button)
      .toggleOnTrue(new setElevatorLevelCommand(mElevatorSubsystem,() -> ElevatorConstants.kL2Position));

    //L3
    mDriverController.button(OperatorConstants.kL3Button)
      .toggleOnTrue(new setElevatorLevelCommand(mElevatorSubsystem,() -> ElevatorConstants.kL3Position));

    //L4
    mDriverController.button(OperatorConstants.kL4Button)
      .toggleOnTrue(new setElevatorLevelCommand(mElevatorSubsystem,() -> ElevatorConstants.kL4Position));
    */
    }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(mOmniSubsystem);
  }
}

