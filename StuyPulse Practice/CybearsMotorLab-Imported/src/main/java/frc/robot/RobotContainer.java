// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.CoolAuton;
import frc.robot.commands.DriveForward;
import frc.robot.commands.DriveHalfSpeed;
import frc.robot.commands.StraightLineAuton;
import frc.robot.Constants;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  private final Drivetrain drivetrain = Drivetrain.getInstance();

  private static SendableChooser<Command> autonChooser = new SendableChooser<>();

  private XboxController driverController = new XboxController(0);

  public RobotContainer() {
    configureButtonBindings();
    configureAutons();

    // Set the default drive command to split-stick arcade drive
    drivetrain.setDefaultCommand(
        new RunCommand(
          () ->
              drivetrain.arcadeDrive(-driverController.getLeftY(), -driverController.getLeftX()*0.65),
              drivetrain
              )
    );
  }

  
  private void configureButtonBindings() {
    new JoystickButton(driverController, XboxController.Button.kY.value)
      .onTrue(new DriveHalfSpeed(drivetrain));
  }

  public Drivetrain getRobotDrive() {
    return drivetrain;
  }

  public void configureAutons() {
    autonChooser.addOption("Straight Line", new StraightLineAuton(drivetrain));
    autonChooser.addOption("Drive Forward Auton", new DriveForward(drivetrain, 10));
    autonChooser.addOption("Midline Sweep", new CoolAuton(drivetrain));
    
    autonChooser.addOption("AD (Amp)", drivetrain.followPathCommand("./pathplanner/autos/AD (Amp).auto"));
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
}
}
