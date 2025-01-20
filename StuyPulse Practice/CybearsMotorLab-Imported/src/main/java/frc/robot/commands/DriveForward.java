package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveForward extends Command {
    private final Drivetrain driveSubsystem;
    private final double distance;

    public DriveForward(Drivetrain driveSubsystem, double distance) {
        this.driveSubsystem = driveSubsystem;
        this.distance = driveSubsystem.getDistance() + distance;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("DriveForward Command started!");
    }

    @Override
    public void execute() {
        driveSubsystem.setMotors(Constants.Drivetrain.autoSpeed, Constants.Drivetrain.autoSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setMotors(0, 0);
        System.out.println("DriveForward Command ended!");
    }

    @Override
    public boolean isFinished() {
        return (driveSubsystem.getDistance() > distance);
    }
}
