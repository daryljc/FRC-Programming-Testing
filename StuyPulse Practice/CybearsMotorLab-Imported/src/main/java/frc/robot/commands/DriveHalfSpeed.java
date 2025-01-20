package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveHalfSpeed extends Command{
    private final Drivetrain driveSubsystem;
    
    public DriveHalfSpeed(Drivetrain driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    public void execute() {
        driveSubsystem.arcadeDrive(0.5, 0.5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
