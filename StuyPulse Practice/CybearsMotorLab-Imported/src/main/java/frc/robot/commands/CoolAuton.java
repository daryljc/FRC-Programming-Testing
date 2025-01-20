package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TrajectoryLoader;

public class CoolAuton extends SequentialCommandGroup {
    Drivetrain drivetrain;
    Trajectory trajectory;

    public CoolAuton(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        trajectory = TrajectoryLoader.getTrajectory("./output/CoolAuton.wpilib.json");
        addCommands(new DrivetrainRamsete(drivetrain, trajectory));
    }
}

