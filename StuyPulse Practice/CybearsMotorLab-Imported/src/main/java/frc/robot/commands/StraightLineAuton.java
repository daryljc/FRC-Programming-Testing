package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TrajectoryLoader;

public class StraightLineAuton extends SequentialCommandGroup {
    Drivetrain drivetrain;
    Trajectory trajectory;

    public StraightLineAuton(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        trajectory = TrajectoryLoader.getTrajectory("./output/BasicAuton.wpilib.json");
        addCommands(new DrivetrainRamsete(drivetrain, trajectory));
    }
}

