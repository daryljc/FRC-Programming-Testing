package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TrajectoryLoader;

public class DrivetrainRamsete extends RamseteCommand {
    private Drivetrain drivetrain;
    private Trajectory trajectory;

    public DrivetrainRamsete(Drivetrain drivetrain, Trajectory trajectory) {
        super(
            trajectory,
            drivetrain::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(0.5, 1),
            new DifferentialDriveKinematics(1),
            drivetrain::getWheelSpeeds,
            new PIDController(2, 0, 0.05),
            new PIDController(2, 0, 0.05),
            drivetrain::setVolts
        );

        this.drivetrain = drivetrain;
        this.trajectory = trajectory;
    }

    public DrivetrainRamsete(Drivetrain drivetrain, String path) {
        this(drivetrain, TrajectoryLoader.getTrajectory(path));
    }
}
