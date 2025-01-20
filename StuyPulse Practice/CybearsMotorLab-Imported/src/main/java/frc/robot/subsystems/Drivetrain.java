package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.ADIS16448_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Drivetrain extends SubsystemBase {

  private static final Drivetrain instance = new Drivetrain();

  public static Drivetrain getInstance() {
    return instance;
  }

  private final PWMSparkMax[] leftMotors;
  private final PWMSparkMax[] rightMotors;

  private final Encoder leftEncoder;
  private final Encoder rightEncoder;

  private final DifferentialDrive driveTrain;

  private final ADIS16448_IMU gyro;
  private final DifferentialDriveOdometry odometry;
  private final Field2d field;
  private final FieldObject2d robotPose;
  private final DifferentialDriveKinematics kinematics;

  private final SimpleMotorFeedforward m_feedforward;
  private final PIDController m_leftPIDController;
  private final PIDController m_rightPIDController;

  private final DifferentialDrivetrainSim driveTrainSim;
  private final EncoderSim leftEncoderSim;
  private final EncoderSim rightEncoderSim;
  private final ADIS16448_IMUSim gyroSim;
  
  private final Joystick joystick;

  public Drivetrain() {

    leftMotors = new PWMSparkMax[] {
      new PWMSparkMax(Constants.Ports.LEFT_TOP),
      new PWMSparkMax(Constants.Ports.LEFT_BOTTOM)
    };

    rightMotors = new PWMSparkMax[] {
      new PWMSparkMax(Constants.Ports.RIGHT_TOP),
      new PWMSparkMax(Constants.Ports.RIGHT_BOTTOM)
    };

    leftEncoder = new Encoder(Constants.Ports.LEFT_A, Constants.Ports.LEFT_B);
    rightEncoder = new Encoder(Constants.Ports.RIGHT_A, Constants.Ports.RIGHT_B);
    
    leftMotors[0].addFollower(leftMotors[1]);
    rightMotors[0].addFollower(rightMotors[1]);

    // the motor controller for rightMotors[1] follows that of rightMotors[0],
    // so it will also invert
    rightMotors[0].setInverted(true);

    joystick = new Joystick(0);

    driveTrain = new DifferentialDrive(leftMotors[0], rightMotors[0]);
    leftEncoder.setDistancePerPulse(Constants.Drivetrain.DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(Constants.Drivetrain.DISTANCE_PER_PULSE);

    kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH);

    gyro = new ADIS16448_IMU();
    odometry = new DifferentialDriveOdometry(getRotation2d(), getLeftDistance(), getRightDistance()); //Idk if this is right

    field = new Field2d();
    SmartDashboard.putData("Field", field);
    robotPose = field.getObject("Robot Pose");

    // Example constants, must be empirically determined
    m_feedforward = new SimpleMotorFeedforward(1, 3);
    m_leftPIDController = new PIDController(1, 0, 0);
    m_rightPIDController = new PIDController(1, 0, 0);

    try{
      Constants.Drivetrain.config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      System.exit(1);
    }

    driveTrainSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),
      Constants.Drivetrain.GEAR_RATIO,
      Constants.Drivetrain.INERTIA,
      Constants.Drivetrain.MASS,
      Units.inchesToMeters(3),
      Units.inchesToMeters(24),
      null);

    leftEncoderSim = new EncoderSim(leftEncoder);
    rightEncoderSim = new EncoderSim(rightEncoder);
    gyroSim = new ADIS16448_IMUSim(gyro);
  }

  public void arcadeDrive(double fwd, double rot) {
    driveTrain.arcadeDrive(fwd, rot);
  }

    public void drive() {
    double speed = -joystick.getRawAxis(1) * 0.6;
    double turn = joystick.getRawAxis(4) * 0.3;

    double left = speed + turn;
    double right = speed - turn;

    leftMotors[0].set(left);
    rightMotors[0].set(-right);
  }

  public void setMotors(double leftSpeed, double rightSpeed) {
    leftMotors[0].set(leftSpeed);
    rightMotors[0].set(rightSpeed);
  }

  //Encoder
  public double getLeftDistance() {
    return leftEncoder.getDistance();
  }

  public double getRightDistance() {
    return rightEncoder.getDistance();
  }

  public double getDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }

  public double getLeftVelocity() {
    return leftEncoder.getRate();
   }

  public double getRightVelocity() {
    return rightEncoder.getRate();
  }

  public double getVelocities() {
    return (getLeftVelocity() + getRightVelocity()) / 2.0;
  }

  public void setVolts(double leftVolts, double rightVolts) {
    double battery = RobotController.getBatteryVoltage();
    leftMotors[0].set(leftVolts / battery);
    rightMotors[0].set(rightVolts / battery); 
  }

  public Command followPathCommand(String pathName) {
    try{
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        return new FollowPathCommand(
                path,
                this::getPose, // Robot pose supplier
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
                Constants.Drivetrain.config, // The robot configuration
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                this // Reference to this subsystem to set requirements
        );
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPIDController.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(rightEncoder.getRate(), speeds.rightMetersPerSecond);
    leftMotors[0].setVoltage(leftOutput + leftFeedforward);
    rightMotors[0].setVoltage(rightOutput + rightFeedforward);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    setSpeeds(kinematics.toWheelSpeeds(speeds));
  }

  //Odometry
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }
 
  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees((gyro.getAngle()));
  }

  public void resetOdometry(Pose2d pose) {
    leftEncoder.reset();
    rightEncoder.reset();
    odometry.resetPosition(
      Rotation2d.fromDegrees(getHeading()),
      leftEncoder.getDistance(),
      rightEncoder.getDistance(),
      pose
    );
  }

  public void updateOdometry() {
    odometry.update(
      Rotation2d.fromDegrees(getHeading()),
      leftEncoder.getDistance(),
      rightEncoder.getDistance()
    );
  }

  @Override
  public void periodic() {
    updateOdometry();

    SmartDashboard.putNumber("Debug/Drivetrain/Distance Traveled (m)", getDistance());
    SmartDashboard.putNumber("Debug/Drivetrain/Distance Traveled Left (m)", getLeftDistance());
    SmartDashboard.putNumber("Debug/Drivetrain/Distance Traveled Right (m)", getRightDistance());
    SmartDashboard.putNumber("Debug/Drivetrain/Robot Velocity (m per s)", getVelocities());
    SmartDashboard.putNumber("Debug/Drivetrain/Velocity Left (m per s)", getLeftVelocity());
    SmartDashboard.putNumber("Debug/Drivetrain/Velocity Right (m per s)", getRightVelocity());
  }

  @Override
  public void simulationPeriodic() {
    double joystickX = joystick.getX();
    double joystickY = joystick.getY();
    if (Math.abs(joystickX) >= 0) {
      driveTrainSim.setInputs(
        joystickX * 0.5,
        -joystickX * 0.5 
      );
    }
    if (Math.abs(joystickY) >= 0.01) {
        driveTrainSim.setInputs(
          -joystickY,
          -joystickY 
        );
    }
    

    robotPose.setPose(driveTrainSim.getPose());
    field.setRobotPose(driveTrainSim.getPose());
    SmartDashboard.putData(field);
    driveTrainSim.update(0.02);
    
    leftEncoderSim.setDistance(driveTrainSim.getLeftPositionMeters());
    leftEncoderSim.setRate(driveTrainSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(driveTrainSim.getRightPositionMeters());
    rightEncoderSim.setRate(driveTrainSim.getRightVelocityMetersPerSecond());
    gyroSim.setGyroAngleZ(-driveTrainSim.getHeading().getDegrees());
  }

}