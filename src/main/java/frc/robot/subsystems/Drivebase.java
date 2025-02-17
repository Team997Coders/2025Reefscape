// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.reduxrobotics.canand.CanandDevice;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.revrobotics.spark.config.SmartMotionConfig;
import com.studica.frc.AHRS;

import swervelib.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModuleLocations;
import frc.robot.Constants.DriveConstants.SwerveModules;
import frc.robot.subsystems.vision.CameraBlock;


public class Drivebase extends SubsystemBase {
  private final double DRIVE_REDUCTION = 1.0 / 6.75;
  private final double NEO_FREE_SPEED = 5820.0 / 60.0;
  private final double WHEEL_DIAMETER = 0.1016;
  private final double MAX_VELOCITY = NEO_FREE_SPEED * DRIVE_REDUCTION * WHEEL_DIAMETER * Math.PI;
  private final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (ModuleLocations.dist / Math.sqrt(2.0));

  private final double MAX_VOLTAGE = 12;

  private Canandgyro gyro;

  private SwerveModule frontLeft = new SwerveModule(SwerveModules.frontLeft, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModule frontRight = new SwerveModule(SwerveModules.frontRight, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModule backLeft = new SwerveModule(SwerveModules.backLeft, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModule backRight = new SwerveModule(SwerveModules.backRight, MAX_VELOCITY, MAX_VOLTAGE);

  //                                                       0           1          2         3
  private SwerveModule[] modules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      ModuleLocations.frontLeft,
      ModuleLocations.frontRight,
      ModuleLocations.backLeft,
      ModuleLocations.backRight);

  private SwerveDrivePoseEstimator poseEstimator;

  private SwerveDriveOdometry odometry;

  private Field2d field = new Field2d();

  private SlewRateLimiter slewRateX = new SlewRateLimiter(DriveConstants.slewRate);
  private SlewRateLimiter slewRateY = new SlewRateLimiter(DriveConstants.slewRate);

  private BooleanEntry fieldOrientedEntry;

  private CameraBlock cameraBlock;

  SendableChooser fieldOrientedChooser = new SendableChooser<Boolean>();

  /** Creates a new Drivebase. */
  public Drivebase(Canandgyro gyro, CameraBlock cameraBlock) {
    // fieldOrientedChooser.addOption("field oriented", true);
    // fieldOrientedChooser.addOption("robot oriented", false);
    var inst = NetworkTableInstance.getDefault();
    var table = inst.getTable("SmartDashboard");

    // fieldOrientedChooser.setDefaultOption("field oriented", true);
    // Boolean value; 
    this.fieldOrientedEntry = table.getBooleanTopic("Field Oriented").getEntry( false /* value = fieldOrientedChooser.getSelected().equals(true)? true: false*/);

    this.gyro = gyro;
    this.cameraBlock = cameraBlock;

    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

    poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), getPositions(), odometry.getPoseMeters());

    //ModuleConfig ModuleConfig = new ModuleConfig(WHEEL_DIAMETER/2, 3, WHEEL_DIAMETER, DCMotor.getNEO(2), 1.8, 0);
    //RobotConfig config = new RobotConfig(15, 11.25, ModuleConfig, 0.66);
    RobotConfig config;
     try{
       config = RobotConfig.fromGUISettings();
     } catch (Exception e) {
       // Handle exception as needed
       e.printStackTrace();
       config = null;
     }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    SmartDashboard.putData("Field", field);
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
    
        builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getEncoderRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getState().speedMetersPerSecond, null);
    
        builder.addDoubleProperty("Front Right Angle", () -> frontRight.getEncoderRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getState().speedMetersPerSecond, null);
    
        builder.addDoubleProperty("Back Left Angle", () -> backLeft.getEncoderRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getState().speedMetersPerSecond, null);
    
        builder.addDoubleProperty("Back Right Angle", () -> backRight.getEncoderRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> backRight.getState().speedMetersPerSecond, null);
    
        builder.addDoubleProperty("Robot Angle", () -> gyro.getRotation2d().getRadians(), null);
      }
    });
  }

  public ChassisSpeeds getRobotRelativeSpeeds()
  {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double getFieldAngle() {
    return gyro.getYaw()*(Constants.Gyro.gyroYawConversionFactor);
  }

  public void fieldOrientedDrive(double speedX, double speedY, double rot) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rot,
        Rotation2d.fromDegrees(getFieldAngle()));
    this.drive(speeds);
  }

  public void robotOrientedDrive(double speedX, double speedY, double rot) {
    ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, rot);
    this.drive(speeds);
  }

  public void defaultDrive(double speedX, double speedY, double rot) {
    defaultDrive(speedX, speedY, rot, true);
  }
  public boolean isFieldOriented;
  public void defaultDrive(double speedX, double speedY, double rot, boolean slew) {
   
    if (slew) {
      speedX = slewRateX.calculate(speedX);
      speedY = slewRateY.calculate(speedY);
    }

    if (this.fieldOrientedEntry.get(true)) {
      fieldOrientedDrive(speedX, speedY, rot);
      isFieldOriented = true;
    } else {
      robotOrientedDrive(speedX, speedY, rot);
      isFieldOriented = false;
    }
  }

  /** drive:
   * Move the robot. Given the requested chassis speed (where do we want to go) in meters/sec and radians.
   * moduleStates are in meters/sec and radians (for rotation).
   * This can be a source of angle mismatch degrees <> radians
   */
  private void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds, new Translation2d(0, 0));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY);

    this.frontLeft.drive(moduleStates[0]);
    this.frontRight.drive(moduleStates[1]);
    this.backLeft.drive(moduleStates[2]);
    this.backRight.drive(moduleStates[3]);

    SmartDashboard.putNumber("BL Target Angle", moduleStates[2].angle.getDegrees());
    SmartDashboard.putNumber("BL angle", backLeft.getEncoderRadians());

  }

  public double getMaxVelocity() {
    return MAX_VELOCITY;
  }

  public double getMaxAngleVelocity() {
    return MAX_ANGULAR_VELOCITY;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose2d) {
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose2d);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }


  public void changeDriveMultiplier(double newDriveMultiplier) {
    Constants.DriveConstants.driveMultiplier = newDriveMultiplier;
  }

  public Command setDriveMultiplier(double newDriveMultiplier) {
    return this.runOnce(() -> changeDriveMultiplier(newDriveMultiplier));
  }
 


  @Override
  public void periodic() {
    var positions = getPositions();
    var rotation = gyro.getRotation2d();

    odometry.update(rotation, positions);

    poseEstimator.update(rotation, positions);

    this.cameraBlock.update(poseEstimator);

    field.setRobotPose(poseEstimator.getEstimatedPosition());

  
    SmartDashboard.putData("fieldOriented", fieldOrientedChooser);
  }
}
