// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.goToLocation;
import frc.robot.subsystems.Drivebase;
//import frc.robot.subsystems.automation.AutomaticSystems;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraBlock;

import java.util.Arrays;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeToggleIntake;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.automation.AutomaticSystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  //LEDS
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;

  //GYRO
  private Canandgyro gyro = new Canandgyro(Constants.Gyro.gyroID);
  
  //CONTROLLERS
  //private static XboxController driveStick = new XboxController(0);
  //private static XboxController box = new XboxController(1);
  
    private static CommandXboxController c_driveStick;
    // final CommandXboxController m_driverController;
    private static CommandXboxController c_buttonStick;
  
  
  //AUTOCHOOSER
  private SendableChooser<Command> autoChooser;
  
  //CAMERA STUFF
  private static Camera RIGHT_CAMERA;
  private static Camera LEFT_CAMERA;
  private static Camera BACK_CAMERA;
  
  private static CameraBlock cameraBlock;
    
    
  //SUBSYSTEMS
  public final Drivebase drivebase;
    
  private final Coral m_coral;
      
  public final Algae m_algae;
    
  private final Elevator elevator;
    
  //TRIGGERS
  public Trigger coralFirstBeamBreak;
  public Trigger coralSecondBeamBreak;
    
  // AUTOMATIC SYSTEMS
  //private final AutomaticSystems systems;
    
  //CONSTRUCTOR
  //The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
      CanandEventLoop.getInstance();
    
      //GYRO
      gyro = new Canandgyro(Constants.Gyro.gyroID);
  
      //CONTROLLERS
    // driveStick = new XboxController(0);
      //box = new XboxController(1);
  
      c_driveStick = new CommandXboxController(0);
      final CommandXboxController m_driverController =
          new CommandXboxController(OperatorConstants.kDriverControllerPort);

      c_buttonStick = new CommandXboxController(1);
    
    
     //CAMERA STUFF
      RIGHT_CAMERA = new Camera("rightBerry",
          new Transform3d(new Translation3d(Units.inchesToMeters(13.5), -Units.inchesToMeters(11.5), Units.inchesToMeters(8.5)), new Rotation3d(0, Units.degreesToRadians(20), 0)));
      LEFT_CAMERA = new Camera("leftBerry",
          new Transform3d(new Translation3d(Units.inchesToMeters(13.5), Units.inchesToMeters(11.5), Units.inchesToMeters(8.5)), new Rotation3d(0, Units.degreesToRadians(20), 0)));
      BACK_CAMERA = new Camera("backBerry",
          new Transform3d(new Translation3d(-Units.inchesToMeters(13), Units.inchesToMeters(11), Units.inchesToMeters(28.5)), new Rotation3d(0, Units.degreesToRadians(17.5), Math.PI)));
    
      cameraBlock = new CameraBlock(Arrays.asList(RIGHT_CAMERA, LEFT_CAMERA, BACK_CAMERA));


      //INITALIZE SUBSYSTEMS
      drivebase = new Drivebase(gyro, cameraBlock);

      m_coral = new Coral();

      m_algae = new Algae();

      coralFirstBeamBreak = new Trigger(() -> m_coral.BeamBrake1());
      coralSecondBeamBreak = new Trigger(() -> m_coral.BeamBrake2());

      elevator = new Elevator(coralFirstBeamBreak, coralSecondBeamBreak);

      //systems = new AutomaticSystems(box, drivebase, elevator, c_driveStick);
      
      //TRIGGERS   
      // CONFIGURE THE TRIGGER BINDINGS
      drivebase.setDefaultCommand(
        new Drive(
            drivebase,
            () -> getScaledXY(),
            () -> scaleRotationAxis(c_driveStick.getRawAxis(4))));


      m_algae.setDefaultCommand(new AlgaeToggleIntake(m_algae, 
        () -> m_driverController.a().getAsBoolean(), 
        () -> m_driverController.b().getAsBoolean()));

      // m_coral.setDefaultCommand(new CoralAutomatic(m_coral, m_driverController.y(), coralFirstBeamBreak, coralSecondBeamBreak));

      // //elevator.setDefaultCommand(new ElevatorAutomaticControl(elevator, c_driveStick.povUp(), c_driveStick.povDown()));
      // elevator.setDefaultCommand(new ElevatorManualControl(elevator, m_driverController.povRight(), m_driverController.povLeft()));

      //AUTOCHOOSER
      autoChooser = AutoBuilder.buildAutoChooser("moveForward");
      SmartDashboard.putData("Auto Choser", autoChooser);

      NamedCommands.registerCommand("Pick Up Coral", m_coral.manualMoveCoralMotorsIntake());
      NamedCommands.registerCommand("Place Coral", m_coral.manualMoveCoralMotorsOutake());
      NamedCommands.registerCommand("Elevator Source", elevator.goToStateCommand(ElevatorState.SOURCE));
      NamedCommands.registerCommand("Elevator L1", elevator.goToStateCommand(ElevatorState.L1));
      NamedCommands.registerCommand("Elevator L2", elevator.goToStateCommand(ElevatorState.L2));
      NamedCommands.registerCommand("Elevator L3", elevator.goToStateCommand(ElevatorState.L3));
      NamedCommands.registerCommand("Elevator L4", elevator.goToStateCommand(ElevatorState.L4));


    //LEDS
    //   m_led = new AddressableLED(9);

    // // Reuse buffer
    // // Default to a length of 60, start empty output
    // // Length is expensive to set, so only set it once, then just update data
    //   m_ledBuffer = new AddressableLEDBuffer(60);
    //   m_led.setLength(m_ledBuffer.getLength());

    // // Set the data
    //   m_led.setData(m_ledBuffer);
    //   m_led.start();
    

    configureBindings();
  }

  /**
   * {@link edu.wpi.first.math.MathUtil}
   */
  private double deadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    } else {
      return input;
    }
  }

  private double[] getXY() {
    double[] xy = new double[2];
    xy[0] = deadband(c_driveStick.getLeftX(), DriveConstants.deadband);
    xy[1] = deadband(c_driveStick.getLeftY(), DriveConstants.deadband);
    return xy;
  }

  private double[] getScaledXY() {
    double[] xy = getXY();

    // Convert to Polar coordinates
    double r = Math.sqrt(xy[0] * xy[0] + xy[1] * xy[1]);
    double theta = Math.atan2(xy[1], xy[0]);

    // Square radius and scale by max velocity
    r = r * r * drivebase.getMaxVelocity();

    // Convert to Cartesian coordinates
    xy[0] = r * Math.cos(theta);
    xy[1] = r * Math.sin(theta);

    return xy;
  }

  private double squared(double input) {
    return Math.copySign(input * input, input);
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Scaled_X", getScaledXY()[0]);
    SmartDashboard.putNumber("Scaled_Y", getScaledXY()[1]);
    SmartDashboard.putNumber("Rotation", scaleRotationAxis(c_driveStick.getRawAxis(4)));
  }

  @SuppressWarnings("unused")
  private double cube(double input) {
    return Math.copySign(input * input * input, input);
  }

  @SuppressWarnings("unused")
  private double scaleTranslationAxis(double input) {
    return deadband(-squared(input), DriveConstants.deadband) * drivebase.getMaxVelocity();
  }

  private double scaleRotationAxis(double input) {
    return -deadband(squared(input), DriveConstants.deadband) * drivebase.getMaxAngleVelocity() * -0.6;
  }

  public void resetGyro() {
    gyro.setYaw(0);
  }


  public boolean onBlueAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == Alliance.Blue;
    }
    return false;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {   
    //ALGAE COMMANDS

    //CORAL COMMANDS
    coralFirstBeamBreak.onTrue(m_coral.manualMoveCoralMotorsIntake()).onFalse(m_coral.CoralStop());
    coralFirstBeamBreak.and(coralSecondBeamBreak).onTrue(m_coral.manualMoveCoralMotorsIntake()).onFalse(m_coral.CoralStop());
    coralSecondBeamBreak.and(c_driveStick.y()).onTrue(m_coral.manualMoveCoralMotorsOutake()).onFalse(m_coral.CoralStop());
   
    //ELEVATOR COMMANDS
    // c_driveStick.povUp().onTrue(elevator.stateUp());
    // c_driveStick.povDown().onTrue(elevator.stateDown());

    //(1.13, 1.05,
    c_driveStick.x().whileTrue(new goToLocation(drivebase, new Pose2d(0.7, 0.75, new Rotation2d(.9425))));

    c_driveStick.povUp().whileTrue(elevator.manualUp());
    c_driveStick.povDown().whileTrue(elevator.manualDown());

    c_driveStick.rightBumper().onTrue(elevator.stateUp());
    c_driveStick.leftBumper().onTrue(elevator.stateDown());

    c_driveStick.povRight().onTrue(elevator.goToStateCommand(ElevatorState.L4));
    c_driveStick.povLeft().onTrue(elevator.goToStateCommand(ElevatorState.SOURCE));
    
   
    //c_driveStick.rightBumper().onTrue(new goToLocation(drivebase, new Pose2d(2, 2, new Rotation2d(0))));
    //DRIVE STUFF 
    c_driveStick.rightTrigger().onTrue(drivebase.setDriveMultiplier(0.3)).onFalse(drivebase.setDriveMultiplier(1));

    c_buttonStick.a().onTrue(elevator.goToStateCommand(ElevatorState.SOURCE));
    c_buttonStick.x().onTrue(elevator.goToStateCommand(ElevatorState.L2));
    c_buttonStick.y().onTrue(elevator.goToStateCommand(ElevatorState.L3));
    c_buttonStick.b().onTrue(elevator.goToStateCommand(ElevatorState.L4));
    c_buttonStick.rightBumper().onTrue(elevator.goToStateCommand(ElevatorState.L1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * 
   * 
   */


  public Command getAutonomousCommand() {
    /*L4*/
  // return new SequentialCommandGroup(autoChooser.getSelected(), new ParallelRaceGroup(drivebase.setDriveMultiplier(0), m_algae.AlgaeOuttake(Constants.Algae.motorSpin)), new SequentialCommandGroup( new WaitCommand(1), m_algae.AlgaeStop(), elevator.goToStateCommand(ElevatorState.L4), new WaitCommand(3), m_coral.manualMoveCoralMotorsOutake(), new WaitCommand(1), m_coral.CoralStop()));

    /*L1*/
  //return new SequentialCommandGroup(autoChooser.getSelected(), new ParallelRaceGroup(drivebase.setDriveMultiplier(0)), new SequentialCommandGroup( elevator.goToStateCommand(ElevatorState.L1), new WaitCommand(3), m_coral.manualMoveCoralMotorsOutake(), new WaitCommand(1), m_coral.CoralStop(), elevator.stateUp()));

  /*just leave */
  return autoChooser.getSelected(); 
  }
}
