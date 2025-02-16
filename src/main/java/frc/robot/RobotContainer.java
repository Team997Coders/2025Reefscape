// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.ElevatorAutomaticControl;
import frc.robot.commands.ElevatorManualControl;
import frc.robot.commands.goToTag;
import frc.robot.commands.stop;
import frc.robot.subsystems.Drivebase;
//import frc.robot.subsystems.automation.AutomaticSystems;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraBlock;

import java.util.Arrays;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.commands.CoralIntake;
import frc.robot.commands.CoralOutTake;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeCommandIntake;
import frc.robot.commands.AlgaeCommandOutTake;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final Canandgyro gyro = new Canandgyro(Constants.Gyro.gyroID);

  //CONTROLLERS
  private static XboxController driveStick = new XboxController(0);
  private static XboxController box = new XboxController(1);

  private static CommandXboxController c_driveStick = new CommandXboxController(0);
  final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  //AUTOCHOOSER
  private SendableChooser<Command> autoChooser;

//CAMERA STUFF
  private static final Camera frontCamera = new Camera("pineapple",
      new Transform3d(new Translation3d(0.254, 0, 0.1524), new Rotation3d(0, -0.785, 0)));
  private static final Camera backCamera = new Camera("dragonfruit",
      new Transform3d(new Translation3d(-0.254, 0, 0.1524), new Rotation3d(Math.PI, -0.785, 0)));

  private static final CameraBlock cameraBlock = new CameraBlock(Arrays.asList(frontCamera, backCamera));


  //SUBSYSTEMS
  public final Drivebase drivebase = new Drivebase(gyro, cameraBlock);

  private final Coral m_coral = new Coral();
  
  private final Algae m_algae = new Algae();

  private final Elevator elevator = new Elevator();



  //AUTOMATIC SYSTEMS
//  private final AutomaticSystems systems = new AutomaticSystems(box, drivebase, new Drive(
//      drivebase,
//      () -> getScaledXY(),
//      () -> scaleRotationAxis(driveStick.getRawAxis(4))),
//      m_coral, elevator, driveStick, c_driveStick);

      

  //CONSTRUCTOR
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CanandEventLoop.getInstance();

   // CONFIGURE THE TRIGGER BINDINGS
    drivebase.setDefaultCommand(
       new Drive(
           drivebase,
           () -> getScaledXY(),
           () -> scaleRotationAxis(driveStick.getRawAxis(4))));

    elevator.setDefaultCommand(new ElevatorAutomaticControl(elevator, ()->c_driveStick.povUp().getAsBoolean(),
    ()->c_driveStick.povDown().getAsBoolean()));

    //AUTOCHOOSER
    autoChooser = AutoBuilder.buildAutoChooser("moveForward");
    SmartDashboard.putData("Auto Choser", autoChooser);

    NamedCommands.registerCommand("Pick Up Coral", new CoralIntake(m_coral));
    NamedCommands.registerCommand("Place Coral", new CoralOutTake(m_coral));
    NamedCommands.registerCommand("Elevator Down", elevator.goToStateCommand(ElevatorState.DOWN));
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
    xy[0] = -deadband(driveStick.getLeftX(), DriveConstants.deadband);
    xy[1] = -deadband(driveStick.getLeftY(), DriveConstants.deadband);
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
    SmartDashboard.putNumber("Rotation", scaleRotationAxis(driveStick.getRawAxis(4)));
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
    Command goToTag = new goToTag(drivebase, frontCamera, 0.0);
    Command stop = new stop(goToTag);
   
    //ALGAE COMMANDS
    // m_driverController.a().whileTrue(new AlgaeCommandIntake(m_algae)).onFalse(m_algae.AlgaeStop());
    // m_driverController.b().whileTrue(new AlgaeCommandOutTake(m_algae)).onFalse(m_algae.AlgaeStop());

    m_driverController.a().whileTrue(m_algae.AlgaeIntake(.5)).onFalse(m_algae.AlgaeStop());
    m_driverController.b().whileTrue(m_algae.AlgaeOuttake(.5)).onFalse(m_algae.AlgaeStop());



    //CORAL COMMANDS
    m_driverController.x().whileTrue(m_coral.manualMoveCoralMotorsIntake()).onFalse(m_coral.CoralStop());
    m_driverController.y().whileTrue(m_coral.manualMoveCoralMotorsOutake()).onFalse(m_coral.CoralStop());



    //ELEVATOR COMMANDS
    c_driveStick.povRight().whileTrue(elevator.manualUp());
    c_driveStick.povLeft().whileTrue(elevator.manualDown());
   elevator.setDefaultCommand(new ElevatorManualControl(elevator, () -> c_driveStick.povUp().getAsBoolean(), () -> c_driveStick.povDown().getAsBoolean()));

//  c_driveStick.povUp().whileTrue(elevator.moveMotorsNoPID(0.2)).onFalse(elevator.moveMotorsNoPID(0));
//     c_driveStick.povDown().whileTrue(elevator.moveMotorsNoPID(-0.2)).onFalse(elevator.moveMotorsNoPID(0));

    // c_driveStick.a().onTrue(elevator.goToPosition(112));
    // c_driveStick.b().onTrue(elevator.goToPosition(3));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
