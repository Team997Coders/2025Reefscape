// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Autos;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.goToBoxCommand;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraBlock;

import java.util.Arrays;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.automation.AutomaticSystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
  private static XboxController box = new XboxController(1);
  
    private static CommandXboxController c_driveStick;
    // final CommandXboxController m_driverController;
    private static CommandXboxController c_studentController;
  
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
  public Trigger algaeBeamBreak;
    
  // AUTOMATIC SYSTEMS
  private final AutomaticSystems systems;

  //AUTOS
  private frc.robot.subsystems.Autos autos;
  private final SendableChooser<Command> autoChooser;

    
  //CONSTRUCTOR
  //The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    UsbCamera drivercamera = CameraServer.startAutomaticCapture();
    drivercamera.setResolution(640, 480);
    drivercamera.setFPS(15);

    
     CanandEventLoop.getInstance();
    
      //GYRO
      gyro = new Canandgyro(Constants.Gyro.gyroID);
  
      //CONTROLLERS
    // driveStick = new XboxController(0);
      box = new XboxController(1);
      c_driveStick = new CommandXboxController(0);      
      c_studentController = new CommandXboxController(1);

    
    
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
      algaeBeamBreak = new Trigger(() -> m_algae.getBeamBreakStatus());

      elevator = new Elevator(coralFirstBeamBreak, coralSecondBeamBreak);

      systems = new AutomaticSystems(box, drivebase, elevator, c_driveStick);
      
      
      drivebase.setDefaultCommand(
        new Drive(
            drivebase,
            () -> getScaledXYStudentController(),
            () -> scaleRotationAxis(c_studentController.getRawAxis(4))));

    

      drivebase.setDriveMultiplier(0.3);
                
      //AUTOS
      autos = new Autos(drivebase, elevator, m_coral, m_algae);
      autoChooser = new SendableChooser<>();

      autoChooser.setDefaultOption("do nothing", new Command() {});
      autoChooser.addOption("taxi", AutoBuilder.buildAuto("moveForward"));
      

     // blue
      autoChooser.addOption("left front blue", autos.LeftTag21Blue());
      autoChooser.addOption("left barge side blue", autos.LeftTag20Blue());


      //red
      autoChooser.addOption("left front red", autos.LeftTag10Red());
      autoChooser.addOption("right front red", autos.RightTag10Red());
      autoChooser.addOption("left barge side red", autos.LeftTag11Red());
      
      
      SmartDashboard.putData("Auto Choser", autoChooser);

      NamedCommands.registerCommand("Pick Up Coral", m_coral.manualMoveCoralMotorsIntake());
      NamedCommands.registerCommand("Place Coral", m_coral.manualMoveCoralMotorsOutake());
      NamedCommands.registerCommand("Elevator Source", elevator.goToStateCommand(ElevatorState.SOURCE));
      NamedCommands.registerCommand("Elevator L1", elevator.goToStateCommand(ElevatorState.L1));
      NamedCommands.registerCommand("Elevator L2", elevator.goToStateCommand(ElevatorState.L2));
      NamedCommands.registerCommand("Elevator L3", elevator.goToStateCommand(ElevatorState.L3));
      NamedCommands.registerCommand("Elevator L4", elevator.goToStateCommand(ElevatorState.L4));

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


  private double[] getXYStudentController() {
    double[] xy = new double[2];
    xy[0] = deadband(c_studentController.getLeftX(), DriveConstants.deadband);
    xy[1] = deadband(c_studentController.getLeftY(), DriveConstants.deadband);
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

  private double[] getScaledXYStudentController() {
    double[] xy = getXYStudentController();

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

    SmartDashboard.putData(CommandScheduler.getInstance());
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


    c_driveStick.x().whileTrue(new Drive(
      drivebase,
      () -> getScaledXY(),
      () -> scaleRotationAxis(c_driveStick.getRawAxis(4))));


    //ALGAE COMMANDS
    c_driveStick.a().whileTrue(m_algae.AlgaeIntake(Constants.Algae.motorSpin));
    algaeBeamBreak.whileTrue(m_algae.AlgaeIntake(Constants.Algae.motorSpin));
    c_driveStick.b().and(algaeBeamBreak).whileTrue(m_algae.AlgaeOuttake(Constants.Algae.motorSpin));
    c_driveStick.a().and(c_driveStick.b()).and(algaeBeamBreak).whileFalse(m_algae.AlgaeStop());


    //CORAL COMMANDS
    coralFirstBeamBreak.onTrue(m_coral.manualMoveCoralMotorsIntake()).onFalse(m_coral.CoralStop());
    coralFirstBeamBreak.and(coralSecondBeamBreak).onTrue(m_coral.manualMoveCoralMotorsIntake()).onFalse(m_coral.CoralStop());
    coralSecondBeamBreak.and(c_driveStick.y()).onTrue(m_coral.manualMoveCoralMotorsOutake()).onFalse(m_coral.CoralStop());
   

    //ELEVATOR COMMANDS
    c_driveStick.povUp().whileTrue(elevator.manualUp());
    c_driveStick.povDown().whileTrue(elevator.manualDown());

    c_driveStick.rightBumper().onTrue(elevator.stateUp());
    c_driveStick.leftBumper().onTrue(elevator.stateDown());

    c_driveStick.povRight().onTrue(elevator.goToStateCommand(ElevatorState.L4));
    c_driveStick.povLeft().onTrue(elevator.goToStateCommand(ElevatorState.SOURCE));

    // c_buttonStick.a().onTrue(elevator.goToStateCommand(ElevatorState.SOURCE));
    // c_buttonStick.x().onTrue(elevator.goToStateCommand(ElevatorState.L2));
    // c_buttonStick.y().onTrue(elevator.goToStateCommand(ElevatorState.L3));
    // c_buttonStick.b().onTrue(elevator.goToStateCommand(ElevatorState.L4));
    // c_buttonStick.rightBumper().onTrue(elevator.goToStateCommand(ElevatorState.L1));
    

    systems.buttonBox.go.whileTrue(new goToBoxCommand(drivebase, () -> systems.getSelectedScoreSide(), () -> systems.getTagFromBox(), systems));
    
   
    //DRIVE STUFF 
   // c_driveStick.rightTrigger().onTrue(drivebase.setDriveMultiplier(0.3)).onFalse(drivebase.setDriveMultiplier(1));
    c_driveStick.leftTrigger().whileTrue(drivebase.robotCentric()).whileFalse(drivebase.fieldOriented());

    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * 
   * 
   */


  public Command getAutonomousCommand() {

   return autoChooser.getSelected(); 

  }
} 
