package frc.robot.subsystems;

import java.security.spec.EncodedKeySpec;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.ResetMode;

public class Elevator extends SubsystemBase{

    private final SparkMax leftSparkMax;
    private final SparkMax rightSparkMax;

    private final SparkBaseConfig leftConfig;
    private final SparkBaseConfig rightConfig;

    private final RelativeEncoder relativeEncoder;

    private final DigitalInput bottomSwitch;

    private final PIDController pid;

    public ElevatorState elevatorState;

    public Elevator() {
        leftSparkMax = new SparkMax(Constants.ElevatorConstants.leftSparkMaxID, MotorType.kBrushless);
        rightSparkMax = new SparkMax(Constants.ElevatorConstants.leftSparkMaxID, MotorType.kBrushless);

        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        rightConfig.follow(leftSparkMax);

        leftConfig.inverted(Constants.ElevatorConstants.leftSparkMaxInverted);
        rightConfig.inverted(Constants.ElevatorConstants.rightSparkMaxInverted);

        leftSparkMax.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightSparkMax.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        relativeEncoder = leftSparkMax.getEncoder();

        bottomSwitch = new DigitalInput(Constants.ElevatorConstants.bottomSwitchPort);

        pid = new PIDController(Constants.ElevatorConstants.PID.kP, Constants.ElevatorConstants.PID.kI, Constants.ElevatorConstants.PID.kD, Constants.ElevatorConstants.PID.kF);
    
        elevatorState = ElevatorState.DOWN;

    }

    
    //pidloop
    private double encoderPosition;
    private double goal; 
    @Override
    public void periodic() {
        setOutput(pid.calculate(encoderPosition, goal));
    
        loggers();

        encoderPosition = bottomSwitch.get() ? 0 : encoderPosition;
        setEncoderPosition(encoderPosition);   
    }


   //elevator states
    public enum ElevatorState {
        DOWN("DOWN", Constants.ElevatorConstants.SetpointRotations.DOWN),
        L1("L1", Constants.ElevatorConstants.SetpointRotations.L1),
        L2("L2", Constants.ElevatorConstants.SetpointRotations.L2),
        L3("L3", Constants.ElevatorConstants.SetpointRotations.L3),
        L4("L4", Constants.ElevatorConstants.SetpointRotations.L4);

        double rotations; 
        String name;

        ElevatorState(String name, double rotations) {
            this.rotations = rotations;
            this.name = name;
        }
    }


/*FUNCTIONS*/

    //set motor outputs
    public void setOutput(double output) {
        leftSparkMax.set(output);
    }

    public void manualControl(double input) {
        goal = input;
    }

    public void setEncoderPosition(double position) {
        relativeEncoder.setPosition(position);
    }

    public double getEncoderPosition() {
        return relativeEncoder.getPosition();
    }

    // public double getEncoderAbsPosition() {
    // return relativeEncoder.getPosition();
    // }

 
    //move elevator to DOWN position
    public void goToDOWN() {
        elevatorState = ElevatorState.DOWN;
        goal = elevatorState.rotations;
    }


    //move elevator to L1 position 
    public void goToL1() {
        elevatorState = ElevatorState.L1;
        goal = elevatorState.rotations;
    }


    //move elevator to L2 position
    public void goToL2() {
        elevatorState = ElevatorState.L2;
        goal = elevatorState.rotations;
    }

    //move elevator to L3 position
    public void goToL3() {
        elevatorState = ElevatorState.L3;
        goal = elevatorState.rotations;
    }

    //move elevator to L4 position
    public void goToL4() {
        elevatorState = ElevatorState.L4;
        goal = elevatorState.rotations;
    }

    public String getElevatorState() {
        return elevatorState.name;
    }


/*LOGGERS*/

    private void loggers() {
        SmartDashboard.putNumber("elevator encoder position", getEncoderPosition());
        SmartDashboard.putNumber("elevator pid goal", goal);
        SmartDashboard.putString("elevatorstate", elevatorState.toString());
        SmartDashboard.putNumber("elevator kp", Constants.ElevatorConstants.PID.kP);
        SmartDashboard.putNumber("elevator ki", Constants.ElevatorConstants.PID.kI);
        SmartDashboard.putNumber("elevator kd", Constants.ElevatorConstants.PID.kD);
        SmartDashboard.putNumber("elevator kf", Constants.ElevatorConstants.PID.kF);
    }

}
