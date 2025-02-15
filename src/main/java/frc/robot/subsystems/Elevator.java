package frc.robot.subsystems;

import java.security.spec.EncodedKeySpec;
import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.exceptions.unfilledConstant;

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
        rightSparkMax = new SparkMax(Constants.ElevatorConstants.rightSparkMaxID, MotorType.kBrushless);

        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        leftConfig.inverted(Constants.ElevatorConstants.leftSparkMaxInverted);
        rightConfig.inverted(Constants.ElevatorConstants.rightSparkMaxInverted);

        leftConfig.smartCurrentLimit(40);
        rightConfig.smartCurrentLimit(40);
        

        leftSparkMax.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightSparkMax.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        relativeEncoder = leftSparkMax.getEncoder();

        bottomSwitch = new DigitalInput(Constants.ElevatorConstants.bottomSwitchID);

        pid = new PIDController(Constants.ElevatorConstants.PID.kP, Constants.ElevatorConstants.PID.kI, Constants.ElevatorConstants.PID.kD);
        pid.setTolerance(Constants.ElevatorConstants.atTargetOffset);
    
        elevatorState = ElevatorState.DOWN;

       // climber.setAngle(0);

       goal = 0; 


    }

    //pidloop
    private double encoderPosition;
    private double goal; 
    @Override
    public void periodic() {
        loggers();
  
        if (getEncoderPosition() < 0 && !getBottomSwitch())
        {
            setOutput(-0.125);
        } else
        {
        setOutput(pid.calculate(getEncoderPosition(), goal));
        }
    }

    public void pidControl()
    {
        setOutput(pid.calculate(encoderPosition, goal));
    }


   //elevator states
    public enum ElevatorState {
        DOWN("DOWN", Constants.ElevatorConstants.SetpointRotations.DOWN, 0),
        SOURCE("SOURCE", Constants.ElevatorConstants.SetpointRotations.SOURCE, 1),
        L1("L1", Constants.ElevatorConstants.SetpointRotations.L1, 2),
        L2("L2", Constants.ElevatorConstants.SetpointRotations.L2, 3),
        L3("L3", Constants.ElevatorConstants.SetpointRotations.L3, 4),
        L4("L4", Constants.ElevatorConstants.SetpointRotations.L4, 5);

        double rotations; 
        String name;
        int index;

        ElevatorState(String name, double rotations, int index) {
            this.rotations = rotations;
            this.name = name;
            this.index = index;
        }

        public static ElevatorState findByIndex(int index)
        {
            ElevatorState[] values = ElevatorState.values();
            for(int i = 0; i < values.length; i++)
            {
                if(values[i].index == index)
                {
                    return values[i];
                }
            }
            throw new Error();
        }
    }


/*FUNCTIONS*/

    //set motor outputs
    public void setOutput(double output) {
        rightSparkMax.set(output);
        leftSparkMax.set(output);
    }

    public void setGoal(double newgoal) {
        goal = newgoal;
    }

    public void manualControl(double input) {
        if (input > 0 && goal + input <= Constants.ElevatorConstants.kMaxElevatorHeightRotations) {
            goal += input;
        } else if (input < 0 && goal + input >= Constants.ElevatorConstants.kMinElevatorHeightRotations) {
            if (goal > 0 && goal < 1) {
                goal = 0;
            } else {
            goal += input;
            }
            
        }
    }

    public void setEncoderPosition(double position) {
        relativeEncoder.setPosition(position);
    }

    public double getEncoderPosition() {
        if(getBottomSwitch()) {
             setEncoderPosition(0);
        }
        return relativeEncoder.getPosition();
    }

    public boolean getBottomSwitch() {
        return !bottomSwitch.get();
    }

    public ElevatorState getElevatorState() {
        return elevatorState;
    }

    public int getElevatorStateIndex() {
        return elevatorState.index;
    }

    public void setStateByIndex(int desiredIndex) {
        elevatorState = ElevatorState.findByIndex(desiredIndex);
        goal = elevatorState.rotations;
    }

    public boolean elevatorAtTarget() throws unfilledConstant
    {
        double offset = Constants.ElevatorConstants.atTargetOffset;
        if (offset == 0)
        {
            throw new unfilledConstant("The atTargetOffset elevator constants is set to zero meaning nothing will work ever");
        }
        if (encoderPosition > goal-offset && encoderPosition < goal+offset)
        {
            return true;
        }
        return false;
    }

/*LOGGERS*/

    private void loggers() {
        SmartDashboard.putNumber("elevator encoder position", getEncoderPosition());
        SmartDashboard.putNumber("elevator pid goal", goal);
        SmartDashboard.putString("elevatorstate", getElevatorState().toString());
        SmartDashboard.putNumber("elevator kp", Constants.ElevatorConstants.PID.kP);
        SmartDashboard.putNumber("elevator ki", Constants.ElevatorConstants.PID.kI);
        SmartDashboard.putNumber("elevator kd", Constants.ElevatorConstants.PID.kD);
        SmartDashboard.putBoolean("elevator bottom switch", getBottomSwitch());
        SmartDashboard.putBoolean("pid at goal", pid.atSetpoint());
    }

/*RUNNABLE ACTIONS FOR BUTTON BOX*/

    public Command goToStateCommand(ElevatorState state) {
        return this.runOnce(() -> setStateByIndex(state.index));
    }

    public Command goToPosition(double position) {
        return this.runOnce(() -> setGoal(position));
    }

    public Command moveMotorsNoPID(double output) {
        return this.runOnce(() -> setOutput(output));
    }

    public Command manualUp() {
        return this.runOnce(() -> manualControl(0.0001));
    }

    public Command manualDown() {
        return this.runOnce(() -> manualControl(-0.0001));
    }

}
