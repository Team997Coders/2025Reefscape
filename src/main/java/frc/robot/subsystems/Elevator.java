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

import edu.wpi.first.math.controller.ElevatorFeedforward;
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
    private final ElevatorFeedforward feedforward; 

    public ElevatorState elevatorState;
    public final Servo climber;

    public Elevator() {
        leftSparkMax = new SparkMax(Constants.ElevatorConstants.leftSparkMaxID, MotorType.kBrushless);
        rightSparkMax = new SparkMax(Constants.ElevatorConstants.rightSparkMaxID, MotorType.kBrushless);

        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        rightConfig.follow(leftSparkMax);

        leftConfig.inverted(Constants.ElevatorConstants.leftSparkMaxInverted);
        rightConfig.inverted(Constants.ElevatorConstants.rightSparkMaxInverted);

        leftSparkMax.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightSparkMax.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        relativeEncoder = leftSparkMax.getEncoder();

        bottomSwitch = new DigitalInput(Constants.ElevatorConstants.bottomSwitchID);

        pid = new PIDController(Constants.ElevatorConstants.PID.kP, Constants.ElevatorConstants.PID.kI, Constants.ElevatorConstants.PID.kD);
        feedforward = new ElevatorFeedforward(Constants.ElevatorConstants.FeedForward.kS, Constants.ElevatorConstants.FeedForward.kG, Constants.ElevatorConstants.FeedForward.kV);

        elevatorState = ElevatorState.DOWN;

        climber = new Servo(Constants.ElevatorConstants.climberServoID);


    }

    //pidloop
    private double encoderPosition;
    private double goal; 
    @Override
    public void periodic() {
        loggers();
        encoderPosition = getEncoderPosition(); /*bottomSwitch.get() ? 0 : getEncoderPosition();*/
        //setEncoderPosition(encoderPosition);   

        climberSwitchHeight(climberSwitch());
    }

    public void pidControl()
    {
        setOutput(pid.calculate(encoderPosition, goal) + feedforward.calculate(encoderPosition, goal));
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
        leftSparkMax.set(output);
    }

    public void manualControl(double input) {
        if (input > 0 && getEncoderPosition() + input > Constants.ElevatorConstants.kMaxElevatorHeightMeters) {
            goal += input;
        } else if (input < 0 && getEncoderPosition() - input < Constants.ElevatorConstants.kMinElevatorHeightMeters) {
            goal -= input;
        }
    }

    public void setEncoderPosition(double position) {
        relativeEncoder.setPosition(position);
    }

    public double getEncoderPosition() {
        return relativeEncoder.getPosition();
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
        int offset = Constants.ElevatorConstants.atTargetOffset;
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

    public BooleanSupplier climberSwitch() {
        if (getEncoderPosition() <= Constants.ElevatorConstants.climberEncoderPosition) {
            return () -> true;
        } else if (getEncoderPosition() >= Constants.ElevatorConstants.climberEncoderPosition) {
            return () -> false;
        }
            return () -> false;
    }


    public void climberSwitchHeight(BooleanSupplier switchThing) {
        if (switchThing.getAsBoolean()) {
            climber.setAngle(Constants.ElevatorConstants.climberAngle1);
        } else if (switchThing.getAsBoolean() == false) {
            climber.setAngle(Constants.ElevatorConstants.climberAngle2);
        }
    }

/*LOGGERS*/

    private void loggers() {
        SmartDashboard.putNumber("elevator encoder position", getEncoderPosition());
        SmartDashboard.putNumber("elevator pid goal", goal);
        SmartDashboard.putString("elevatorstate", getElevatorState().toString());
        SmartDashboard.putNumber("elevator kp", Constants.ElevatorConstants.PID.kP);
        SmartDashboard.putNumber("elevator ki", Constants.ElevatorConstants.PID.kI);
        SmartDashboard.putNumber("elevator kd", Constants.ElevatorConstants.PID.kD);
    }

/*RUNNABLE ACTIONS FOR BUTTON BOX*/

    public Command goToStateCommand(ElevatorState state) {
        return this.runOnce(() -> setStateByIndex(state.index));
    }
}
