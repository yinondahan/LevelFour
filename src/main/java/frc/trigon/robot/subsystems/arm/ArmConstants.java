package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.simulation.SingleJointedArmSimulation;
import frc.trigon.lib.utilities.Conversions;
import frc.trigon.lib.utilities.mechanisms.SingleJointedArmMechanism2d;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderSignal;

public class ArmConstants {
    private static final int
            MOTOR_ID = 1,// motor's id
            ENCODER_ID = 1;// encoder's id
    private static final String
            MOTOR_NAME = "ArmMotor",//motor's name
            ENCODER_NAME = "ArmEncoder";// encoder's name
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);
    static final CANcoderEncoder ENCODER = new CANcoderEncoder(ENCODER_ID, ENCODER_NAME);
    static final double GEAR_RATIO = 42;//ill take this as an example when the motor spins 42 times the arm rotates once

    static final double
            LENGTH_METERS = 0.52,
            MASS_KILOGRAMS = 3.5;
    private static final boolean SHOULD_ARM_SIMULATE_GRAVITY = true;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);//creates a motor model for simulation that representing a real motor
    static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(0),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(360);
    private static final SingleJointedArmSimulation ARM_SIMULATION = new SingleJointedArmSimulation(
            GEARBOX,
            GEAR_RATIO,
            LENGTH_METERS,
            MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SHOULD_ARM_SIMULATE_GRAVITY
    );

    private static final String MECHANISM_NAME = "Arm";
    static final SingleJointedArmMechanism2d MECHANISM =
            new SingleJointedArmMechanism2d(
                    MECHANISM_NAME,
                    LENGTH_METERS,
                    Color.kBlue
            );//defining the mechanism of the arm in the simulation

    static final double
            MAX_VELOCITY = RobotHardwareStats.isSimulation() ? 2.46 : 0, //same here
            MAX_ACCELERATION = RobotHardwareStats.isSimulation() ? 67.2 : 0, //same here
            MAX_JERK = MAX_ACCELERATION * 10;
    static final TrapezoidProfile.Constraints CONSTRAINTS =
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION); //define the constraints
    static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2);
    private static final double ANGLE_ENCODER_GRAVITY_OFFSET = 0;
    static final double POSITION_OFFSET_FROM_GRAVITY_OFFSET =
            RobotHardwareStats.isSimulation()
                    ? -Conversions.degreesToRotations(90) // "?" means "if it's true" then that the value of the variable
                    : -ANGLE_ENCODER_GRAVITY_OFFSET; // ":" means "if it's false"
    static final boolean FOC_ENABLED = true;//who doesn't know wot de FOC is this?

    static {
        configureEncoder();
        configureMotor();
    }

    private static void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.RotorToSensorRatio = GEAR_RATIO; //configuring the gear ratio
        config.Feedback.FeedbackRemoteSensorID = ENCODER.getID();// Tells the motor to read from this encoder
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;// the source of the encoder

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 34 : 0; // slot is a different settings for the same motor that you can change in the middle of a run
        config.Slot0.kI = 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 3 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.026 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 4.87 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.178 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.112 : 0;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Automatically adds power to hold the arm up against gravity
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign; // adds a little extra power in the right direction so the arm moves smoothly

        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;//setting the max velocity for the motion magic // motion magic: moves the arm smoothly to a position without shaking
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;//same here but with acceleration
        config.MotionMagic.MotionMagicJerk = MAX_JERK; // makes the arm move smoothly

        config.CurrentLimits.StatorCurrentLimitEnable = true; //turns on the current limit to protect the motor
        config.CurrentLimits.StatorCurrentLimit = 50;//sets the current limit

        MOTOR.applyConfiguration(config);//applying all the motor configuration
        MOTOR.setPhysicsSimulation(ARM_SIMULATION); // Makes the motor act like the arm is moving in simulation
    }

    private static void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_GRAVITY_OFFSET;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        ENCODER.applyConfiguration(config);
        ENCODER.setSimulationInputsFromTalonFX(MOTOR);//makes the encoder follow the motor in the simulation

        ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);//updates the encoder position and speed every 100ms
    }

    public enum ArmState {
        HIGH(Rotation2d.fromDegrees(167)),
        LOW(Rotation2d.fromDegrees(67)),
        REST(Rotation2d.fromDegrees(0));

        public final Rotation2d targetAngle;

        ArmState(Rotation2d angle) {
            this.targetAngle = angle;
        }
    }
}