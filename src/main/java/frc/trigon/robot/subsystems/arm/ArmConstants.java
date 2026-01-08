package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderSignal;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SingleJointedArmSimulation;
import frc.trigon.lib.utilities.mechanisms.SingleJointedArmMechanism2d;

public class ArmConstants {
    private static final int
            MOTOR_ID = 9,
            ENCODER_ID = 12;
    private static final String
            MOTOR_NAME = "ArmMotor",
            ENCODER_NAME = "ArmEncoder";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);
    static final CANcoderEncoder ENCODER = new CANcoderEncoder(ENCODER_ID, ENCODER_NAME);

    static final double GEAR_RATIO = 42;
    static final double
            MAX_VELOCITY = RobotHardwareStats.isSimulation() ? 1 : 0,
            MAX_ACCELERATION = RobotHardwareStats.isSimulation() ? 1 : 0,
            MAX_JERK = MAX_ACCELERATION * 10;

    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final double
            LENGTH_METERS = 0.52,
            MASS_KILOGRAMS = 3.5;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(0),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(360);
    private static final boolean SHOULD_ARM_SIMULATE_GRAVITY = true;
    static final SingleJointedArmSimulation ARM_SIMULATION = new SingleJointedArmSimulation(
            GEARBOX,
            GEAR_RATIO,
            LENGTH_METERS,
            MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SHOULD_ARM_SIMULATE_GRAVITY
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Seconds),
            Units.Volts.of(4),
            Units.Second.of(1000)
    );

    static final String MECHANISM_NAME = "Arm";
    static final SingleJointedArmMechanism2d MECHANISM = new SingleJointedArmMechanism2d(
            MECHANISM_NAME,
            LENGTH_METERS,
            Color.kLightSeaGreen
    );

    static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2);
    static final boolean FOC_ENABLED = false;

    static {
        configureEncoder();
        configureMotor();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.RotorToSensorRatio = GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = ENCODER.getID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 3 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0.5 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.0215 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 5.43 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.2785 : 0;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MAX_JERK;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 50;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(ARM_SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = 0;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        ENCODER.applyConfiguration(config);
        ENCODER.setSimulationInputsFromTalonFX(MOTOR);

        ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
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