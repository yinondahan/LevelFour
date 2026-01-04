package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;

import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.simulation.SingleJointedArmSimulation;
import frc.trigon.lib.utilities.Conversions;
import frc.trigon.lib.utilities.mechanisms.SingleJointedArmMechanism2d;

public class ArmConstants {

    private static final int
            MOTOR_ID = 1,
            ENCODER_ID = 1;

    private static final String
            MOTOR_NAME = "ArmMotor",
            ENCODER_NAME = "ArmEncoder";

    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);
    static final CANcoder ENCODER = new CANcoder(ENCODER_ID, ENCODER_NAME);

    static final StatusSignal<Angle> ANGLE_SIGNAL = ENCODER.getPosition();

    static final double
            GEAR_RATIO = 42,
            ARM_LENGTH_METERS = 0.52,
            ARM_MASS_KG = 3.5,
            MOTOR_CURRENT_LIMIT = 50;

    static final Rotation2d
            MIN_ANGLE = Rotation2d.fromDegrees(0),
            MAX_ANGLE = Rotation2d.fromDegrees(360);

    private static final double ANGLE_ENCODER_GRAVITY_OFFSET = 0;

    static final double POSITION_OFFSET_FROM_GRAVITY_OFFSET =
            RobotHardwareStats.isSimulation()
                    ? -Conversions.degreesToRotations(90)
                    : -ANGLE_ENCODER_GRAVITY_OFFSET;

    private static final double
            MAX_VELOCITY = RobotHardwareStats.isSimulation() ? 2.46 : 0,
            MAX_ACCELERATION = RobotHardwareStats.isSimulation() ? 67.2 : 0;

    static final TrapezoidProfile.Constraints CONSTRAINTS =
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);

    private static final double
            P = RobotHardwareStats.isSimulation() ? 34 : 0,
            I = 0,
            D = RobotHardwareStats.isSimulation() ? 3 : 0;

    private static final double
            KS = RobotHardwareStats.isSimulation() ? 0.026 : 0,
            KV = RobotHardwareStats.isSimulation() ? 4.87 : 0,
            KA = RobotHardwareStats.isSimulation() ? 0.178 : 0,
            KG = RobotHardwareStats.isSimulation() ? 0.112 : 0;

    static final PIDController PID = new PIDController(P, I, D);
    static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(KS, KG, KV, KA);

    static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2);

    static final boolean FOC_ENABLED = true;

    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);

    static final SingleJointedArmSimulation SIMULATION =
            new SingleJointedArmSimulation(
                    GEARBOX,
                    GEAR_RATIO,
                    ARM_LENGTH_METERS,
                    ARM_MASS_KG,
                    MIN_ANGLE,
                    MAX_ANGLE,
                    true
            );

    static final SingleJointedArmMechanism2d MECHANISM =
            new SingleJointedArmMechanism2d(
                    "Arm",
                    ARM_LENGTH_METERS,
                    Color.kBlue
            );

    static {
        configureMotor();
        configureEncoder();
    }

    private static void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.RotorToSensorRatio = GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = ENCODER.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        config.Slot0.kG = KG;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = MOTOR_CURRENT_LIMIT;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);
    }

    private static void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_GRAVITY_OFFSET;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        ENCODER.getConfigurator().apply(config);
        ANGLE_SIGNAL.setUpdateFrequency(100);
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