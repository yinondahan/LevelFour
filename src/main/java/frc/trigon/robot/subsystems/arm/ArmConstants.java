package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;

public class ArmConstants {
    private static final int ENCODER_ID = 1, MOTOR_ID = 1;
    private static final String MOTOR_NAME = "ArmMotor";
    static final CANcoder ENCODER = new CANcoder(ENCODER_ID);
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);

    static final boolean FOC_ENABLED = true;
    static final StatusSignal<Angle> ANGLE_STATUS_SIGNAL = ENCODER.getPosition();

    private static final double
            MAX_VELOCITY = 5,
            MAX_ACCELERATION = 3;
    static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_VELOCITY,
            MAX_ACCELERATION
    );

    private static final double
            P = 0,
            I = 0,
            D = 0,
            KS = 0,
            KV = 0,
            KA = 0;
    static final PIDController PID_CONTROLLER = new PIDController(P, I, D);
    static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(KS, KV, KA);

    static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2);

    static {
        configMotor();
        configEncoder();
    }

    private static void configMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = 1.5;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        MOTOR.applyConfiguration(config);

    }

    private static void configEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.MagnetOffset = 0;
        ENCODER.getConfigurator().apply(config);
        ANGLE_STATUS_SIGNAL.setUpdateFrequency(100);
        ENCODER.optimizeBusUtilization();
    }

    public enum ArmState {
        HIGH_STATE(Rotation2d.fromDegrees(167)),
        LOW_STATE(Rotation2d.fromDegrees(67)),
        REST(Rotation2d.fromDegrees(0));

        public final Rotation2d targetAngle;

        ArmState(Rotation2d targetAngle) {
            this.targetAngle = targetAngle;
        }
    }
}
