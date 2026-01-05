package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderSignal;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;


public class Arm extends SubsystemBase {
    private final TalonFXMotor motor = ArmConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ArmConstants.FOC_ENABLED);
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(0, ArmConstants.MAX_VELOCITY, ArmConstants.MAX_ACCELERATION, ArmConstants.MAX_JERK).withEnableFOC(ArmConstants.FOC_ENABLED);
    private final CANcoderEncoder encoder = ArmConstants.ENCODER;
    private final TrapezoidProfile profile = new TrapezoidProfile(ArmConstants.CONSTRAINTS);
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
    private TrapezoidProfile.State initialState = new TrapezoidProfile.State();
    private final Timer profileTimer = new Timer();

    public Arm() {
    }

    public boolean atState(ArmConstants.ArmState targetState) {
        Rotation2d currentAngle = getCurrentAngle();
        Rotation2d targetAngle = targetState.targetAngle;

        return Math.abs(
                currentAngle.minus(targetAngle).getRotations()
        ) <= ArmConstants.TOLERANCE.getRotations();
    }

    void followSetpoint(Rotation2d setpoint) {
        setTargetVoltage(pidOutput + ffOutput);
    }

    void setTargetAngle(Rotation2d targetAngle) {
        final Rotation2d rotations = Rotation2d.fromRotations(targetAngle.getDegrees());
        motor.setControl(positionRequest.withPosition());
    }

    void stop() {
        motor.stopMotor();
    }

    void initializeMotionProfile(Rotation2d targetPosition) {
        initialState = new TrapezoidProfile.State(
                getCurrentAngle().getRotations(),
                encoder.getSignal(CANcoderSignal.VELOCITY)
        );

        goalState = new TrapezoidProfile.State(targetPosition.getRotations(), 0);

        profileTimer.restart();
    }

    void followMotionProfile() {
        final TrapezoidProfile.State setpoint = calculateSetpoint();
        final Rotation2d setpointAsRotation = Rotation2d.fromRotations(setpoint.position);
        followSetpoint(setpointAsRotation);
    }

    private TrapezoidProfile.State calculateSetpoint() {
        return profile.calculate(profileTimer.get(), initialState, goalState);
    }

    private double calculatePIDOutput(Rotation2d targetAngle) {
        return pidController.calculate(getCurrentAngle().getRotations(), targetAngle.getRotations());
    }

    private double calculateFeedForward() {
        double position = getCurrentAngle().getRotations();
        double velocity = 0;
        return feedforward.calculate(position, velocity);
    }

    private Rotation2d getCurrentAngle() {
        final double rotations = ArmConstants.ENCODER.getSignal(CANcoderSignal.POSITION);
        return Rotation2d.fromRotations(rotations);
    }

    private void setTargetVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }
}
