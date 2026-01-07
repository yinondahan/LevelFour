package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderSignal;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SingleJointedArmSimulation;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.swerve.swervemodule.SwerveModuleConstants;
import org.littletonrobotics.junction.Logger;

public class Arm extends MotorSubsystem {
    private final TalonFXMotor motor = ArmConstants.MOTOR;
    private final CANcoderEncoder encoder = ArmConstants.ENCODER;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ArmConstants.FOC_ENABLED);
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(
            0,
            ArmConstants.MAX_VELOCITY,
            ArmConstants.MAX_ACCELERATION,
            ArmConstants.MAX_JERK
    ).withEnableFOC(ArmConstants.FOC_ENABLED);
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    public Arm() {
        setName(ArmConstants.MECHANISM_NAME);
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Arm")
                .angularPosition(Units.Rotations.of(getCurrentAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        ArmConstants.MECHANISM.update(
                getCurrentAngle(),
                Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
        System.out.println(targetVoltage);
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return ArmConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    @Override
    public void updatePeriodically() {
        motor.update();
        encoder.update();
        Logger.recordOutput("Arm/CurrentPositionDegrees", getCurrentAngle().getDegrees());
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    public boolean atTargetAngle() {
        return atAngle(targetAngle);
    }

    public boolean atAngle(Rotation2d angle) {
        return Math.abs(
                angle.minus(getCurrentAngle()).getDegrees()
        ) < ArmConstants.TOLERANCE.getDegrees();
    }

    public void setTargetState(ArmConstants.ArmState targetState) {
        setTargetAngle(targetState.targetAngle);
    }

    public void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle = targetAngle;
        motor.setControl(
                positionRequest.withPosition(targetAngle.getRotations())
        );
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(
                encoder.getSignal(CANcoderSignal.POSITION)
        );
    }
}