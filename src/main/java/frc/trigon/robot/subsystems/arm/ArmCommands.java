package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.ExecuteEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;
import java.util.function.Supplier;

public class ArmCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetAngleDegrees) -> RobotContainer.ARM.setTargetAngle(Rotation2d.fromDegrees(targetAngleDegrees)),
                false,
                Set.of(RobotContainer.ARM),
                "Debugging/TargetArmAngleDegrees"
        );
    }

    public static Command getSetTargetStateCommand(ArmConstants.ArmState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.ARM.setTargetState(targetState),
                RobotContainer.ARM::stop,
                RobotContainer.ARM
        );
    }

    public static Command getSetTargetAngleCommand(Supplier<Rotation2d> targetAngle) {
        return new ExecuteEndCommand(
                () -> RobotContainer.ARM.setTargetAngle(targetAngle.get()),
                RobotContainer.ARM::stop,
                RobotContainer.ARM
        );
    }
}
