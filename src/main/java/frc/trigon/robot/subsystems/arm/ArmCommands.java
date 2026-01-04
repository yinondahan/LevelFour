package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;

public class ArmCommands {
    public static Command getSetTargetAngleCommand(ArmConstants.ArmState targetState) {
        return new FunctionalCommand(
                () -> RobotContainer.ARM.initializeMotionProfile(targetState.targetAngle),
                RobotContainer.ARM::followMotionProfile,
                (Interrupted) -> RobotContainer.ARM.stop(),
                () -> false,
                RobotContainer.ARM
        );
    }
}