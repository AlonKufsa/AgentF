package frc.robot.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.swerve.SwerveSubsystem

fun SwerveSubsystem.setSwerveRotation(rotation: () -> Rotation2d):Command {
	return run({ setRotation(rotation()) })
}