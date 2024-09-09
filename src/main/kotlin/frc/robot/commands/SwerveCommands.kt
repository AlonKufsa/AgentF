package frc.robot.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveKinematics
import frc.robot.subsystems.swerve.SwerveSubsystem

fun SwerveSubsystem.setSwerveRotation(rotation: () -> Rotation2d): Command {
	return run { setRotation(rotation()) }
}

fun SwerveSubsystem.setSwerveSpeedMPS(speed: () -> Double): Command {
	return run { setSpeed(speed()) }
}

class RobotRelativeSwerveDrive(val lJoyY: () -> Double, val lJoyX: () -> Double, val rJoyX: () -> Double) :
	Command() {
	init {
		name = "Robot relative swerve drive"
		addRequirements(SwerveSubsystem)
	}

	override fun execute() {
		val lY = if (lJoyY() in -0.2..0.2) 0.0 else lJoyY()
		val lX = if (lJoyX() in -0.2..0.2) 0.0 else lJoyX()
		val rX = if (rJoyX() in -0.2..0.2) 0.0 else rJoyX()

		val moduleStates = SwerveKinematics.robotRelativeChassisSpeedsToModuleStates(
			ChassisSpeeds(lX * SwerveConstants.MAX_SPEED_MPS,
				lY * SwerveConstants.MAX_SPEED_MPS,
				rX * Math.PI), SwerveConstants.MAX_SPEED_MPS)

		SwerveSubsystem.setModuleStates(moduleStates)
	}

	override fun end(interrupted: Boolean) {
		SwerveSubsystem.setRotation(Rotation2d(0.0))
		SwerveSubsystem.setSpeed(0.0)
	}
}