package frc.robot.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveSubsystem
import kotlin.math.sign

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
		val lY = if (lJoyY() in -0.02..0.02) 0.0 else lJoyY()
		val lX = if (lJoyX() in -0.02..0.02) 0.0 else lJoyX()
		val rX = if (rJoyX() in -0.02..0.02) 0.0 else rJoyX()

		SwerveSubsystem.robotRelativeDrive(
			ChassisSpeeds(lX * lX * lX.sign * SwerveConstants.MAX_SPEED_MPS,
				lY * lY * lY.sign * SwerveConstants.MAX_SPEED_MPS,
				rX * rX * rX.sign * Math.PI))
	}

	override fun end(interrupted: Boolean) {
		SwerveSubsystem.setRotation(Rotation2d(0.0))
		SwerveSubsystem.setSpeed(0.0)
	}
}

class resetGyro() : Command() {
	init {
		name = "Reset gyro"
	}

	override fun initialize() {
		SwerveSubsystem.resetPigeon()

	}
}

class FieldRelativeDrive(val lJoyY: () -> Double, val lJoyX: () -> Double, val rJoyX: () -> Double) : Command() {
	init {
		addRequirements(SwerveSubsystem)
		name = "Field relative drive"
	}

	override fun execute() {
		val lY = if (lJoyY() in -0.02..0.02) 0.0 else lJoyY()
		val lX = if (lJoyX() in -0.02..0.02) 0.0 else lJoyX()
		val rX = if (rJoyX() in -0.02..0.02) 0.0 else rJoyX()

		SwerveSubsystem.fieldRelativeDrive(ChassisSpeeds(lX * lX * lX.sign * SwerveConstants.MAX_SPEED_MPS,
			lY * lY * lY.sign * SwerveConstants.MAX_SPEED_MPS,
			rX * rX * rX.sign * Math.PI))
	}

	override fun end(interrupted: Boolean) {
		SwerveSubsystem.setRotation(Rotation2d(0.0))
		SwerveSubsystem.setSpeed(0.0)
	}
}