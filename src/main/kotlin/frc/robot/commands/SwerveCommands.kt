package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveKinematics
import frc.robot.subsystems.swerve.SwerveSubsystem
import frc.robot.subsystems.vision.NoteVision
import frc.robot.subsystems.vision.VisionConstants
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

class AssistedIntake(val r2: () -> Double, val l2: () -> Double) : Command() {
	init {
		name = "Assisted intake"
		addRequirements(SwerveSubsystem)
	}

	val pidController = PIDController(
		VisionConstants.assistedIntakePIDGains.kP,
		VisionConstants.assistedIntakePIDGains.kI,
		VisionConstants.assistedIntakePIDGains.kD)

	override fun execute() {
		if (NoteVision.hasTargets) {
			val target = NoteVision.lastResult.bestTarget
			val output = pidController.calculate(-target.yaw)

			val moduleStates = SwerveKinematics.robotRelativeChassisSpeedsToModuleStates(
				if (r2() < -0.5 && l2() < -0.5) ChassisSpeeds(0.0, 0.0, output)
				else if (r2() > -0.5 && l2() < -0.5) ChassisSpeeds(0.0, r2() + 0.5, output)
				else if (l2() > -0.5 && r2() < -0.5) ChassisSpeeds(0.0, -l2() - 0.5, output)
				else ChassisSpeeds(0.0, 0.0, output),
				SwerveConstants.MAX_SPEED_MPS
			)

			SwerveSubsystem.setModuleStates(moduleStates)
		} else {
			pidController.reset()
			SwerveSubsystem.resetAllModules()
		}
	}
}