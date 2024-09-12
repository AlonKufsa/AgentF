package frc.robot.subsystems.swerve

import com.ctre.phoenix6.hardware.Pigeon2
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.SwerveMap
import frc.robot.subsystems.swerve.SwerveConstants as Constants

object SwerveSubsystem : SubsystemBase("Swerve subsystem") {
	private val frontRight = SwerveModule(
		SwerveMap.FrontRight.DRIVE_MOTOR_ID,
		SwerveMap.FrontRight.STEER_MOTOR_ID,
		SwerveMap.FrontRight.CANCODER_ID,
		"FrontRight",
		invertedDrive = true,
		invertedSteer = false
	)
	private val frontLeft = SwerveModule(
		SwerveMap.FrontLeft.DRIVE_MOTOR_ID,
		SwerveMap.FrontLeft.STEER_MOTOR_ID,
		SwerveMap.FrontLeft.CANCODER_ID,
		"FrontLeft",
		invertedDrive = true,
		invertedSteer = false
	)
	private val backLeft = SwerveModule(
		SwerveMap.BackLeft.DRIVE_MOTOR_ID,
		SwerveMap.BackLeft.STEER_MOTOR_ID,
		SwerveMap.BackLeft.CANCODER_ID,
		"BackLeft",
		invertedDrive = true,
		invertedSteer = false
	)
	private val backRight = SwerveModule(
		SwerveMap.BackRight.DRIVE_MOTOR_ID,
		SwerveMap.BackRight.STEER_MOTOR_ID,
		SwerveMap.BackRight.CANCODER_ID,
		"BackRight",
		invertedDrive = true,
		invertedSteer = false
	)

	/** FR, FL, BL, BR*/
	private val modules = listOf(frontRight, frontLeft, backLeft, backRight)

	/** Use externally only for testing */
	fun setModuleStates(moduleStates: ModuleStates) {
		frontRight.setModuleState(moduleStates.frontRight)
		frontLeft.setModuleState(moduleStates.frontLeft)
		backLeft.setModuleState(moduleStates.backLeft)
		backRight.setModuleState(moduleStates.backRight)
	}

	fun setRotation(rotation: Rotation2d) {
		for (module in modules) {
			module.setModuleRotation(rotation)
		}
	}

	fun setSpeed(speedMPS: Double) {
		for (module in modules) {
			module.setModuleSpeed(speedMPS)
		}
	}

	fun robotRelativeDrive(chassisSpeeds: ChassisSpeeds) {
		val moduleStates =
			SwerveKinematics.robotRelativeChassisSpeedsToModuleStates(chassisSpeeds, Constants.MAX_SPEED_MPS)

		setModuleStates(moduleStates)
	}

	// Gyro

	private val pigeon = Pigeon2(SwerveMap.PIGEON_2_ID, Constants.SWERVE_CANBUS)

	fun resetPigeon() {
		pigeon.reset()
	}


	fun fieldRelativeDrive(chassisSpeeds: ChassisSpeeds) {
		val moduleStates =
			SwerveKinematics.fieldRelativeChassisSpeedsToModuleStates(chassisSpeeds,
				Constants.MAX_SPEED_MPS,
				Rotation2d.fromDegrees(pigeon.angle))

		setModuleStates(moduleStates)
	}


	// Testing

	fun setSteerVoltage(voltage: Volts) {
		for (module in modules) {
			module.setSteerVoltage(voltage)
		}
	}

	fun setDriveVoltage(voltage: Volts) {
		for (module in modules) {
			module.setDriveVoltage(voltage)
		}
	}

	fun resetAllModules() {
		for (module in modules) {
			module.setModuleState(SwerveModuleState(0.0, Rotation2d(0.0)))
		}
	}

	fun povDrive(pov: String) {
		when (pov) {
			"Up" -> setModuleStates(SwerveKinematics.robotRelativeVelocityToModuleStates(Translation2d(0.0, 1.0)))
			"Down" -> setModuleStates(SwerveKinematics.robotRelativeVelocityToModuleStates(Translation2d(0.0, -1.0)))
			"Right" -> setModuleStates(SwerveKinematics.robotRelativeVelocityToModuleStates(Translation2d(1.0, 0.0)))
			"Left" -> setModuleStates(SwerveKinematics.robotRelativeVelocityToModuleStates(Translation2d(-1.0, 0.0)))
		}
	}

	// Logging
	override fun initSendable(builder: SendableBuilder) {
		for (module in modules) {
			module.addModuleInfo(builder)
		}
		builder.addDoubleProperty("Robot yaw deg", { pigeon.angle }, null)
	}
}