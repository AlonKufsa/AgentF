package frc.robot.subsystems.swerve

import com.hamosad1657.lib.units.Volts
import edu.wpi.first.math.geometry.Rotation2d
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

	// Logging
	override fun initSendable(builder: SendableBuilder) {
		for (module in modules) {
			module.addModuleInfo(builder)
		}
	}
}