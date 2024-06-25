package frc.robot.subsystems.swerve

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.SwerveMap

object SwerveSubsystem : SubsystemBase("Swerve subsystem") {
	private val frontRight = SwerveModule(
		SwerveMap.FrontRight.DRIVE_MOTOR_ID,
		SwerveMap.FrontRight.STEER_MOTOR_ID,
		SwerveMap.FrontRight.CANCODER_ID,
		"FrontRight"
	)
	private val frontLeft = SwerveModule(
		SwerveMap.FrontLeft.DRIVE_MOTOR_ID,
		SwerveMap.FrontLeft.STEER_MOTOR_ID,
		SwerveMap.FrontLeft.CANCODER_ID,
		"FrontLeft"
	)
	private val backLeft = SwerveModule(
		SwerveMap.BackLeft.DRIVE_MOTOR_ID,
		SwerveMap.BackLeft.STEER_MOTOR_ID,
		SwerveMap.BackLeft.CANCODER_ID,
		"BackLeft"
	)
	private val backRight = SwerveModule(
		SwerveMap.BackRight.DRIVE_MOTOR_ID,
		SwerveMap.BackRight.STEER_MOTOR_ID,
		SwerveMap.BackRight.CANCODER_ID,
		"BackRight"
	)

	private fun setModuleStates(moduleStates: ModuleStates) {
		frontRight.setModuleState(moduleStates.frontRight)
		frontLeft.setModuleState(moduleStates.frontLeft)
		backLeft.setModuleState(moduleStates.backLeft)
		backRight.setModuleState(moduleStates.backRight)
	}

	fun robotRelativeDrive(chassisSpeeds: ChassisSpeeds) {
		val moduleStates = SwerveKinematics.robotRelativeChassisSpeedsToModuleStates(chassisSpeeds)

		setModuleStates(moduleStates)
	}


	override fun initSendable(builder: SendableBuilder) {
		frontRight.addModuleInfo(builder)
		frontLeft.addModuleInfo(builder)
		backLeft.addModuleInfo(builder)
		backRight.addModuleInfo(builder)
	}
}