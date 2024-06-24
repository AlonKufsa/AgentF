package frc.robot.subsystems.swerve

import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.SwerveMap

object SwerveSubsystem : SubsystemBase("Swerve subsystem") {
	val frontRight = SwerveModule(
		SwerveMap.FrontRight.DRIVE_MOTOR_ID,
		SwerveMap.FrontRight.STEER_MOTOR_ID,
		SwerveMap.FrontRight.CANCODER_ID,
		"FrontRight"
	)
	val frontLeft = SwerveModule(
		SwerveMap.FrontLeft.DRIVE_MOTOR_ID,
		SwerveMap.FrontLeft.STEER_MOTOR_ID,
		SwerveMap.FrontLeft.CANCODER_ID,
		"FrontRight"
	)
	val backLeft = SwerveModule(
		SwerveMap.BackLeft.DRIVE_MOTOR_ID,
		SwerveMap.BackLeft.STEER_MOTOR_ID,
		SwerveMap.BackLeft.CANCODER_ID,
		"FrontRight"
	)
	val backRight = SwerveModule(
		SwerveMap.BackRight.DRIVE_MOTOR_ID,
		SwerveMap.BackRight.STEER_MOTOR_ID,
		SwerveMap.BackRight.CANCODER_ID,
		"FrontRight"
	)

	private fun setModuleStates(moduleStates: ModuleStates) {
		frontRight.setModuleState(moduleStates.frontRight)
		frontLeft.setModuleState(moduleStates.frontLeft)
		backLeft.setModuleState(moduleStates.backLeft)
		backRight.setModuleState(moduleStates.backRight)
	}

	override fun initSendable(builder: SendableBuilder?) {
		SmartDashboard.putData(frontRight)
		SmartDashboard.putData(frontLeft)
		SmartDashboard.putData(backLeft)
		SmartDashboard.putData(backRight)
	}
}