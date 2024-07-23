package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.RobotMap.SwerveMap

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

	/** Use externally only for testing */
	fun setModuleStates(moduleStates: ModuleStates) {
		frontRight.setModuleState(moduleStates.frontRight)
		frontLeft.setModuleState(moduleStates.frontLeft)
		backLeft.setModuleState(moduleStates.backLeft)
		backRight.setModuleState(moduleStates.backRight)
	}

	fun setRotation() {
		frontRight.setAngleSetpoint()
		frontLeft.setAngleSetpoint()
		backLeft.setAngleSetpoint()
		backRight.setAngleSetpoint()
	}

	fun robotRelativeDrive(chassisSpeeds: ChassisSpeeds) {
		val moduleStates = SwerveKinematics.robotRelativeChassisSpeedsToModuleStates(chassisSpeeds)

		setModuleStates(moduleStates)
	}

	fun testMotor() {
		frontRight.setSteerVoltage(2.0)
		frontLeft.setSteerVoltage(2.0)
		backLeft.setSteerVoltage(2.0)
		backRight.setSteerVoltage(2.0)
	}

	// Logging
	override fun initSendable(builder: SendableBuilder) {
		frontRight.addModuleInfo(builder)
		frontLeft.addModuleInfo(builder)
		backLeft.addModuleInfo(builder)
		backRight.addModuleInfo(builder)
	}

	// SysID
	private fun voltageDrive(voltage: Measure<Voltage>) {
		val modules = listOf(frontRight, frontLeft, backLeft, backRight)
		for (module in modules) {
			module.voltageDrive(voltage)
		}
	}

	private fun logMotors(log: SysIdRoutineLog) {
		val modules = listOf(frontRight, frontLeft, backLeft, backRight)
		for (module in modules) {
			log.motor(module.moduleName).angularPosition(Degrees.of(module.currentRotationNotWrapped.degrees))
				.angularVelocity(DegreesPerSecond.of(module.currentAngularVelocity.asDegPs))
				.voltage(Volt.of(module.currentAppliedVoltage))

		}
	}

	val routine = SysIdRoutine(
		SysIdRoutine.Config(),
		SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this)
	)
}