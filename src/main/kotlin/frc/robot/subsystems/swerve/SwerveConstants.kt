package frc.robot.subsystems.swerve

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue.Unsigned_0To1
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder
import com.ctre.phoenix6.signals.SensorDirectionValue.Clockwise_Positive
import com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.meters
import com.hamosad1657.lib.units.rps
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DriverStation
import kotlin.math.max

class ModuleStates() {

	constructor(
		frontRightState: SwerveModuleState,
		frontLeftState: SwerveModuleState,
		backLeftState: SwerveModuleState,
		backRightState: SwerveModuleState
		): this() {
		frontRight = frontRightState
		frontLeft = frontLeftState
		backLeft = backLeftState
		backRight = backRightState
	}

	/** The maximum speed modules are allowed to receive. */
	var maxAllowedModuleSpeed = 0.0

	var frontRight: SwerveModuleState = SwerveModuleState()
	var frontLeft: SwerveModuleState = SwerveModuleState()
	var backLeft: SwerveModuleState = SwerveModuleState()
	var backRight: SwerveModuleState = SwerveModuleState()

	fun setStates(
		frontRightState: SwerveModuleState,
		frontLeftState: SwerveModuleState,
		backLeftState: SwerveModuleState,
		backRightState: SwerveModuleState
	) {
		frontRight = frontRightState
		frontLeft = frontLeftState
		backLeft = backLeftState
		backRight = backRightState
		if (max(max(frontRightState.speedMetersPerSecond, frontLeftState.speedMetersPerSecond), max(backRightState.speedMetersPerSecond, backLeftState.speedMetersPerSecond)) > maxAllowedModuleSpeed && maxAllowedModuleSpeed != 0.0) {
			val highestModuleSpeed = max(max(frontRightState.speedMetersPerSecond, frontLeftState.speedMetersPerSecond), max(backRightState.speedMetersPerSecond, backLeftState.speedMetersPerSecond))
			val factor = maxAllowedModuleSpeed / highestModuleSpeed

			frontRight.speedMetersPerSecond *= factor
			frontLeft.speedMetersPerSecond *= factor
			backLeft.speedMetersPerSecond *= factor
			backRight.speedMetersPerSecond *= factor
		}
	}
}

object SwerveConstants {
	// Canbus network swerve is connected to
	const val SWERVE_CANBUS = "SwerveBus"


	// TODO: Tune PID
	// Drive pid and ff values
	val DRIVE_PID_GAINS = PIDGains()
	const val DRIVE_KA = 0.0
	const val DRIVE_KV = 0.0
	const val DRIVE_KS = 0.0

	const val DRIVE_MOTION_MAGIC_ACCELERATION = 0.0
	const val DRIVE_MOTION_MAGIC_CRUISE_VELOCITY = 0.0

	// Steer pid and ff values
	val STEER_PID_GAINS = PIDGains()
	const val STEER_KA = 0.0
	const val STEER_KV = 0.0
	const val STEER_KS = 0.0

	const val STEER_MOTION_MAGIC_ACCELERATION = 0.0
	const val STEER_MOTION_MAGIC_CRUISE_VELOCITY = 0.0

	const val WHEEL_CIRCUMFERENCE_METERS = 0.0

	/** Theoretical free speed (m/s) at 12v applied output. */
	const val MAX_SPEED_MPS = 9.0 // 9.46 according to CTRE ?

	/** Theoretical free rotation speed (rotations/s) at 12v applied output. (how fast it can steer) */
	val MAX_ANGULAR_VELOCITY = 2.0.rps

	/** The distance from the center of the chassis to a center of a module. */
	val DRIVEBASE_RADIUS = 0.417405.meters

	/** The amount of rotations the engine does for every rotation of the wheel */
	const val DRIVE_TRANSMISSION = 6.746031746031747

	// TODO: Find values
	// The CANCoder offsets for each module
	val FRONT_RIGHT_OFFSET = Rotation2d.fromDegrees(-268.066406)
	val FRONT_LEFT_OFFSET = Rotation2d.fromDegrees(-222.539062)
	val BACK_LEFT_OFFSET = Rotation2d.fromDegrees(95.537)
	val BACK_RIGHT_OFFSET = Rotation2d.fromDegrees(-185.888672)

	// Assuming that at 0 degrees a positive output will lead to a positive speed
	val DRIVE_MOTOR_CONFIGS: TalonFXConfiguration = TalonFXConfiguration().apply {
		// Current limits
		CurrentLimits.SupplyCurrentLimit = 45.0
		CurrentLimits.SupplyCurrentLimitEnable = true
		CurrentLimits.StatorCurrentLimitEnable = false

		// Limits the speed in which the motors voltage consumption can change
		ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25

		// PID and FF
		with(Slot0) {
			// PID
			kP = DRIVE_PID_GAINS.kP
			kI = DRIVE_PID_GAINS.kI
			kD = DRIVE_PID_GAINS.kD

			// FF
			kA = DRIVE_KA
			kV = DRIVE_KV
			kS = DRIVE_KS
		}

		// MotionMagic configs
		with(MotionMagic) {
			MotionMagicAcceleration = DRIVE_MOTION_MAGIC_ACCELERATION
			MotionMagicCruiseVelocity = DRIVE_MOTION_MAGIC_CRUISE_VELOCITY
		}

	}

	fun steerMotorConfigs(canCoderID: Int): TalonFXConfiguration = TalonFXConfiguration().apply {
		CurrentLimits.SupplyCurrentLimit = 20.0
		CurrentLimits.SupplyCurrentLimitEnable = true
		CurrentLimits.StatorCurrentLimitEnable = false

		Feedback.FeedbackRemoteSensorID = canCoderID
		Feedback.FeedbackSensorSource = RemoteCANcoder

		with(Slot0) {
			// PID
			kP = STEER_PID_GAINS.kP
			kI = STEER_PID_GAINS.kI
			kD = STEER_PID_GAINS.kD

			// FF
			kA = STEER_KA
			kV = STEER_KV
			kS = STEER_KS
		}

		// Motion Magic configs
		with(MotionMagic) {
			MotionMagicAcceleration = STEER_MOTION_MAGIC_ACCELERATION
			MotionMagicCruiseVelocity = STEER_MOTION_MAGIC_CRUISE_VELOCITY
		}

	}

	fun canCoderConfigs(moduleName: String): CANcoderConfiguration = CANcoderConfiguration().apply {
		with(MagnetSensor) {
			// The offset added to the CANCoder for it to measure correctly for a wheel pointing right to be 0 degrees
			MagnetOffset = when (moduleName) {
				"FrontRight" -> FRONT_RIGHT_OFFSET.rotations
				"FrontLeft" -> FRONT_LEFT_OFFSET.rotations
				"BackLeft" -> BACK_LEFT_OFFSET.rotations
				"BackRight" -> BACK_RIGHT_OFFSET.rotations
				else -> 0.0.also { DriverStation.reportError("Invalid swerve module name: $moduleName", false) }
			}

			SensorDirection = CounterClockwise_Positive
			AbsoluteSensorRange = Unsigned_0To1
		}
	}

}