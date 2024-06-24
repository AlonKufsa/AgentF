package frc.robot.subsystems.swerve

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue.Unsigned_0To1
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder
import com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.meters
import com.hamosad1657.lib.units.rps
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.wpilibj.DriverStation

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

	var frontRight: SwerveModuleState = SwerveModuleState()
		private set
	var frontLeft: SwerveModuleState = SwerveModuleState()
		private set
	var backLeft: SwerveModuleState = SwerveModuleState()
		private set
	var backRight: SwerveModuleState = SwerveModuleState()
		private set

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
	}

	fun setFrontRight(state: SwerveModuleState) {frontRight = state}
	fun setFrontLeft(state: SwerveModuleState) {frontLeft = state}
	fun setBackLeft(state: SwerveModuleState) {backLeft = state}
	fun setBackRight(state: SwerveModuleState) {backRight = state}
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
	const val FRONT_RIGHT_OFFSET_DEG = 0.0
	const val FRONT_LEFT_OFFSET_DEG = 0.0
	const val BOTTOM_LEFT_OFFSET_DEG = 0.0
	const val BOTTOM_RIGHT_OFFSET_DEG = 0.0

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
				"FrontRight" -> FRONT_RIGHT_OFFSET_DEG
				"FrontLeft" -> FRONT_LEFT_OFFSET_DEG
				"BottomLeft" -> BOTTOM_LEFT_OFFSET_DEG
				"BottomRight" -> BOTTOM_RIGHT_OFFSET_DEG
				else -> 0.0.also { DriverStation.reportError("Invalid swerve module name: $moduleName", false) }
			}

			SensorDirection = CounterClockwise_Positive
			AbsoluteSensorRange = Unsigned_0To1
		}
	}

}