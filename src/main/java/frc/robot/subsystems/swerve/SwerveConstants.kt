package frc.robot.subsystems.swerve

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.meters
import com.hamosad1657.lib.units.rps
import kotlin.math.PI

object SwerveConstants {
	const val SWERVE_CANBUS = "SwerveBus"

	val DRIVE_PID_GAINS = PIDGains()
	val STEER_PID_GAINS = PIDGains()

	const val WHEEL_CIRCUMFERENCE_METERS = 0.0
	const val WHEEL_RADIUS_METERS = WHEEL_CIRCUMFERENCE_METERS / (2 * PI)

	// Theoretical free speed (m/s) at 12v applied output.
	const val MAX_SPEED_MPS = 9.0 // 9.46 according to CTRE ?

	// Theoretical free rotation speed (rotations/s) at 12v applied output. (how fast it can steer)
	val MAX_ANGULAR_VELOCITY = 2.0.rps

	// The distance from the center of the chassis to a center of a module.
	private val DRIVEBASE_RADIUS = 0.417405.meters

	const val TOP_RIGHT_OFFSET_DEG = 0.0
	const val TOP_LEFT_OFFSET_DEG = 0.0
	const val BOTTOM_LEFT_OFFSET_DEG = 0.0
	const val BOTTOM_RIGHT_OFFSET_DEG = 0.0

	val DRIVE_MOTOR_CONFIGS: TalonFXConfiguration = TalonFXConfiguration().apply {
		CurrentLimits.SupplyCurrentLimit = 45.0
		CurrentLimits.SupplyCurrentLimitEnable = true
		CurrentLimits.StatorCurrentLimitEnable = true

		ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25
	}

	fun steerMotorFeedbackConfigs(canCoderID: Int, moduleName: String): FeedbackConfigs = FeedbackConfigs().apply {
		FeedbackRemoteSensorID = canCoderID
		FeedbackSensorSource = RemoteCANcoder
		FeedbackRotorOffset = when (moduleName) {
			"TopRight" -> TOP_RIGHT_OFFSET_DEG
			"TopLeft" -> TOP_LEFT_OFFSET_DEG
			"BottomLeft" -> BOTTOM_LEFT_OFFSET_DEG
			"BottomRight" -> BOTTOM_RIGHT_OFFSET_DEG
			else -> 0.0
		}
	}

}