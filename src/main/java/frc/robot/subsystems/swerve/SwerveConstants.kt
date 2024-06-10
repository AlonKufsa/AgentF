package frc.robot.subsystems.swerve

import com.hamosad1657.lib.math.PIDGains
import kotlin.math.PI

object SwerveConstants {
	const val SWERVE_CANBUS = "SwerveBus"

	val DRIVE_PID_GAINS = PIDGains()
	val STEER_PID_GAINS = PIDGains()

	const val WHEEL_CIRCUMFERENCE_METERS = 0.0
	const val WHEEL_RADIUS_METERS = WHEEL_CIRCUMFERENCE_METERS / (2 * PI)

	const val TOP_RIGHT_OFFSET_DEG = 0.0
	const val TOP_LEFT_OFFSET_DEG = 0.0
	const val BOTTOM_LEFT_OFFSET_DEG = 0.0
	const val BOTTOM_RIGHT_OFFSET_DEG = 0.0

}