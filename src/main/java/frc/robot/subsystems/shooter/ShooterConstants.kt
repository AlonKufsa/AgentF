package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.minus
import com.hamosad1657.lib.units.rpm
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.RobotMap

data class ShooterState(val angle: Rotation2d, val angularVelocity: AngularVelocity) {
	companion object {
		// --- Collection ---
		val COLLECT = ShooterState(172.degrees, 0.0.rpm)

		// --- Teleop Speaker ---
		val AT_SPEAKER = ShooterState(200.degrees, 2600.rpm)
		val REVERSE_AT_SPEAKER = ShooterState(270.degrees, 2600.rpm)
		val AT_PODIUM = ShooterState(175.0.degrees, 3500.rpm)
		var AT_STAGE = ShooterState(162.0.degrees, 4100.rpm)

		// --- Teleop Misc. ---
		val TO_AMP = ShooterState(5.degrees, 0.0.rpm)
		val EJECT = ShooterState(168.degrees, 1000.rpm)
	}
}


object ShooterConstants {

	/**
	 * 1 degree should be the lowest possible angle.
	 * It should be 1 degree and not 0 so that it doesn't wrap to 360 by accident.
	 */
	val CANCODER_OFFSET = (-274).degrees

	val RESTING_ANGLE = 223.5.degrees
	const val KEEP_PARALLEL_TO_FLOOR_OUTPUT = -0.0185
	val FLOOR_RELATIVE_OFFSET = RESTING_ANGLE minus 90.0.degrees

	val MIN_ANGLE = 0.0.degrees
	val MAX_ANGLE = 360.0.degrees

	val ANGLE_PID_GAINS = PIDGains(12.0, 0.5)
	val SHOOTING_PID_GAINS = PIDGains(0.0, 0.0, 0.0)

	val ANGLE_TOLERANCE = 1.degrees
	val VELOCITY_TOLERANCE = 50.rpm

	val ANGLE_FEEDBACK_CONFIGS = FeedbackConfigs().apply {
		FeedbackSensorSource = RemoteCANcoder
		FeedbackRemoteSensorID = RobotMap.ShooterMap.CANCODER_ID
	}
	val ANGLE_MOTION_MAGIC_CONFIGS = MotionMagicConfigs().apply {
		MotionMagicAcceleration = 4.0
		MotionMagicCruiseVelocity = 2.0
	}
	val ANGLE_CURRENT_LIMIT_CONFIGS = CurrentLimitsConfigs().apply {
		SupplyCurrentLimitEnable = true
		SupplyCurrentLimit = 30.0
	}

	val CANCODER_CONFIGS = CANcoderConfiguration().apply {
		MagnetSensor.MagnetOffset = CANCODER_OFFSET.rotations
		MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1
		MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive
	}
}