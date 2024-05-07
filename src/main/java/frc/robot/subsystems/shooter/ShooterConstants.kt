package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.minus
import frc.robot.RobotMap

object ShooterConstants {
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
///
	/**
	 * 1 degree should be the lowest possible angle.
	 * It should be 1 degree and not 0 so that it doesn't wrap to 360 by accident.
	 */
	val CANCODER_OFFSET = (-274).degrees

	val LOAD_ANGLE = 0.degrees
	val AMP_ANGLE = 0.degrees

	val RESTING_ANGLE = 223.5.degrees
	const val KEEP_PARALLEL_TO_FLOOR_OUTPUT = -0.0185
	val FLOOR_RELATIVE_OFFSET = RESTING_ANGLE minus 90.0.degrees

	val MIN_ANGLE = 0.0.degrees
	val MAX_ANGLE = 360.0.degrees

	val shooterAnglePIDGains = PIDGains(12.0, 0.5)
	val shooterMotorPIDGains = PIDGains(0.0, 0.0, 0.0)
}