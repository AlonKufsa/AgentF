package frc.robot.subsystems.leds

import com.hamosad1657.lib.units.Seconds

enum class LedMode {
	DISABLED,
	INTAKE,
	SHOOTING,
	MANUAL_ANGLE_CONTROL,
	DEFAULT,
	ACTION_FINISHED_SUCCESSFULLY
}

object LedConstants {
	const val STRIP_LENGTH = 24
	const val BLINK_TIME: Seconds = 0.1
	const val ACTION_FINISHED_DURATION: Seconds = 1.0
}