package frc.robot

object RobotMap {
	object ShooterMap {
		const val SHOOTER_ANGLE_MOTOR_ID = 4
		const val SHOOTER_TOP_SHOOTING_MOTOR_ID = 23
		const val SHOOTER_BOTTOM_SHOOTING_MOTOR_ID = 24
		const val CANCODER_ID = 3
		const val MIN_SWITCH_CHANNEL = 0
		const val MAX_SWITCH_CHANNEL = 1
	}

	object IntakeMap {
		const val BOTTOM_MOTOR_ID: Int = 20
		const val TOP_MOTOR_ID: Int = 21
	}

	object LoaderMap {
		const val MOTOR_ID = 22
		const val BEAMBREAK_CHANNEL = 0
	}

	object ClimbingMap {
		const val LEFT_MAIN_MOTOR_ID = 25
		const val LEFT_SECONDARY_MOTOR_ID = 26
		const val RIGHT_MAIN_MOTOR_ID = 27
		const val RIGHT_SECONDARY_MOTOR_ID = 28
	}

	object LedMap {
		const val pwmPort = 0
	}
}