package frc.robot.subsystems.climbing

import com.hamosad1657.lib.motors.HaSparkFlex
import com.revrobotics.CANSparkBase.IdleMode.kBrake
import frc.robot.RobotMap

object ClimbingSubsystem {
	private val leftMainMotor = HaSparkFlex(RobotMap.ClimbingMap.LEFT_MAIN_MOTOR_ID).apply {
		restoreFactoryDefaults()
		inverted = true
		idleMode = kBrake
		setSmartCurrentLimit(ClimbingConstants.SMART_CURRENT_LIMIT)
	}

	private val leftSecondaryMotor = HaSparkFlex(RobotMap.ClimbingMap.LEFT_SECONDARY_MOTOR_ID).apply {
		
	}
	private val rightMainMotor = HaSparkFlex(RobotMap.ClimbingMap.RIGHT_MAIN_MOTOR_ID)
	private val rightSecondaryMotor = HaSparkFlex(RobotMap.ClimbingMap.RIGHT_SECONDARY_MOTOR_ID)


}