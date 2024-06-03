package frc.robot.subsystems.climbing

import com.hamosad1657.lib.motors.HaSparkFlex
import com.revrobotics.CANSparkBase.IdleMode.kBrake
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object ClimbingSubsystem : SubsystemBase("Climbing") {

	private val leftMainMotor = HaSparkFlex(RobotMap.ClimbingMap.LEFT_MAIN_MOTOR_ID).apply {
		restoreFactoryDefaults()
		inverted = true
		idleMode = kBrake
		setSmartCurrentLimit(ClimbingConstants.SMART_CURRENT_LIMIT)
	}

	private val leftSecondaryMotor = HaSparkFlex(RobotMap.ClimbingMap.LEFT_SECONDARY_MOTOR_ID).apply {
		restoreFactoryDefaults()
		idleMode = kBrake
		setSmartCurrentLimit(ClimbingConstants.SMART_CURRENT_LIMIT)
		follow(leftMainMotor, true)
	}
	private val rightMainMotor = HaSparkFlex(RobotMap.ClimbingMap.RIGHT_MAIN_MOTOR_ID).apply {
		restoreFactoryDefaults()
		inverted = false
		idleMode = kBrake
		setSmartCurrentLimit(ClimbingConstants.SMART_CURRENT_LIMIT)
	}
	private val rightSecondaryMotor = HaSparkFlex(RobotMap.ClimbingMap.RIGHT_SECONDARY_MOTOR_ID).apply {
		restoreFactoryDefaults()
		idleMode = kBrake
		setSmartCurrentLimit(ClimbingConstants.SMART_CURRENT_LIMIT)
		follow(rightMainMotor, true)
	}

	fun openRight() {
		rightMainMotor.set(ClimbingConstants.OPEN_CLIMBING_OUTPUT)
	}

	fun closeRight() {
		rightMainMotor.set(ClimbingConstants.CLOSE_CLIMBING_OUTPUT)
	}

	fun stopRight() {
		rightMainMotor.stopMotor()
	}


	fun openLeft() {
		leftMainMotor.set(ClimbingConstants.OPEN_CLIMBING_OUTPUT)
	}

	fun closeLeft() {
		leftMainMotor.set(ClimbingConstants.CLOSE_CLIMBING_OUTPUT)
	}

	fun stopLeft() {
		leftMainMotor.stopMotor()
	}
}