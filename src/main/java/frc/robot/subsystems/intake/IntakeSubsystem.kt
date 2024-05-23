package frc.robot.subsystems.intake

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object IntakeSubsystem : SubsystemBase("Intake") {
	private val bottomMotor = TalonFX(RobotMap.IntakeMap.BOTTOM_MOTOR_ID).apply {
		configurator.apply(IntakeConstants.CURRENT_LIMITS_CONFIGS)
	}
	private val topMotor = TalonFX(RobotMap.IntakeMap.TOP_MOTOR_ID).apply {
		configurator.apply(IntakeConstants.CURRENT_LIMITS_CONFIGS)
	}
	private val timer = Timer()
	private var areMotorsOn = false
	private var hasStarted = false

	private fun updateIsNoteInIntake() {
		if (topMotor.velocity.value > IntakeConstants.MOTOR_REGULAR_SPEED.asRps)
			hasStarted = true


		if (hasStarted && areMotorsOn) {
			if (timer.hasElapsed(2.0)) {
				if (!isNoteInIntake) timer.reset()
				isNoteInIntake = ((topMotor.velocity.value < IntakeConstants.MOTOR_REGULAR_SPEED.asRps) && areMotorsOn)
			} else if (isNoteInIntake) isNoteInIntake = true
			else {
				isNoteInIntake = (topMotor.velocity.value < IntakeConstants.MOTOR_REGULAR_SPEED.asRps && areMotorsOn)
				if (isNoteInIntake) timer.restart()
			}
		}
	}

	var isNoteInIntake: Boolean = false
		private set

	fun runMotors() {
		bottomMotor.setVoltage(IntakeConstants.MOTOR_VOLTAGE)
		topMotor.setVoltage(IntakeConstants.MOTOR_VOLTAGE)
		areMotorsOn = true
	}

	fun stopMotors() {
		bottomMotor.stopMotor()
		topMotor.stopMotor()
		areMotorsOn = false
		hasStarted = false
		isNoteInIntake = false
	}

	override fun periodic() {
		updateIsNoteInIntake()
	}

	override fun initSendable(builder: SendableBuilder) {
		builder.addBooleanProperty("Is note in intake", { isNoteInIntake }, null)
		builder.addDoubleProperty("Motor speed RPM", { topMotor.velocity.value * 60 }, null)
		builder.addDoubleProperty("Note in intake timer seconds", { timer.get() }, null)
	}
}