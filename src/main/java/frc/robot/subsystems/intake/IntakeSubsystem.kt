package frc.robot.subsystems.intake

import com.ctre.phoenix6.hardware.TalonFX
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

	fun updateIsNoteInIntake() {
		if (timer.hasElapsed(2.0)) {
			if (!isNoteInIntake) timer.restart()
			isNoteInIntake = topMotor.velocity.value < IntakeConstants.MOTOR_RPM.asRps
		} else if (isNoteInIntake) isNoteInIntake = true
		else isNoteInIntake = (topMotor.velocity.value < IntakeConstants.MOTOR_RPM.asRps)
	}

	var isNoteInIntake: Boolean = false
		get() {
			updateIsNoteInIntake()
			return field
		}
		private set

	fun runMotors() {
		bottomMotor.setVoltage(IntakeConstants.MOTOR_VOLTAGE)
		topMotor.setVoltage(IntakeConstants.MOTOR_VOLTAGE)
	}

	fun stopMotors() {
		bottomMotor.stopMotor()
		topMotor.stopMotor()
	}
}