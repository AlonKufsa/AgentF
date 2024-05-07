package frc.robot.subsystems.intake

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase

object IntakeSubsystem : SubsystemBase("Intake") {
	private val bottomMotor = TalonFX(IntakeConstants.BOTTOM_MOTOR_ID)
	private val topMotor = TalonFX(IntakeConstants.TOP_MOTOR_ID)

	fun runMotors() {
		bottomMotor.setVoltage(IntakeConstants.MOTOR_VOLTAGE)
		topMotor.setVoltage(IntakeConstants.MOTOR_VOLTAGE)
	}

	fun stopMotors() {
		bottomMotor.stopMotor()
		topMotor.stopMotor()
	}
}