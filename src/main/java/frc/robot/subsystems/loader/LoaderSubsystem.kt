package frc.robot.subsystems.loader

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj2.command.SubsystemBase

object LoaderSubsystem : SubsystemBase("Loader") {
	private val motor = TalonFX(LoaderConstants.MOTOR_ID)
	private val beamBreak = AnalogInput(LoaderConstants.BEAMBREAK_ID)

	val isNoteDetected: Boolean
		get() = beamBreak.voltage < LoaderConstants.BEAMBREAK_TURNPOINT

	fun runMotorOutward() {
		motor.setVoltage(LoaderConstants.OUTWARD_VOLTAGE)
	}

	fun runMotorInward() {
		motor.setVoltage(LoaderConstants.INWARD_VOLTAGE)
	}

	fun stopMotor() {
		motor.stopMotor()
	}
}