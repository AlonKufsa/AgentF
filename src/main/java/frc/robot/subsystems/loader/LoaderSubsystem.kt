package frc.robot.subsystems.loader

import com.ctre.phoenix6.hardware.TalonFX
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object LoaderSubsystem : SubsystemBase("Loader") {
	private val motor = TalonFX(RobotMap.LoaderMap.MOTOR_ID)
	private val beamBreak = AnalogInput(RobotMap.LoaderMap.BEAMBREAK_CHANNEL)

	val isNoteDetected: Boolean
		get() = beamBreak.voltage < LoaderConstants.BEAMBREAK_TURNPOINT

	fun runMotor(voltage: Volts) {
		motor.setVoltage(voltage)
	}

	fun stopMotor() {
		motor.stopMotor()
	}
}