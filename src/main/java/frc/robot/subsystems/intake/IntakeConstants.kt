package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.rpm

object IntakeConstants {
	val CURRENT_LIMITS_CONFIGS =
		CurrentLimitsConfigs().apply {
			SupplyCurrentLimitEnable = true
			SupplyCurrentLimit = 40.0
		}
	const val MOTOR_VOLTAGE: Volts = 12.0
	val MOTOR_RPM: AngularVelocity = 2600.rpm
}