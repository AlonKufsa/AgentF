package frc.robot.subsystems.loader

import com.hamosad1657.lib.units.Seconds
import com.hamosad1657.lib.units.Volts

object LoaderConstants {

	const val INTAKE_VOLTAGE: Volts = 2.0
	const val LOADING_VOLTAGE: Volts = 5.0
	const val AMP_EJECT_VOLTAGE: Volts = -4.0

	const val BEAMBREAK_TURNPOINT = 0.4

	const val AMP_EJECT_DURATION: Seconds = 1.5
	const val TRANSFER_TO_SHOOTER_DURATION: Seconds = 2.0
}