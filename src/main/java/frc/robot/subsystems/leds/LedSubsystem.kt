package frc.robot.subsystems.leds

import com.hamosad1657.lib.leds.LEDStrip
import com.hamosad1657.lib.leds.RGBColor
import com.hamosad1657.lib.units.Seconds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.RobotMap.LedMap
import frc.robot.subsystems.leds.LedConstants as Constants

object LedSubsystem : SubsystemBase("Led subsystem") {
	private val led = LEDStrip(Constants.STRIP_LENGTH, LedMap.pwmPort)
	private var flashing = false
		set(value) {
			field = value
			if (!value) setColor(RGBColor.BLACK)
		}

	private var withTimeout = false
	private val timer = Timer()

	fun setColor(color: RGBColor) {
		flashing = false
		led.setColor(color)
	}

	//Call periodically
	fun blink(blinkTime: Seconds) {
		led.blink(blinkTime)
	}

	fun turnOff() {
		led.setColor(RGBColor.BLACK)
		flashing = false
	}

	fun startFlashing(color: RGBColor, timeout: Boolean) {
		withTimeout = if (timeout) true
		else false
		setColor(color)
		timer.restart()
		flashing = true
	}

	override fun periodic() {
		if (flashing) blink(Constants.BLINK_TIME)
		if (timer.hasElapsed(Constants.FLASHING_DURATION) && withTimeout) {
			flashing = false
			timer.stop()
		}
		if (Robot.isDisabled) disabled()
	}

	//Intake
	fun intakeRunningNoNote() {
		flashing = false
		setColor(RGBColor.YELLOW)
	}

	fun intakeRunningHasNote() {
		setColor(RGBColor.GREEN)
	}

	fun intakeFinished() {
		startFlashing(RGBColor.GREEN, true)
	}

	//Shooting
	fun shootingNotReady() {
		setColor(RGBColor.YELLOW)
	}

	fun shootingReady() {
		flashing = false
		setColor(RGBColor.GREEN)
	}

	fun shootingFinished() {
		startFlashing(RGBColor.GREEN, true)
	}

	fun disabled() {
		DriverStation.getAlliance().ifPresentOrElse({
			when (DriverStation.getAlliance().get()) {
				Alliance.Blue -> setColor(RGBColor.BLUE)
				Alliance.Red -> setColor(RGBColor.RED)
			}
		},
			{ startFlashing(RGBColor.ORANGE, false) }
		)
	}


}