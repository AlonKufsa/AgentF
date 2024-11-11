package frc.robot.subsystems.leds

import com.hamosad1657.lib.leds.LEDStrip
import com.hamosad1657.lib.leds.RGBColor
import com.hamosad1657.lib.units.Seconds
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.RobotMap.LedMap
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.leds.LedMode.*
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.leds.LedConstants as Constants


object LedSubsystem : SubsystemBase("Led subsystem") {
	private val led = LEDStrip(Constants.STRIP_LENGTH, LedMap.pwmPort)
	private var atFirstLoop = true
	var ledMode = LedMode.DEFAULT
		set(value) {
			atFirstLoop = true
			field = value
		}


	private val actionFinishedTimer = Timer()

	private fun setColor(color: RGBColor) {
		led.setColor(color)
	}

	//Call periodically
	private fun blink(blinkTime: Seconds) {
		led.blink(blinkTime)
	}

	private fun turnOff() {
		led.setColor(RGBColor.BLACK)
	}

	override fun periodic() {
		if (Robot.isDisabled) ledMode = DISABLED

		if (ledMode == DEFAULT) default()
		if (ledMode == DISABLED) disabled()
		if (ledMode == SHOOTING) shooting()
		if (ledMode == MANUAL_ANGLE_CONTROL) manualShootingControl()
		if (ledMode == INTAKE) intake()
		if (ledMode == ACTION_FINISHED_SUCCESSFULLY) actionFinished()
	}

	private fun default() {
		disabled()
	}

	private fun intake() {
		if (!IntakeSubsystem.isNoteInIntake) {
			setColor(RGBColor.YELLOW)
		} else {
			setColor(RGBColor.GREEN)
		}
	}

	private fun actionFinished() {
		if (atFirstLoop) {
			actionFinishedTimer.restart()
			setColor(RGBColor.GREEN)
			atFirstLoop = false
		}
		blink(Constants.BLINK_TIME)
		if (actionFinishedTimer.hasElapsed(Constants.ACTION_FINISHED_DURATION)) {
			ledMode = DEFAULT
			actionFinishedTimer.stop()
		}
	}

	private fun shooting() {
		if (!(ShooterSubsystem.isAtShooterState)) {
			setColor(RGBColor.YELLOW)
		} else {
			setColor(RGBColor.GREEN)
		}
	}

	private fun manualShootingControl() {
		setColor(RGBColor.MAGENTA)
	}


	private fun disabled() {
		DriverStation.getAlliance().ifPresentOrElse({
			when (DriverStation.getAlliance().get()) {
				Alliance.Blue -> setColor(RGBColor.BLUE)
				Alliance.Red -> setColor(RGBColor.RED)
			}
		},
			{
				if (atFirstLoop) {
					setColor(RGBColor.ORANGE)
					atFirstLoop = false
				}
				blink(Constants.BLINK_TIME)
			}
		)
	}

	override fun initSendable(builder: SendableBuilder) {
		builder.addStringProperty("LED color", { led.currentColor.toString() }, null)
		builder.addStringProperty("Mode", { ledMode.toString() }, null)
		builder.addDoubleProperty("Action finished timer reading seconds", { actionFinishedTimer.get() }, null)
	}


}