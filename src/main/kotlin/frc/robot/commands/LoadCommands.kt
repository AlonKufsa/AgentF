package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.leds.LedMode.ACTION_FINISHED_SUCCESSFULLY
import frc.robot.subsystems.leds.LedMode.DEFAULT
import frc.robot.subsystems.leds.LedSubsystem
import frc.robot.subsystems.loader.LoaderConstants
import frc.robot.subsystems.loader.LoaderSubsystem

class DefaultLoaderCommand : Command() {
	init {
		name = "Default loader command"
		addRequirements(LoaderSubsystem)
	}

	// This command is for added safety, in the case someone forgets to disable motors on command end
	override fun initialize() {
		LoaderSubsystem.stopMotor()
	}
}

class TransferToShooterCommand : Command() {
	//Use with a 2 second timeout and make sure shooter is actually moving and in the right position
	init {
		name = "Transfer to shooter"
		addRequirements(LoaderSubsystem)
	}

	override fun execute() {
		LoaderSubsystem.runMotor(LoaderConstants.LOADING_VOLTAGE)
	}

	override fun end(interrupted: Boolean) {
		LoaderSubsystem.stopMotor()
	}
}


class LoadNoteCommand : Command() {
	//Make sure Shooter is in loading position
	init {
		name = "Load note"
		addRequirements(LoaderSubsystem)
	}

	override fun execute() {
		LoaderSubsystem.runMotor(LoaderConstants.INTAKE_VOLTAGE)
	}

	override fun isFinished(): Boolean {
		return LoaderSubsystem.isNoteDetected
	}

	override fun end(interrupted: Boolean) {
		LoaderSubsystem.stopMotor()
		if (!interrupted) LedSubsystem.ledMode = ACTION_FINISHED_SUCCESSFULLY
		else LedSubsystem.ledMode = DEFAULT
	}
}

class LoaderEjectToAmpCommand : Command() {
	//Use with a 1.5 second timeout in a command group and make sure shooter is in amp position
	init {
		name = "Loader eject to amp"
		addRequirements(LoaderSubsystem)
	}

	override fun execute() {
		LoaderSubsystem.runMotor(LoaderConstants.AMP_EJECT_VOLTAGE)
	}

	override fun end(interrupted: Boolean) {
		LoaderSubsystem.stopMotor()
		LedSubsystem.ledMode = ACTION_FINISHED_SUCCESSFULLY
	}
}

class TestLoaderMotors : Command() {
	init {
		name = "Loader test"
		addRequirements(LoaderSubsystem)
	}

	override fun execute() {
		LoaderSubsystem.runMotor(1.0)
	}

	override fun end(interrupted: Boolean) {
		LoaderSubsystem.stopMotor()
	}
}