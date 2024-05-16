package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.intake.IntakeSubsystem

class DefaultIntakeCommand : Command() {
	init {
		name = "Default intake command"
		addRequirements(IntakeSubsystem)
	}

	//This command is for added safety, in the case someone forgets to stop intake motors on command end
	override fun initialize() {
		IntakeSubsystem.stopMotors()
	}
}

class RunIntakeCommand : Command() {
	init {
		name = "Run intake"
		addRequirements(IntakeSubsystem)
	}

	override fun execute() {
		IntakeSubsystem.runMotors()
	}

	override fun end(interrupted: Boolean) {
		IntakeSubsystem.stopMotors()
	}
}