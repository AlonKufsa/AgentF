package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.leds.LedSubsystem
import frc.robot.subsystems.shooter.ShooterState
import frc.robot.subsystems.shooter.ShooterSubsystem


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
		addRequirements(IntakeSubsystem, LedSubsystem)
	}

	override fun execute() {
		if (ShooterSubsystem.isWithinAngleTolerance && ShooterSubsystem.angleSetpoint == ShooterState.COLLECT.angle) {
			IntakeSubsystem.runMotors()
			LedSubsystem.intakeRunning()
		} else {
			IntakeSubsystem.stopMotors()
		}

	}

	override fun end(interrupted: Boolean) {
		IntakeSubsystem.stopMotors()
		LedSubsystem.actionFinished()
	}
}