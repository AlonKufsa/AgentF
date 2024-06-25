package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.swerve.SwerveSubsystem

class SwerveTestCommand(): Command() {
	init {
		name = "Swerve test command"
		requirements.add(SwerveSubsystem)
	}

	override fun execute() {
		SwerveSubsystem.testMotor()
	}
}