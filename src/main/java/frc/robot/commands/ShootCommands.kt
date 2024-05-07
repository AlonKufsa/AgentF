package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.shooter.ShooterConstants
import frc.robot.subsystems.shooter.ShooterSubsystem

class ShooterDefaultCommand : Command() {
	init {
		name = "Default shooter command"
		addRequirements(ShooterSubsystem)
	}

	override fun initialize() {
		ShooterSubsystem.updateAngleControl(ShooterConstants.LOAD_ANGLE)
	}

	override fun execute() {
		ShooterSubsystem.updateAngleControl()
	}
}

class setShooterAngle