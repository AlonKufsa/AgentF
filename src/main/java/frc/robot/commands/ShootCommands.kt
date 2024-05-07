package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.shooter.ShooterState
import frc.robot.subsystems.shooter.ShooterSubsystem

class ShooterDefaultCommand : Command() {
	init {
		name = "Default shooter command"
		addRequirements(ShooterSubsystem)
	}

	override fun initialize() {
		//add
	}

	override fun execute() {
		ShooterSubsystem.updateAngleControl()
	}
}

class maintainShooterState(val shooterState: ShooterState) : Command() {
	init {
		name = "Maintain shooter state"
		addRequirements(ShooterSubsystem)
	}

	override fun initialize() {
		ShooterSubsystem.setShooterState(shooterState)
	}

	override fun execute() {
		ShooterSubsystem.maintainShooterState()
	}
}