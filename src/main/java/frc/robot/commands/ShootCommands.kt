package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.shooter.ShooterState
import frc.robot.subsystems.shooter.ShooterSubsystem

class DefaultShooterCommand : Command() {
	init {
		name = "Default shooter command"
		addRequirements(ShooterSubsystem)
	}

	override fun initialize() {
		ShooterSubsystem.setShooterState(ShooterState.COLLECT)
	}

	override fun execute() {
		ShooterSubsystem.maintainShooterState()
	}
}

class MaintainShooterStateCommand(val shooterState: ShooterState) : Command() {
	//To stop this command, you must interrupt it, or manually stop it
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