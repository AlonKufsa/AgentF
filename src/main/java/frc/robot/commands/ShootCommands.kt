package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.leds.LedSubsystem
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

class MaintainShooterStateCommand(val shooterState: ShooterState, val useLeds: Boolean) : Command() {
	//To stop this command, you must interrupt it, or manually stop it
	init {
		name = "Maintain shooter state"
		addRequirements(ShooterSubsystem)
		if (useLeds) addRequirements(LedSubsystem)
	}

	override fun initialize() {
		ShooterSubsystem.setShooterState(shooterState)
	}

	override fun execute() {
		ShooterSubsystem.maintainShooterState()
		if (useLeds)
			LedSubsystem.shooting(ShooterSubsystem.isWithinAngleTolerance && ShooterSubsystem.isWithinShootingTolerance)
	}

	override fun end(interrupted: Boolean) {
		ShooterSubsystem.stopAngleMotor()
		LedSubsystem.actionFinished()
	}
}

class TestShooter : Command() {
	init {
		name = "Test shooter"
		addRequirements(ShooterSubsystem)
	}

	override fun initialize() {
		ShooterSubsystem.shooterMotorTest(8.0)
	}

	override fun end(interrupted: Boolean) {
		ShooterSubsystem.stopShootingMotors()
	}
}

class TestAngleMotor() : Command() {
	init {
		name = "Test angle motor"
		addRequirements(ShooterSubsystem)
	}

	override fun initialize() {
		ShooterSubsystem.angleMotorTest()
	}
}
