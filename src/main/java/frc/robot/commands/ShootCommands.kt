package frc.robot.commands

import com.hamosad1657.lib.units.minus
import com.hamosad1657.lib.units.rpm
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.leds.LedMode.*
import frc.robot.subsystems.leds.LedSubsystem
import frc.robot.subsystems.shooter.ShooterConstants
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
			LedSubsystem.ledMode = SHOOTING
	}

	override fun end(interrupted: Boolean) {
		ShooterSubsystem.stopAngleMotor()
		if (useLeds) LedSubsystem.ledMode = ACTION_FINISHED_SUCCESSFULLY
	}
}

class ManualShootingAngleControl(val xPos: () -> Double, val yPos: () -> Double) : Command() {
	init {
		name = "Manual shooting angle control"
		addRequirements(ShooterSubsystem, LedSubsystem)
	}

	override fun execute() {
		val angle: Rotation2d = ShooterConstants.FLOOR_RELATIVE_OFFSET minus Rotation2d(xPos(), -yPos())
		val length: Double = Translation2d(xPos(), yPos()).norm
		if (length > 0.5) ShooterSubsystem.setShooterState(ShooterState(angle, 0.0.rpm))
		else ShooterSubsystem.maintainShooterState()
		LedSubsystem.ledMode = MANUAL_ANGLE_CONTROL
	}

	override fun end(interrupted: Boolean) {
		LedSubsystem.ledMode = DEFAULT
	}
}
