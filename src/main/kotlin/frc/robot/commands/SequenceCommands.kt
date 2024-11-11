package frc.robot.commands

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import frc.robot.subsystems.shooter.ShooterState

class CollectAndLoadCommand : ParallelRaceGroup() {
	init {
		name = "Collect and load"
		addCommands(
			MaintainShooterStateCommand(ShooterState.COLLECT, false),
			RunIntakeCommand(),
			LoadNoteCommand(),
		)
	}
}

class ShootLoadedNoteCommand(state: ShooterState) : ParallelRaceGroup() {
	init {
		name = "Shoot loaded note"
		addCommands(
			MaintainShooterStateCommand(state, true),
			TransferToShooterCommand(true)
		)
	}
}