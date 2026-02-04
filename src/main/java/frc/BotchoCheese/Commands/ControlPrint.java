package frc.BotchoCheese.Commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.BotchoCheese.Subsystems.ShooterOne;

public class ControlPrint extends Command {
    private final ShooterOne shooter1;
    public ControlPrint(ShooterOne shooter1) {
        this.shooter1 = shooter1;

        addRequirements(shooter1);
    }



    @Override
    public void execute() {
        System.out.println("Printed!");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("");
    }

}
