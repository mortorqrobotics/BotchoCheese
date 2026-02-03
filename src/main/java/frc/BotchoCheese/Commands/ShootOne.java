package frc.BotchoCheese.Commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.BotchoCheese.Subsystems.ShooterOne;

public class ShootOne extends Command {
    private final ShooterOne shooter1;
    public ShootOne(ShooterOne shooter1) {
        this.shooter1 = shooter1;

        addRequirements(shooter1);
    }



    @Override
    public void execute() {
        System.out.println("Shot!");
        shooter1.shoot();
    }

    @Override
    public void end(boolean interrupted) {
        shooter1.stopMotors();
    }

}
