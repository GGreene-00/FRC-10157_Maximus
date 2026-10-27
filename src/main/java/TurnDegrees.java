package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class TurnDegrees extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final double adjustmentTD;
    private double target;

    private final PIDController controller = new PIDController(0.015, 0, 0.002);
    // kp is the deadzone for this turn. It might cause overshooting if too high, or oscillating if too low.
    // kd very slightly slows turning based on distance to the target, which should help with overshooting.

    public TurnDegrees(CommandSwerveDrivetrain drivetrain, double adjustmentTD) {
        this.drivetrain = drivetrain;
        this.adjustmentTD = adjustmentTD;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        target = drivetrain.getHeadingDegrees() + adjustmentTD;
    }

    @Override
    public void execute() {
        double output = controller.calculate(drivetrain.getHeadingDegrees(), target); // determines movement
        output = Math.max(-0.3, Math.min(0.3, output)); // clamp in order to prevent overshooting

        drivetrain.setControl(
            new SwerveRequest.FieldCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(output)
        );
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getHeadingDegrees() - target) < 0.5; // tolerance in degrees
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
        System.out.println("Adjustment complete");
    }
}
