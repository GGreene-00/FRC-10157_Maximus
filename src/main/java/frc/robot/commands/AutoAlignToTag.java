package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoAlignToTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    int[] allowedTags = {10,26};
 
    @Override
    public void initialize() {
        System.out.println("AutoAlign started");
    }

    private void stopAutoAlign() {
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    private boolean isAllowedTag(int tid){ 
        for (int tag : allowedTags) {
            if (tag == tid)
                return true;
        }
        return false;
    }

    // 0.1 - 0.4, make negative
    double kP_turn = -0.06;
    // 0.03- 0.08
    double kP_drive = 0.07;
    // This needs to be replaced with the value given from the limelight exaclty
    // where we want it placed when shooting.
    // this will give a direct shot of the target regardless of where we are on the field.
    double targetTY = 9.5;


    public AutoAlignToTag(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);             
    }

    @Override
    public void execute() {

        System.out.println("AutoAlign Running");

        int tid = (int) LimelightHelpers.getFiducialID("limelight");

        if (!LimelightHelpers.getTV("limelight") || !isAllowedTag(tid)) {
            stopAutoAlign();
            drivetrain.setControl(new SwerveRequest.Idle());
            return;
        }

        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");
        
        double drive = -(ty - targetTY) * kP_drive;
        double turn = tx * kP_turn;

        // Deadzone for aiming adjustment
        if (Math.abs(tx) < 0.15)
            turn = 0;

        // Deadzone for distance adjustment
        if (Math.abs(ty - targetTY) < 0.2)
            drive = 0;

        // turn and drive the robot whilst allowing strafing (ideally)
        drivetrain.setControl(
            driveRequest
                .withVelocityX(drive)
                .withVelocityY(0)
                .withRotationalRate(turn));

        
        // debug
        System.out.println("tx: " + tx);
        System.out.println("ty: " + ty);
        System.out.println("Aligning to tag: " + tid);
    }

    @Override
    public void end(boolean interrupted) {
        stopAutoAlign();
        drivetrain.setControl(new SwerveRequest.Idle());
        System.out.println("AutoAlign ended");
    }

    @Override
        public boolean isFinished() {
        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");

        return Math.abs(tx) < 0.15 && Math.abs(ty - targetTY) < 0.2;
    }

}
