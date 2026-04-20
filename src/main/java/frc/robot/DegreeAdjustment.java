package frc.robot;

public class DegreeAdjustment {
    
    private static final double adjustment = 3.0;
    private static final double FX = 16.54; // field size (longways)
    private static final double FY = 8.07; // field size (shortways)

    public static double getAdjustmentValue() {

        double[] botPose = LimelightHelpers.getBotPose_wpiBlue("limelight");
        double bfx = botPose[0];
        double bfy = botPose[1];

        if (bfx < (FX / 2)) {
            if (bfy < (FY / 2)) {
                return adjustment;
            } else {
                return -adjustment;
            }
        } else {
            if (bfy < (FY / 2)) {
                return -adjustment;
            } else {
                return adjustment;
            }
        }
    }

}
