package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.vision.Vision;
import org.tinylog.Logger;

import static org.tahomarobotics.robot.auto.AutonomousConstants.*;

public class DriveToPoseV3Command extends Command {
    private final Chassis chassis = Chassis.getInstance();

    private final ProfiledPIDController x, y, r;

    private final Timer readjustmentTimer = new Timer();

    private Pose2d goalPose;
    private boolean scoring = false;
    private final ReefPole pole;

    public DriveToPoseV3Command(ReefPole pole) {
        this.pole = pole;
        this.goalPose = pole.approachPose();

        x = new ProfiledPIDController(
            TRANSLATION_ALIGNMENT_KP, TRANSLATION_ALIGNMENT_KI, TRANSLATION_ALIGNMENT_KD,
            TRANSLATION_ALIGNMENT_CONSTRAINTS
        );
        x.setTolerance(TRANSLATION_ALIGNMENT_TOLERANCE);

        y = new ProfiledPIDController(
            TRANSLATION_ALIGNMENT_KP, TRANSLATION_ALIGNMENT_KI, TRANSLATION_ALIGNMENT_KD,
            TRANSLATION_ALIGNMENT_CONSTRAINTS
        );
        y.setTolerance(TRANSLATION_ALIGNMENT_TOLERANCE);

        r = new ProfiledPIDController(
            ROTATION_ALIGNMENT_KP, ROTATION_ALIGNMENT_KI, ROTATION_ALIGNMENT_KD,
            ROTATION_ALIGNMENT_CONSTRAINTS
        );
        r.setTolerance(ROTATION_ALIGNMENT_TOLERANCE);
        r.enableContinuousInput(-Math.PI, Math.PI);

        syncGoal();
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = chassis.getPose();
        ChassisSpeeds currentVelocity = chassis.getChassisSpeeds();

        Logger.info("Driving to {} from {}", pole.scorePose(), currentPose);

        x.reset(currentPose.getX(), currentVelocity.vxMetersPerSecond);
        y.reset(currentPose.getY(), currentVelocity.vyMetersPerSecond);
        r.reset(currentPose.getRotation().getRadians(), currentVelocity.omegaRadiansPerSecond);

        chassis.setAutoAligning(true);
        Vision.getInstance().isolate(pole.aprilTagId());

        readjustmentTimer.restart();
    }

    @Override
    public void execute() {
        Pose2d currentPose = chassis.getPose();

        double distanceToGoalPose = currentPose.getTranslation().getDistance(goalPose.getTranslation());
        if (distanceToGoalPose < APPROACH_DISTANCE_BLEND_FACTOR && !scoring) {
            goalPose = pole.scorePose();
            syncGoal();

            scoring = true;
            Logger.info("Approached scoring location, scoring...");
        }

        double speedReduction = scoring ? MathUtil.clamp(distanceToGoalPose, 0.75, 1.0) : 1;

        double vx = x.calculate(currentPose.getX()) * speedReduction;
        double vy = y.calculate(currentPose.getY()) * speedReduction;
        double vr = r.calculate(currentPose.getRotation().getRadians());

        chassis.drive(new ChassisSpeeds(vx, vy, vr), true);
    }

    @Override
    public boolean isFinished() {
        return x.atGoal() && y.atGoal() && r.atGoal() && scoring;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
        chassis.setAutoAligning(false);

        Vision.getInstance().globalize();

        readjustmentTimer.stop();
    }

    // Helper Methods

    private void syncGoal() {
        x.setGoal(goalPose.getX());
        y.setGoal(goalPose.getY());
        r.setGoal(goalPose.getRotation().getRadians());
    }
}