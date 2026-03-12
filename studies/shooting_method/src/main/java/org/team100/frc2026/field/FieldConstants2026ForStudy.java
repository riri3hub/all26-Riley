package org.team100.frc2026.field;

import java.util.Optional;
import java.util.OptionalDouble;

import org.team100.lib.targeting.Drag;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * These measurements are from Onshape.
 * 
 * https://cad.onshape.com/documents/8a691e28680da30504859fce/w/c6aa636fb23edb3f1e272fb1/e/c043973ea96914e2eaa1e8fc
 */
public class FieldConstants2026ForStudy {
    /** Field width in meters */
    private static final double FIELD_WIDTH = 8.069;
    private static final double FIELD_LENGTH = 16.541;
    /**
     * Center of the hub target, inside the cone.
     * 
     * Use this for aiming.
     * 
     * Height is the bottom of the cone.
     */
    public static final Translation3d HUB = new Translation3d(
            4.626, 4.035, 1.434);

    /**
     * Estimate of the inclination of the hub funnel from horizontal.
     * We want shots to come in from at least this high.
     */
    public static final double HUB_ELEVATION = 1.0;
    /**
     * The rectangular base of the hub.
     * 
     * Avoid hitting this when lobbing.
     */
    public static final Rectangle2d HUB_BASE = new Rectangle2d(
            new Translation2d(4.028, 3.438),
            new Translation2d(5.223, 4.631));
    /**
     * The rectangular base of the opponent's hub.
     * 
     * Avoid hitting this when lobbing.
     */
    public static final Rectangle2d OPPONENT_HUB_BASE = new Rectangle2d(
            new Translation2d(11.318, 3.438),
            new Translation2d(12.512, 4.631));
    /**
     * Area on the field where you're allowed to shoot.
     * 
     * Measured from the outside edge of the tape to the wall.
     * 
     * Does not include the bump.
     * 
     * G407: bumpers must overlap this zone, or major foul (15 pt).
     */
    public static final Rectangle2d ALLIANCE_ZONE = new Rectangle2d(
            new Translation2d(0.000, 0.000),
            new Translation2d(4.029, FIELD_WIDTH));
    /**
     * Middle zone of the field.
     * 
     * Measured between the faces of each hub.
     * 
     * Does not include the bump.
     */
    public static final Rectangle2d NEUTRAL_ZONE = new Rectangle2d(
            new Translation2d(5.223, 0.000),
            new Translation2d(11.318, FIELD_WIDTH));

    /**
     * Alliance zone on the opposite side of the field.
     * 
     * Measured from the outside edge of the tape to the wall.
     * 
     * Does not include the bump.
     */
    public static final Rectangle2d OPPONENT_ZONE = new Rectangle2d(
            new Translation2d(12.512, 0.000),
            new Translation2d(FIELD_LENGTH, FIELD_WIDTH));

    /**
     * For computing firing solutions.
     * 
     * See page 33 of the 2026 manual; this is the midpoint of the range.
     * 
     * https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
     */
    public static double FUEL_MASS_KG = 0.215;

    /**
     * For computing firing solutions.
     * 
     * See page 32 of the 2026 manual, pi*r^2.
     * 
     * https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
     * 
     */
    public static double FUEL_AREA_M2 = 0.017671;

    /**
     * Drag coefficient.
     * 
     * Reynolds number is around 70000, which is (probably) just below the "drag
     * crisis".
     * 
     * https://www.simscale.com/blog/reynolds-number-calculator/
     * https://aerotoolbox.com/reynolds-number-calculator/
     * https://en.wikipedia.org/wiki/Drag_coefficient
     */
    public static double FUEL_CD = 0.5;

    /**
     * Lift coefficient for the Magnus effect.
     * 
     * Note the solution is near the drag crisis, which
     * means that the magnus effect may actually be reversed
     * 
     * https://en.wikipedia.org/wiki/Magnus_effect#Inverse_Magnus_effect
     */
    public static double FUEL_CL = 0;

    /**
     * Rotational drag
     */
    public static double FUEL_B = 0;

    /**
     * First-principles drag model.
     *
     * TODO: calibrate this if we're going to use it.
     */
    public static Drag FUEL_DRAG = new Drag(
            FUEL_CD,
            FUEL_CL,
            FUEL_AREA_M2,
            FUEL_MASS_KG,
            FUEL_B);

    /**
     * Target depends on robot location.
     * 
     * In our own zone: shoot to score.
     * In the neutral zone: lob to our zone.
     * Otherwise: empty.
     */
    public static Optional<Translation2d> TARGET(Translation2d robotPosition) {
        if (isInAllianceZone(robotPosition)) {
            return Optional.of(FieldConstants2026.HUB.toTranslation2d());
        }
        if (isInNeutralZone(robotPosition)) {
            return Optional.of(new Translation2d(0, robotPosition.getY()));
        }
        return Optional.empty();
    }

    public static boolean isInAllianceZone(Translation2d robotPosition) {
        return FieldConstants2026.ALLIANCE_ZONE.contains(robotPosition);
    }

    public static boolean isInNeutralZone(Translation2d robotPosition) {
        return FieldConstants2026.NEUTRAL_ZONE.contains(robotPosition);
    }

    /**
     * Range to target, meters.
     * 
     * Target depends on location (see above).
     */
    public static OptionalDouble RANGE(Translation2d robotPosition) {
        Optional<Translation2d> o = TARGET(robotPosition);
        if (o.isEmpty())
            return OptionalDouble.empty();
        Translation2d targetPosition = o.get();
        return OptionalDouble.of(robotPosition.getDistance(targetPosition));
    }

    /** 2d distance to the hub center, for scoring. */
    public static double rangeToHub(Translation2d robotPosition) {
        return robotPosition.getDistance(FieldConstants2026.HUB.toTranslation2d());
    }

    /** Distance to the middle of our zone, for lobbing. */
    public static double rangeToLob(Translation2d robotPosition) {
        return robotPosition.getX() - 2;
    }

}
