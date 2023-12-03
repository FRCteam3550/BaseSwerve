package swervelib;
public interface SteerController {
    ContinuousAngle getReferenceAngle();

    void setReferenceAngle(ContinuousAngle referenceAngle);

    ContinuousAngle getAngle();

    DiscreetAngle getAbsoluteAngle();
}
