package sensorSimulationLibrary.trajectories3d;


import kalmanFilterLibrary.stateModels.MovingFrame;
import numericalLibrary.types.Vector3;



public interface DifferentiableTrajectory3dOfReferenceFrame
    extends DifferentiableTrajectory3dOfPoint, ContinuousTrajectory3dOfReferenceFrame
{
    ////////////////////////////////////////////////////////////////
    // PUBLIC ABSTRACT METHODS
    ////////////////////////////////////////////////////////////////
    public Vector3 angularVelocity();  // angular velocity of reference frame with respect to inertial expressed in reference frame (omgFIF)
    //public RealVector3 angularAcceleration();
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC DEFAULT METHODS
    ////////////////////////////////////////////////////////////////
    
    public default void toReferenceFrameNonInertial( MovingFrame rf )
    {
        rf.setOrientationFromThisToReferenceFrame( this.orientation() );
        rf.setAngularVelocity( this.angularVelocity() );
        rf.setPositionFromReferenceFrame( this.position() );
        rf.setVelocityFromReferenceFrame( this.velocity() );
        rf.setAcceleration( this.orientation().rotateWithInverse( this.acceleration() ) );
    }
    
    
    public default double maximumAngularVelocity()
    {
        double h = 1.0e-6;  // hard coded value selected to perform numerical derivatives
        // first, we store the evaluation time, so that we can reset it at the end of the method
        double t0 = this.time();
        // we initialize the maximum acceleration 
        double maxAngularVelocity = 0.0;
        // and we repeat several times the search of the maximum value
        for( int n=0; n<10; n++ ) {
            // we select a random initial time 
            double t = Math.random() * 10000.0;
            // and we iterate to search for the maximum value
            for( int i=0; i<10; i++ ) {
                // we compute the angular velocity at t, t+h, and t-h
                this.setTime( t );
                double angularVelocity = this.angularVelocity().norm();
                this.setTime( t + h );
                double angularVelocityPlus = this.angularVelocity().norm();
                this.setTime( t - h );
                double angularVelocityMinus = this.angularVelocity().norm();
                // we update the evaluation time using Newton's method with numerical approximated derivatives
                double dt = -( angularVelocityPlus - angularVelocityMinus )/( angularVelocityPlus - 2.0 * angularVelocity + angularVelocityMinus ) * 0.5 * h;
                t = t + dt;
                // we also update the maximum value for the angular velocity
                maxAngularVelocity = ( angularVelocity > maxAngularVelocity )? angularVelocity : maxAngularVelocity;
                // and we break iterating if we reached convergence
                if( Math.abs( dt ) < 1.0e-8 ) {
                    break;
                }
            }
        }
        // we reset the evaluation time to the initial value
        this.setTime( t0 );
        // and we return the maximum acceleration
        return maxAngularVelocity;
    }
    
}
