package sensorSimulationLibrary.trajectories3d;


import numericalLibrary.types.Vector3;



public interface DifferentiableTrajectory3dOfPoint
    extends ContinuousTrajectory3dOfPoint
{
    ////////////////////////////////////////////////////////////////
    // PUBLIC ABSTRACT METHODS
    ////////////////////////////////////////////////////////////////
    public Vector3 velocity();  // velocity measured from origin (vOOF)
    public Vector3 acceleration();  // acceleration measured from origin (aOOF)
    public Vector3 jerk();  // jerk (derivative of acceleration) measured from origin (aOOF)
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC DEFAULT METHODS
    ////////////////////////////////////////////////////////////////
    
    public default double maximumAcceleration()
    {
        double h = 1.0e-6;  // hard coded value selected to perform numerical derivatives
        // first, we store the evaluation time, so that we can reset it at the end of the method
        double t0 = this.time();
        // we initialize the maximum acceleration 
        double maxAcceleration = 0.0;
        // and we repeat several times the search of the maximum value
        for( int n=0; n<10; n++ ) {
            // we select a random initial time 
            double t = Math.random() * 10000.0;
            // and we iterate to search for the maximum value
            for( int i=0; i<1000; i++ ) {
                // we compute the gradient of the acceleration norm
                this.setTime( t );
                double gradient = this.acceleration().normalize().dot( this.jerk() );
                // we also update the maximum value for the acceleration
                double acceleration = this.acceleration().norm();
                maxAcceleration = ( acceleration > maxAcceleration )? acceleration : maxAcceleration;
                // and we compute the displaced gradient to later obtain the numerical gradient derivative 
                this.setTime( t + h );
                double gradientPlus = this.acceleration().normalize().dot( this.jerk() );
                this.setTime( t - h );
                double gradientMinus = this.acceleration().normalize().dot( this.jerk() );
                // we update the evaluation time using Newton's method
                double dt = - gradient/( ( gradientPlus - gradientMinus )/h );
                t = t + dt;
                // and we break iterating if we reached convergence
                if( Math.abs( dt ) < 1.0e-10 ) {
                    break;
                }
            }
        }
        // we reset the evaluation time to the initial value
        this.setTime( t0 );
        // and we return the maximum acceleration
        return maxAcceleration;
    }
    
}
