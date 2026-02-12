package sensorSimulationLibrary.trajectories3d;


import numericalLibrary.types.Vector3;



public class Trajectory3dOfPointNumericalDifferentiator
    implements DifferentiableTrajectory3dOfPoint
{
    ////////////////////////////////////////////////////////////////
    // PROTECTED VARIABLES
    ////////////////////////////////////////////////////////////////
    protected double t;  // time selected to evaluate the trajectory
    protected double h;  // step used to compute numerical derivatives
    
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private ContinuousTrajectory3dOfPoint ctp;  // the ContinuousTrajectoryOfPoint to be differentiated
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////

    public Trajectory3dOfPointNumericalDifferentiator( ContinuousTrajectory3dOfPoint theContinuousTrajectoryOfPoint )
    {
        this.ctp = theContinuousTrajectoryOfPoint;
        this.h = 1.0e-5;  // default value
    }

    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public void setTime( double time )
    {
        this.t = time;
    }
    
    
    public double time()
    {
        return this.t;
    }
    
    
    public Vector3 position()
    {
        this.ctp.setTime( this.t );
        return this.ctp.position();
    }
    
    
    public Vector3 velocity()
    {
        this.ctp.setTime( this.t + this.h );
        Vector3 rOOTplus = this.ctp.position();
        this.ctp.setTime( this.t - this.h );
        Vector3 rOOTminus = this.ctp.position();
        return this.numericalDerivative( rOOTplus , rOOTminus );
    }
    
    
    public Vector3 acceleration()
    {
        double t0 = this.t;
        this.setTime( t0 + this.h );
        Vector3 vOOFplus = this.velocity();
        this.setTime( t0 - this.h );
        Vector3 vOOFminus = this.velocity();
        Vector3 aOOF = this.numericalDerivative( vOOFplus , vOOFminus );
        this.setTime( t0 );
        return aOOF;
    }
    
    
    public Vector3 jerk()
    {
        double t0 = this.t;
        this.setTime( t0 + this.h );
        Vector3 aOOFplus = this.acceleration();
        this.setTime( t0 - this.h );
        Vector3 aOOFminus = this.acceleration();
        Vector3 jOOF = this.numericalDerivative( aOOFplus , aOOFminus );
        this.setTime( t0 );
        return jOOF;
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PROTECTED METHODS
    ////////////////////////////////////////////////////////////////
    
    protected Vector3 numericalDerivative( Vector3 plus , Vector3 minus )
    {
        return plus.subtract( minus ).scaleInplace( 1.0/( 2.0 * this.h ) );
    }


}
