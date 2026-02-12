package sensorSimulationLibrary.trajectories3d;


import numericalLibrary.types.Vector3;



public class Translation3dDecorator
    implements DifferentiableTrajectory3dOfPoint
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private DifferentiableTrajectory3dOfPoint dtp;  // the DifferentiableTrajectoryOfPoint to be displaced
    private Vector3 displacement;
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////

    public Translation3dDecorator( DifferentiableTrajectory3dOfPoint theDifferentiableTrajectoryOfPoint )
    {
        this.dtp = theDifferentiableTrajectoryOfPoint;
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public Translation3dDecorator setDisplacement( Vector3 theDisplacement )
    {
        this.displacement = theDisplacement;
        return this;
    }
    
    
    public void setTime( double theTime )
    {
        this.dtp.setTime( theTime );
    }
    
    
    public double time()
    {
        return this.dtp.time();
    }
    
    
    public Vector3 position()
    {
        return this.dtp.position().add( this.displacement );
    }
    
    
    public Vector3 velocity()
    {
        return this.dtp.velocity();
    }
    
    
    public Vector3 acceleration()
    {
        return this.dtp.acceleration();
    }
    
    
    public Vector3 jerk()
    {
        return this.dtp.jerk();
    }
    
}
