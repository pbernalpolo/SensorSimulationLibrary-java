package sensorSimulationLibrary.trajectories3d;


import numericalLibrary.types.Vector3;



public class Difference3dDecorator
    implements DifferentiableTrajectory3dOfPoint
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private DifferentiableTrajectory3dOfPoint plus;  // minuend trajectory
    private DifferentiableTrajectory3dOfPoint minus;  // subtrahend trajectory
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////

    public Difference3dDecorator(
            DifferentiableTrajectory3dOfPoint minuend ,
            DifferentiableTrajectory3dOfPoint subtrahend
            )
    {
        this.plus = minuend;
        this.minus = subtrahend;
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public void setTime( double theTime )
    {
        this.plus.setTime( theTime );
        this.minus.setTime( theTime );
    }
    
    
    public double time()
    {
        return this.plus.time();
    }
    
    
    public Vector3 position()
    {
        return this.plus.position().subtract( this.minus.position() );
    }
    
    
    public Vector3 velocity()
    {
        return this.plus.velocity().subtract( this.minus.velocity() );
    }
    
    
    public Vector3 acceleration()
    {
        return this.plus.acceleration().subtract( this.minus.acceleration() );
    }
    
    
    public Vector3 jerk()
    {
        return this.plus.jerk().subtract( this.minus.jerk() );
    }
    
}
