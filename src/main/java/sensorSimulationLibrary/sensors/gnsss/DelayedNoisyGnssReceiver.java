package sensorSimulationLibrary.sensors.gnsss;


import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.trajectories3d.ContinuousTrajectory3dOfReferenceFrame;
import sensorSimulationLibrary.util.SeedManagerForRandomNumberGenerator;



public class DelayedNoisyGnssReceiver
    extends NoisyGnssReceiver
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private double timeDelay;  // time delay for measurements with respect to time in which they are collected
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public DelayedNoisyGnssReceiver( SeedManagerForRandomNumberGenerator sm )
    {
        super( sm );
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public void setTimeDelay( double theTimeDelay )
    {
        this.timeDelay = theTimeDelay;
    }
    
    
    public Vector3 measure( ContinuousTrajectory3dOfReferenceFrame trf , double time )
    {
        this.updateNoise();
        trf.setTime( time + this.timeDelay );
        return trf.position().add( trf.orientation().rotate( this.posTTS ) ).addInplace( this.noise );
    }
    
}
