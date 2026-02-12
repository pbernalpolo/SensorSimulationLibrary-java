package sensorSimulationLibrary.sensors.gnsss;


import java.util.Random;

import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.trajectories3d.ContinuousTrajectory3dOfReferenceFrame;
import sensorSimulationLibrary.util.SeedManagerForRandomNumberGenerator;



public class NoisyGnssReceiver
{
    ////////////////////////////////////////////////////////////////
    // PROTECTED VARIABLES
    ////////////////////////////////////////////////////////////////
    protected Vector3 posTTS;  // position of sensor reference frame measured from rf reference frame described using a vector
    protected Vector3 noise;
    
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private Random[] rnds;
    private double sigma;  // the standard deviation of the noise
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public NoisyGnssReceiver( SeedManagerForRandomNumberGenerator sm )
    {
        this();  // we call the private constructor
        this.rnds = new Random[3];
        for(int i=0; i<3; i++) {
            this.rnds[i] = new Random();
            this.rnds[i].setSeed( sm.nextSeed() );
        }
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public void setMeasurementSigma( double theSigma )
    {
        this.sigma = theSigma;
    }
    
    
    public Vector3 measure( ContinuousTrajectory3dOfReferenceFrame trf , double time )
    {
        this.updateNoise();
        trf.setTime( time );
        return trf.position().add( trf.orientation().rotate( this.posTTS ) ).addInplace( this.noise );
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PROTECTED CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    protected NoisyGnssReceiver()
    {
        this.posTTS = Vector3.zero();
        this.noise = Vector3.zero();
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PROTECTED METHODS
    ////////////////////////////////////////////////////////////////
    
    protected void updateNoise()
    {
        this.noise = Vector3.fromComponents(
        		this.rnds[0].nextGaussian() ,
                this.rnds[1].nextGaussian() ,
                this.rnds[2].nextGaussian() ).scaleInplace( this.sigma );
    }
    
}
