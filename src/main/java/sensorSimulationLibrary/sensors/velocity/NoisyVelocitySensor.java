package sensorSimulationLibrary.sensors.velocity;


import java.util.Random;

import numericalLibrary.types.UnitQuaternion;
import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.trajectories3d.DifferentiableTrajectory3dOfReferenceFrame;
import sensorSimulationLibrary.util.SeedManagerForRandomNumberGenerator;



public class NoisyVelocitySensor
{
    ////////////////////////////////////////////////////////////////
    // PROTECTED VARIABLES
    ////////////////////////////////////////////////////////////////
    ///
    /**
     * {@link UnitQuaternion} that defines the rotation transformation from sensor to trajectory frame.
     */
    private UnitQuaternion qTS;
    
    /**
     * {@link Vector3} that defines the position of the sensor relative to the trajectory frame.
     */
    private Vector3 rTTS;
    
    
    protected Vector3 noiseVelocity;
    
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private Random[] rnds;
    private double sigmaPosition;  // the standard deviation of the position noise
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public NoisyVelocitySensor( SeedManagerForRandomNumberGenerator sm )
    {
        this();  // we call the private constructor
        this.rnds = new Random[3];
        for(int i=0; i<this.rnds.length; i++) {
            this.rnds[i] = new Random();
            this.rnds[i].setSeed( sm.nextSeed() );
        }
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public void setSensorOrientationToTrajectoryFrame( UnitQuaternion sensorOrientationToTrajectoryFrame )
    {
        this.qTS = sensorOrientationToTrajectoryFrame;
    }
    
    
    public void setSensorPositionFromTrajectoryFrame( Vector3 sensorPositionFromTrajectoryFrame )
    {
        this.rTTS = sensorPositionFromTrajectoryFrame;
    }
    
    
    public void setMeasurementSigmaVelocity( double sigmaPosition )
    {
        this.sigmaPosition = sigmaPosition;
    }
    
    
    public Vector3 measure( DifferentiableTrajectory3dOfReferenceFrame trf , double time )
    {
        this.updateNoise();
        trf.setTime( time );
        UnitQuaternion qOT = trf.orientation();
        Vector3 vSOS = this.qTS.rotateWithInverseInplace( trf.angularVelocity().crossProduct( this.rTTS ).addInplace( qOT.rotateWithInverse( trf.velocity() ) ) );
        return vSOS.addInplace( this.noiseVelocity );
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PROTECTED CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    protected NoisyVelocitySensor()
    {
        this.qTS = UnitQuaternion.one();
        this.rTTS = Vector3.zero();
        this.noiseVelocity = Vector3.zero();
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PROTECTED METHODS
    ////////////////////////////////////////////////////////////////
    
    protected void updateNoise()
    {
        this.noiseVelocity = Vector3.fromComponents(
                this.rnds[0].nextGaussian() ,
                this.rnds[1].nextGaussian() ,
                this.rnds[2].nextGaussian() ).scaleInplace( this.sigmaPosition );
    }
    
}
