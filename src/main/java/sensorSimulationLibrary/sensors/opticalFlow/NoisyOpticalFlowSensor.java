package sensorSimulationLibrary.sensors.opticalFlow;


import java.util.Random;

import numericalLibrary.types.MatrixReal;
import numericalLibrary.types.UnitQuaternion;
import numericalLibrary.types.Vector2;
import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.trajectories3d.DifferentiableTrajectory3dOfReferenceFrame;
import sensorSimulationLibrary.util.SeedManagerForRandomNumberGenerator;



public class NoisyOpticalFlowSensor
{
    ////////////////////////////////////////////////////////////////
    // PROTECTED VARIABLES
    ////////////////////////////////////////////////////////////////
    
    /**
     * {@link UnitQuaternion} that defines the rotation transformation from sensor to trajectory frame.
     */
    private UnitQuaternion qTS;
    
    /**
     * {@link Vector3} that defines the position of the sensor relative to the trajectory frame.
     */
    private Vector3 rTTS;
    
    
    protected Vector2 noiseOpticalFlow;
    
    private double distanceToTexture;
    
    private double scaleX;
    private double scaleY;
    
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private Random[] rnds;
    private double sigmaFlow;  // the standard deviation of the optical flow
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public NoisyOpticalFlowSensor( SeedManagerForRandomNumberGenerator sm )
    {
        this();  // we call the private constructor
        this.rnds = new Random[2];
        for(int i=0; i<this.rnds.length; i++) {
            this.rnds[i] = new Random();
            this.rnds[i].setSeed( sm.nextSeed() );
        }
        this.distanceToTexture = 1.0e10;
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
    
    
    public void setMeasurementSigma( double sigmaFlow )
    {
        this.sigmaFlow = sigmaFlow;
    }
    
    
    public void setScaleFactors( double scaleFactorX , double scaleFactorY )
    {
    	this.scaleX = scaleFactorX;
    	this.scaleY = scaleFactorY;
    }
    
    
    public void setDistanceToTexture( double distanceToTexture )
    {
    	this.distanceToTexture = distanceToTexture;
    }
    
    
    public Vector2 measure( DifferentiableTrajectory3dOfReferenceFrame trf , double time )
    {
        this.updateNoise();
        trf.setTime( time );
        UnitQuaternion qOT = trf.orientation();
        Vector3 v = this.qTS.rotateWithInverseInplace(
        		trf.angularVelocity().crossProduct( this.rTTS.add( this.qTS.rotateInplace( Vector3.k() ) ) )
        		.addInplace( qOT.rotateWithInverse( trf.velocity() ).scaleInplace( 1.0/this.distanceToTexture ) ) );
        return Vector2.fromComponents( -this.scaleX * v.x() , -this.scaleY * v.y() );
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PROTECTED CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    protected NoisyOpticalFlowSensor()
    {
        this.qTS = UnitQuaternion.one();
        this.rTTS = Vector3.zero();
        this.noiseOpticalFlow = Vector2.zero();
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PROTECTED METHODS
    ////////////////////////////////////////////////////////////////
    
    protected void updateNoise()
    {
        this.noiseOpticalFlow = Vector2.fromComponents(
                this.rnds[0].nextGaussian() ,
                this.rnds[1].nextGaussian() ).scaleInplace( this.sigmaFlow );
    }
    
}
