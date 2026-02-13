package sensorSimulationLibrary.sensors.pose;


import java.util.Random;

import numericalLibrary.manifolds.unitQuaternions.atlases.ExponentialMapS3;
import numericalLibrary.manifolds.unitQuaternions.atlases.RodriguesParametersS3;
import numericalLibrary.manifolds.unitQuaternions.atlases.UnitQuaternionDifferentiableAtlas;
import numericalLibrary.types.UnitQuaternion;
import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.trajectories3d.ContinuousTrajectory3dOfReferenceFrame;
import sensorSimulationLibrary.util.SeedManagerForRandomNumberGenerator;
import util.Pose;



public class NoisyPoseIncrementSensor
{
    ////////////////////////////////////////////////////////////////
    /// PROTECTED VARIABLES
    ////////////////////////////////////////////////////////////////
    private UnitQuaternionDifferentiableAtlas unitQuaternionAtlas;
    
    /**
     * {@link UnitQuaternion} that defines the rotation transformation from sensor to trajectory frame.
     */
    private UnitQuaternion qTS;
    
    /**
     * {@link Vector3} that defines the position of the sensor relative to the trajectory frame.
     */
    private Vector3 rTTS;
    
    
    protected Vector3 noiseOrientation;
    protected Vector3 noisePosition;
    
    ////////////////////////////////////////////////////////////////
    /// PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private Random[] rnds;
    private double sigmaOrientation;  // the standard deviation of the orientation noise
    private double sigmaPosition;  // the standard deviation of the position noise

	private double timeIncrement;
    
    
    
    ////////////////////////////////////////////////////////////////
    /// PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public NoisyPoseIncrementSensor( SeedManagerForRandomNumberGenerator sm )
    {
        this();  // we call the private constructor
        this.unitQuaternionAtlas = new ExponentialMapS3();
        this.rnds = new Random[6];
        for(int i=0; i<this.rnds.length; i++) {
            this.rnds[i] = new Random();
            this.rnds[i].setSeed( sm.nextSeed() );
        }
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    /// PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public void setTimeIncrement( double timeIncrement )
    {
    	this.timeIncrement = timeIncrement;
    }
    
    
    public void setSensorOrientationToTrajectoryFrame( UnitQuaternion sensorOrientationToTrajectoryFrame )
    {
        this.qTS.setTo( sensorOrientationToTrajectoryFrame );
    }
    
    
    public void setSensorPositionFromTrajectoryFrame( Vector3 sensorPositionFromTrajectoryFrame )
    {
        this.rTTS.setTo( sensorPositionFromTrajectoryFrame );
    }
    
    
    public void setMeasurementSigmaOrientationAndSigmaPosition( double sigmaOrientation , double sigmaPosition )
    {
        this.sigmaOrientation = sigmaOrientation;
        this.sigmaPosition = sigmaPosition;
    }
    
    
    public Pose measure( ContinuousTrajectory3dOfReferenceFrame trf , double time )
    {
        this.updateNoise();
        // Get initial pose.
        trf.setTime( time - this.timeIncrement );
        UnitQuaternion qOTi = trf.orientation();
        UnitQuaternion qOSi = qOTi.multiply( this.qTS );
        Vector3 rOOTi = trf.position();
        Vector3 rOOSi = rOOTi.add( qOTi.rotate( this.rTTS ) );
        // Get final pose.
        trf.setTime( time );
        UnitQuaternion qOTf = trf.orientation();
        UnitQuaternion qOSf = qOTf.multiply( this.qTS );
        Vector3 rOOTf = trf.position();
        Vector3 rOOSf = rOOTf.add( qOTf.rotate( this.rTTS ) );
        // Compute increments.
        UnitQuaternion qSiO = qOSi.inverseMultiplicativeInplace();
        UnitQuaternion qSiSf = qSiO.multiply( qOSf );
        Vector3 rSiSiSf = qSiO.rotateInplace( rOOSf.subtractInplace( rOOSi ) );
        // Add noise.
        this.unitQuaternionAtlas.setChartSelector( qSiSf );
        UnitQuaternion qSiSf_disturbed = this.unitQuaternionAtlas.toManifold( this.noiseOrientation );
        Vector3 rSiSiSf_disturbed = rSiSiSf.addInplace( this.noisePosition );
        return new Pose( qSiSf_disturbed , rSiSiSf_disturbed );
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    /// PRIVATE CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    private NoisyPoseIncrementSensor()
    {
        this.qTS = UnitQuaternion.one();
        this.rTTS = Vector3.zero();
        this.noiseOrientation = Vector3.zero();
        this.noisePosition = Vector3.zero();
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    /// PRIVATE METHODS
    ////////////////////////////////////////////////////////////////
    
    private void updateNoise()
    {
        this.noiseOrientation = Vector3.fromComponents(
                this.rnds[0].nextGaussian() ,
                this.rnds[1].nextGaussian() ,
                this.rnds[2].nextGaussian() ).scaleInplace( this.sigmaOrientation );
        this.noisePosition = Vector3.fromComponents(
                this.rnds[3].nextGaussian() ,
                this.rnds[4].nextGaussian() ,
                this.rnds[5].nextGaussian() ).scaleInplace( this.sigmaPosition );
    }
    
}
