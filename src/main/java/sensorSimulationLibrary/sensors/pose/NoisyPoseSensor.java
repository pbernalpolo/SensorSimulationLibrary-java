package sensorSimulationLibrary.sensors.pose;


import java.util.Random;

import numericalLibrary.manifolds.unitQuaternions.atlases.RodriguesParametersS3;
import numericalLibrary.manifolds.unitQuaternions.atlases.UnitQuaternionDifferentiableAtlas;
import numericalLibrary.types.UnitQuaternion;
import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.trajectories3d.ContinuousTrajectory3dOfReferenceFrame;
import sensorSimulationLibrary.util.SeedManagerForRandomNumberGenerator;
import util.Pose;



public class NoisyPoseSensor
{
    ////////////////////////////////////////////////////////////////
    // PROTECTED VARIABLES
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
    
    /**
     * {@link UnitQuaternion} that defines the rotation transformation from origin to sensor origin.
     */
    private UnitQuaternion qZO;
    
    /**
     * {@link Vector3} that defines the position of the origin relative to the sensor origin.
     */
    private Vector3 rZZO;
    
    
    protected Vector3 noiseOrientation;
    protected Vector3 noisePosition;
    
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private Random[] rnds;
    private double sigmaOrientation;  // the standard deviation of the orientation noise
    private double sigmaPosition;  // the standard deviation of the position noise
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public NoisyPoseSensor( SeedManagerForRandomNumberGenerator sm )
    {
        this();  // we call the private constructor
        this.unitQuaternionAtlas = new RodriguesParametersS3();
        this.rnds = new Random[6];
        for(int i=0; i<this.rnds.length; i++) {
            this.rnds[i] = new Random();
            this.rnds[i].setSeed( sm.nextSeed() );
        }
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public void setOriginOrientationToSensorOrigin( UnitQuaternion originOrientationFromSensorOrigin )
    {
        this.qZO = originOrientationFromSensorOrigin;
    }
    
    
    public void setOriginPositionFromSensorOrigin( Vector3 originPositionFromSensorOrigin )
    {
        this.rZZO = originPositionFromSensorOrigin;
    }
    
    
    public void setSensorOrientationToTrajectoryFrame( UnitQuaternion sensorOrientationToTrajectoryFrame )
    {
        this.qTS = sensorOrientationToTrajectoryFrame;
    }
    
    
    public void setSensorPositionFromTrajectoryFrame( Vector3 sensorPositionFromTrajectoryFrame )
    {
        this.rTTS = sensorPositionFromTrajectoryFrame;
    }
    
    
    public void setMeasurementSigmaOrientationAndSigmaPosition( double sigmaOrientation , double sigmaPosition )
    {
        this.sigmaOrientation = sigmaOrientation;
        this.sigmaPosition = sigmaPosition;
    }
    
    
    public Pose measure( ContinuousTrajectory3dOfReferenceFrame trf , double time )
    {
        this.updateNoise();
        trf.setTime( time );
        UnitQuaternion qOT = trf.orientation();
        UnitQuaternion qZS = this.qZO.multiply( qOT ).multiply( this.qTS );
        this.unitQuaternionAtlas.setChartSelector( qZS );
        UnitQuaternion qZS_disturbed = this.unitQuaternionAtlas.toManifold( this.noiseOrientation );
        Vector3 rOOT = trf.position();
        Vector3 rZZS = this.qZO.rotate( qOT.rotate( this.rTTS ).addInplace( rOOT ) ).addInplace( this.rZZO );
        Vector3 rZZS_disturbed = rZZS.addInplace( this.noisePosition );
        return new Pose( qZS_disturbed , rZZS_disturbed );
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PROTECTED CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    protected NoisyPoseSensor()
    {
        this.qTS = UnitQuaternion.one();
        this.rTTS = Vector3.zero();
        this.qZO = UnitQuaternion.one();
        this.rZZO = Vector3.zero();
        this.noiseOrientation = Vector3.zero();
        this.noisePosition = Vector3.zero();
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PROTECTED METHODS
    ////////////////////////////////////////////////////////////////
    
    protected void updateNoise()
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
