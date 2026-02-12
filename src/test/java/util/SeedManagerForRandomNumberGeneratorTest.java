package util;


import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import sensorSimulationLibrary.util.SeedManagerForRandomNumberGenerator;




class SeedManagerForRandomNumberGeneratorTest
{
    
    
    @Test
    void nextSeedProducesSequenceOfIntegers()
    {
        SeedManagerForRandomNumberGenerator sm = new SeedManagerForRandomNumberGenerator();
        int counter = 0;
        for(int i=0; i<100; i++) {
            assertEquals( counter , sm.nextSeed() );
            counter++;
        }
    }
    
    
    @Test
    void setUsedTwiceThrowsException() throws Exception
    {
        SeedManagerForRandomNumberGenerator sm = new SeedManagerForRandomNumberGenerator();
        sm.setUsed( 7 );
        Exception exception = assertThrows( Exception.class , () -> {
            sm.setUsed( 7 );
        });
    }
    
    
    @Test
    void nextSeedAndSetUsedWorkTogether() throws Exception
    {
        SeedManagerForRandomNumberGenerator sm = new SeedManagerForRandomNumberGenerator();
        sm.setUsed( 7 );
        for(int i=0; i<10; i++) {
            sm.nextSeed();
        }
    }
    
    

}
