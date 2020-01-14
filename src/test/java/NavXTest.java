import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * Test for checking the validity of the NavX calculations
 * 
 * @author Will Richards
 */
public class NavXTest{

    /**
     * Checks if the values are correct for the edge cases
     */
    @Test
    public void edgeCaseTest(){
        double[] expected = {1.0 , -1,   1,   -1,  -2,   2};
        double[] angle1 =   {89.0, 90, 360,    0, 181, 179};
        double[] angle2 =   {90, 89,   1,  359, 179, 181};

        //Loop through test cases
        for (int i=0; i<expected.length;i++){
            double calculated = (((((angle2[i] - angle1[i]) + 180)+360) % 360) - 180);
            assertEquals(expected[i], calculated, 0.1); 
        }

        
    }
}