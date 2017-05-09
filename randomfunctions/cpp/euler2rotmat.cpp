
void eulerAnglesToRotationMatrix(float R[3][3],float theta[3])
{
    // Calculate rotation about x axis
    float R_x[3][3];

    R_x[0][0]  =  1;  
    R_x[0][1]  =  0;  
    R_x[0][1]  =  0;
    R_x[1][0]  =  0;
    R_x[1][1]  =  cos(theta[0]);  
    R_x[1][2]  =  -sin(theta[0]);
    R_x[2][0]  =  0;
    R_x[2][1]  =  sin(theta[0]);
    R_x[2][2]  =  cos(theta[0]);
    // Matrix.Print((float*)R_x, 3, 3, "Rot Mat");

    
     
    // Calculate rotation about y axis
    float R_y[3][3];

    R_y[0][0] = cos(theta[1]);
    R_y[0][1] = 0;
    R_y[0][2] = sin(theta[1]);
    R_y[1][0] = 0;
    R_y[1][1] = 1;    
    R_y[1][2] = 0;
    R_y[2][0] = -sin(theta[1]); 
    R_y[2][1] = 0;
    R_y[2][2] = cos(theta[1]);
    // Matrix.Print((float*)R_y, 3, 3, "Rot Mat");

     
    // Calculate rotation about z axis
    float R_z[3][3];

    R_z[0][0] =cos(theta[2]),  
    R_z[0][1] =-sin(theta[2]),  
    R_z[0][2] =0;
    R_z[1][0] =sin(theta[2]); 
    R_z[1][1] =cos(theta[2]);
    R_z[1][2] =0;
    R_z[2][0] =0;
    R_z[2][1] =0;
    R_z[2][2] =1;
    // Matrix.Print((float*)R_z, 3, 3, "Rot Mat");

     
    float temp[3][3];

    Matrix.Multiply((float*)R_z, (float*)R_y, 3, 3, 3, (float*)temp);
    // Matrix.Print((float*)temp, 3, 3, "Rot Mat");

    Matrix.Multiply((float*)R_x, (float*)temp, 3, 3, 3, (float*)R);
     
}