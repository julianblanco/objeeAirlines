void cumtrapz (float* time,float* inmatrix,float* outmatrix   ){

//Implementing cumptrapz***********************************************
	int i, j;
	int m =10;int n=3;
	float tempmat[10][3];

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			tempmat[n * i + j] = time[0][i+1]*(inmatrix[n * i + j]+inmatrix[n * i + j]);
		}
		float cumsum = 0;
		float cumsum1 = 0;
		float cumsum2 = 0;
		for ( int i = 0; i < 10; i++ ){

			
			cumsum += tempmat[i][0];
			cumsum1 += tempmat[i][1];
			cumsum2 += tempmat[i][2];		

			outmatrix[i][0]=cumsum;
			outmatrix[i][1]=cumsum1;
			outmatrix[i][2]=cumsum2;

		}


}