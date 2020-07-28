/*
 * prediction.c
 *      Author: Fernando Aguilar
 */

/*Includes files and libraries  */
#include "prediction.h"
#include <math.h>
#include <stdlib.h>

float** matrixConverter(int m, int n, const float* dt)
{
	int i,j,k;
	float **matrix;

	/*Reserve memory  */
	matrix = (float**)malloc(m*sizeof(float*));

	k = 0;

	/*Cycles for to convert the array dt in a Matrix of size mxn and reserve memory to columns */
	for(i = 0; i < m; i++)
	{
		matrix[i] = (float*)malloc(n*sizeof(float));
		for(j = 0; j < n; j++)
		{
			matrix[i][j] = dt[j + k];
		}
		k = k + n;
	}

	return matrix;
}


float** matrixTranspossed(int tags, int n, float** theta_m)
{
	int i,j;
	float **matrix_t;

	/*Reserve memory  */
	matrix_t = (float**)malloc(n*sizeof(float*));

	/*Cycles for to compute the transposed matrix of other, and reserve memory to columns */
	for(i = 0; i < n; i++)
	{
		matrix_t[i] = (float*)malloc(tags*sizeof(float));
		for(j = 0; j < tags; j++)
		{
			matrix_t[i][j] = theta_m[j][i];
		}
	}

	return matrix_t;
}

float** matrixProduct(int m, int n, int tags, float** theta_tr, float** X)
{
	int i,j,a;
	float **matrix_product;
	float sum;

	/* Reserve memory */
	matrix_product = (float**)malloc(m*sizeof(float*));

	/*Cycles for to compute the product of two arrays, and reserve memory to columns */
	for(i = 0; i < m; i++)
	{
		matrix_product[i] = (float*)malloc(tags*sizeof(float));
		for(j = 0; j < tags; j++)
		{
			sum = 0;
			for(a = 0; a < n; a++)
			{
				sum += X[i][a] * theta_tr[a][j];
			}
			matrix_product[i][j] = sum;
		}
	}


	return matrix_product;
}



float** computeSigmoid(float** X_theta, int m, int tags)
{
	int i,j;
	float **matrix_sigmoid;

	/* Reserv memory */
	matrix_sigmoid = (float**)malloc(m*sizeof(float*));

	/*Cycles for to compute the sigmoid of each element using the sigmoid function in each iteration, and reserve memory to columns */
	for(i = 0; i < m; i++)
	{
		matrix_sigmoid[i] = (float*)malloc(tags*sizeof(float));
		for(j = 0; j < tags; j++)
		{
			matrix_sigmoid[i][j] = sigmoid(X_theta[i][j]);
		}
	}
	return matrix_sigmoid;
}


int* computePrediction(float** matrixSigmoid, int m, int tags)
{
	int *prediction;

	/* reserve memory */
	prediction = (int*)malloc(m*sizeof(int));
	int i, j;
	double max;

	/*Cycles for to calcule the values of P getting the value max in each row */
	for(i = 0; i<m; i++)
	{
		max = 0;
		for(j = 0; j < tags; j++)
		{
			if(matrixSigmoid[i][j] > max)
			{
				prediction[i] = j+1;
				max = matrixSigmoid[i][j];
			}
		}
	}

	return prediction;
}


float sigmoid(float z)
{
	float y;
	/* function sigmoid */
	y = 1/(1 + pow(E, -z));
	return y;
}



