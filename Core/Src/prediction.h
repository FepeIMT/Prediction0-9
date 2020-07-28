/*
 * prediction.h
 *      Author: Fernando Aguilar
 */

#ifndef _PREDICTION_H_
#define _PREDICTION_H_

/*Define M that is the number of examples to evaluate */
#ifndef M
#define M 	30
#endif

/*Define N that is the feature numbers in this case is 401 because we have data from number images of 20x20 pixels + X0 value*/
#ifndef N
#define N 	401
#endif

/* Define TAGS that is the number of the multiple classifications in this case is the number 0-9 */
#ifndef TAGS
#define TAGS 10
#endif

/*Define the constant E to use in the sigmoid function */
#ifndef E
#define E 2.7182818284
#endif

/*
 * brief: convert data to Matriz
 * parameters: int n, m
 * retval: Matrix size mxn
 *
 */
float** matrixConverter(int m, int n, const float* dt);


/*
 * brief: Compute transpossed matrix
 * parameters: int m,n.
 * retval: Matrix transpossed size mxn
 *
 */
float** matrixTranspossed(int tags, int n, float** theta_m);


/*
 * brief: Compute product of two matrix
 * parameters: int m, n.
 * retval: Product of matrix X*theta'
 *
 */
float** matrixProduct(int m, int n, int tags, float**  theta_tr, float** X);


/*
 * brief: Compute sigmoid of each elemnt of the matrix theta*X
 * parameters: Matrix X*theta, int m, int tags
 * retval: Matrix with the sigmoid compute of each element
 *
 */
float** computeSigmoid(float** X_theta, int m, int tags);


/*
 * brief: Compute the array P with the prediction in each example
 * parameters: Matrix sigmoid, int m, int prediction[]
 * retval: Matrix with the sigmoid compute of each element
 */
int* computePrediction(float** matrixSigmoid, int m, int tags);

/*
 * brief: Compute sigmoid value.
 * parameters: value to cumpute the sigmoid function
 * retval: sigmoid value
 *
 */
float sigmoid(float z);



#endif /* _PREDICTION_H_ */


