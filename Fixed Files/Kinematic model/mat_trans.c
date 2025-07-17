/* Version of the  03 February 2016 */
/* Information from .urdf of July 2015 */

#define TRUE 1
#define FALSE 0
#define NJoints 25
#define Real double
#include "mex.h"
#include <math.h>
#define pi 3.14159265358979323846
#define Set_Val_Matrix(Matrix, NRow, Row, Col, Value) *((Matrix) + ((Row)-1) + ((Col)-1)*NRow ) = (Value)


/* Geometric parameters */
#define d4 -0.1
#define d5 -0.1029
#define d10 -0.1
#define d11 -0.1029
#define d25 0.030
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#define r1  -0.07071
#define r7  0.07071
//#define r13 -0.233
#define r13 0
#define r15 0.105
#define r17 0.05595
//#define r18 -0.037
#define r18 0
#define r20 0.105
#define r22 0.05595
//#define r23 -0.037
#define r23 0
//#define r25 0.105
#define r25 0
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
//b
//#define b13 0.19091
#define b13 0
//#define b18 0.19091 
#define b18 0 
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
//gamma
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
//alpha

/* Headers */
void	mat_trans     (Real *, Real *);

/* Utilities */
void	Zeros                    (Real *, int, int);


/* Global variables */


/* Interface function: This function will be called as:
 
[T] = mat_trans( Q );
 
 */
void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] )
{
Real Zeros_3x1[] = {0,0,0};
Real *Q;
Real *T;
/* Check for proper number of arguments */

/* Getting input arguments */
Q           = mxGetPr(prhs[0]);

    
/* Memory allocation for output arguments */
plhs[0]     = mxCreateDoubleMatrix(4*NJoints,4, mxREAL);
T      = mxGetPr(plhs[0]);


/* Calcul des matrices de transformation */
mat_trans( Q , T );
}


void mat_trans(Real *Q, Real *T)
{
int j; 
int im;
/* Geometric parameters */
Real gam [25] =   {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // 
Real b [25] =     {0,0,0,0,0,0,0,0,0,0,0,0,b13,0,0,0,0,b18,0,0,0,0,0,0,0};
Real r [25] =     {r1,0,0,0,0,0,r7,0,0,0,0,0,r13,0,r15,0,r17,r18,0,r20,0,r22,r23,0,r25};
Real d [25] =     {0,0,0,d4,d5,0,0,0,0,d10,d11,0,0,0,0,0,0,0,0,0,0,0,0,0,d25};
//Real alpha [25] = {0,pi/2,pi/2,0,0,-pi/2,-pi/2,-pi/2,pi/2,0,0,-pi/2,-pi/4,pi/2,pi/2,-pi/2,pi/2,-pi/4,pi/2,pi/2,-pi/2,pi/2,pi/4,-pi/2,0};
Real alpha [25] = {0,pi/2,pi/2,0,0,-pi/2,-pi/2,-pi/2,pi/2,0,0,-pi/2,-pi/2,pi/2,pi/2,-pi/2,pi/2,pi/2,pi/2,pi/2,-pi/2,pi/2,0,-pi/2,0};

/* Transformations */
Zeros(T,NJoints*4,4);
im=1;
for (j = 0 ; j < NJoints ; j++){
     Set_Val_Matrix(T,NJoints*4,im,1,cos(gam[j])*cos(Q[j])-sin(gam[j])*cos(alpha[j])*sin(Q[j])); 
     Set_Val_Matrix(T,NJoints*4,im,2,-cos(gam[j])*sin(Q[j])-sin(gam[j])*cos(alpha[j])*cos(Q[j]));
     Set_Val_Matrix(T,NJoints*4,im,3,sin(gam[j])*sin(alpha[j]));
     Set_Val_Matrix(T,NJoints*4,im,4,d[j]*cos(gam[j])+r[j]*sin(gam[j])*sin(alpha[j])); 
     Set_Val_Matrix(T,NJoints*4,im+1,1,sin(gam[j])*cos(Q[j])+cos(gam[j])*cos(alpha[j])*sin(Q[j]));  
     Set_Val_Matrix(T,NJoints*4,im+1,2,-sin(gam[j])*sin(Q[j])+cos(gam[j])*cos(alpha[j])*cos(Q[j]));
     Set_Val_Matrix(T,NJoints*4,im+1,3,-cos(gam[j])*sin(alpha[j]));
     Set_Val_Matrix(T,NJoints*4,im+1,4,d[j]*sin(gam[j])-r[j]*cos(gam[j])*sin(alpha[j]));
     Set_Val_Matrix(T,NJoints*4,im+2,1,sin(alpha[j])*sin(Q[j]));
     Set_Val_Matrix(T,NJoints*4,im+2,2,sin(alpha[j])*cos(Q[j]));
     Set_Val_Matrix(T,NJoints*4,im+2,3,cos(alpha[j]));
     Set_Val_Matrix(T,NJoints*4,im+2,4,r[j]*cos(alpha[j])+b[j]);
     Set_Val_Matrix(T,NJoints*4,im+3,4,1);  
     im=im+4;
     
 }                 
}

/* Creates a "Rows" by "Cols" zero matrix */
void Zeros(Real *Matrix, int Rows, int Cols )
{
  int i;
  for(i=1; i <= Rows * Cols; i++)
        *Matrix++ = 0;
}





