//rotate.c
//Chad Phillips
//Microbotics, Inc.
//Copyright 2002
//Information contained in this document is proprietary to Microbotics, Inc.
//Any use or distribution without written permission is prohibited.

#include <math.h>
#include "rotate.h"

/***********************************************************************/
/*       Operations defined on standard 3x3 rotation matrices          */
/***********************************************************************/
/* Create3x3RotYPR
 This function fills the M matrix with the appropriate values that make
 it a rotation matrix built using a yaw rotation, a pitch rotation, and
 roll rotation, in that order. The angles should be specified in radians.*/
void CreateRot_YPR(float M[3][3], float ypr[3])
  {
  float Sphi, Stheta, Spsi;
  float Cphi, Ctheta, Cpsi;

  Sphi   = sin(ypr[2]);
  Stheta = sin(ypr[1]);
  Spsi   = sin(ypr[0]);
  Cphi   = cos(ypr[2]);
  Ctheta = cos(ypr[1]);
  Cpsi   = cos(ypr[0]);

  M[0][0] =  Ctheta*Cpsi;
  M[0][1] =  Ctheta*Spsi;
  M[0][2] = -Stheta;
  M[1][0] = -Cphi*Spsi + Sphi*Stheta*Cpsi;
  M[1][1] = Cphi*Cpsi + Sphi*Stheta*Spsi;
  M[1][2] = Sphi*Ctheta;
  M[2][0] = Sphi*Spsi + Cphi*Stheta*Cpsi;
  M[2][1] = -Sphi*Cpsi + Cphi*Stheta*Spsi;
  M[2][2] = Cphi*Ctheta;
  }

/* CreateRot_Q
 Creates a rotation matrix from a quaternion.*/
void CreateRot_Q(float M[3][3], float q[4])
  {
  float x,y,z,w;
  float x2,y2,z2,w2;
  float twx, twy, twz, txy, txz, tyz;

  w=q[0];  x=q[1];  y=q[2];  z=q[3];
  w2=w*w;  x2=x*x;  y2=y*y;  z2=z*z;
  twx=2*w*x;  twy=2*w*y;  twz=2*w*z;
  txy=2*x*y;  txz=2*x*z;
  tyz=2*y*z;

  M[0][0] = w2+x2-y2-z2;
  M[0][1] = txy-twz;
  M[0][2] = txz+twy;

  M[1][0] = txy+twz;
  M[1][1] = w2-x2+y2-z2;
  M[1][2] = tyz-twx;

  M[2][0] = txz-twy;
  M[2][1] = tyz+twx;
  M[2][2] = w2-x2-y2+z2;
  }

/* MultRot_V
 This function multiplies a 3x3 matrix by a vector producing another
 vector. */
void MultRot_V(float vout[3], float M[3][3], float V[3])
  {
  vout[0] = M[0][0]*V[0] + M[0][1]*V[1] + M[0][2]*V[2];
  vout[1] = M[1][0]*V[0] + M[1][1]*V[1] + M[1][2]*V[2];
  vout[2] = M[2][0]*V[0] + M[2][1]*V[1] + M[2][2]*V[2];
  }

/* MultRot_Rot
 This function multiplies a pair of 3x3 matrices and stores the result
 as another 3x3 matrix.*/
void MultRot_Rot(float O[3][3], float L[3][3], float R[3][3])
  {
  O[0][0] = L[0][0]*R[0][0] + L[0][1]*R[1][0] + L[0][2]*R[2][0];
  O[0][1] = L[0][0]*R[0][1] + L[0][1]*R[1][1] + L[0][2]*R[2][1];
  O[0][2] = L[0][0]*R[0][2] + L[0][1]*R[1][2] + L[0][2]*R[2][2];

  O[1][0] = L[1][0]*R[0][0] + L[1][1]*R[1][0] + L[1][2]*R[2][0];
  O[1][1] = L[1][0]*R[0][1] + L[1][1]*R[1][1] + L[1][2]*R[2][1];
  O[1][2] = L[1][0]*R[0][2] + L[1][1]*R[1][2] + L[1][2]*R[2][2];

  O[2][0] = L[2][0]*R[0][0] + L[2][1]*R[1][0] + L[2][2]*R[2][0];
  O[2][1] = L[2][0]*R[0][1] + L[2][1]*R[1][1] + L[2][2]*R[2][1];
  O[2][2] = L[2][0]*R[0][2] + L[2][1]*R[1][2] + L[2][2]*R[2][2];
  }

/* CopyRot
 makes a copy of a rotation matrix. */
void CopyRot(float O[3][3], float I[3][3])
  {
  O[0][0] = I[0][0];   O[0][1] = I[0][1];   O[0][2] = I[0][2];
  O[1][0] = I[1][0];   O[1][1] = I[1][1];   O[1][2] = I[1][2];
  O[2][0] = I[2][0];   O[2][1] = I[2][1];   O[2][2] = I[2][2];
  }

/* TransposeRot
 Transposes a rotation matrix.*/
void TransposeRot(float M[3][3])
  {
  float T[3][3];

  CopyRot(T,M);
  M[0][1] = T[1][0];
  M[0][2] = T[2][0];
  M[1][0] = T[0][1];
  M[1][2] = T[2][1];
  M[2][0] = T[0][2];
  M[2][1] = T[1][2];
  }

/* SolveRotYPR
 This function uses the information in the rotation matrix M to determine
 the yaw, pitch, and roll angles that M represents. */
void SolveRot_YPR(float ypr[3], float M[3][3])
  {
  float theta, phi, psi;
  float Ctheta, absStheta;
  /*Since the pitch of an aircraft can only be between -90 and 90 degrees,
    the following will give us the correct value for theta */
  absStheta = fabs(M[0][2]);
  if(absStheta >= 1.0)
    M[0][2] /= (absStheta+1e-15);
  theta = asin(-M[0][2]);
  Ctheta = cos(theta);

  if(fabs(Ctheta) < SMALL)
    {
    /*set phi and psi to zero.*/
    phi = 0.0;
    psi = 0.0;
    }
  else
    {
    phi = atan2(M[1][2], M[2][2]);
    psi = atan2(M[0][1], M[0][0]);
    }

  ypr[0] = psi;
  ypr[1] = theta;
  ypr[2] = phi;
  }

/***********************************************************************/
/*                        Quaternion Operations                        */
/***********************************************************************/
/* Note that for all of these operations, quaternion is defined with   */
/* q[0] = w, q[1] = x, q[2] = y, q[3] = z                              */
/***********************************************************************/
/* CreateQ_Rot
 Creates a unit quarternion from a rotation matrix.*/
void CreateQ_Rot(float q[4], float M[3][3])
  {
  float  tr, s;
  int    i, j, k;
  int nxt[3] = {1, 2, 0};

  tr = M[0][0] + M[1][1] + M[2][2];
  /* check the diagonal */
  if (tr > 0.0)
    {
    s = sqrt (tr + 1.0);
    q[0] = s / 2.0;
    s = 0.5 / s;

    q[1] = (M[1][2] - M[2][1]) * s;
    q[2] = (M[2][0] - M[0][2]) * s;
    q[3] = (M[0][1] - M[1][0]) * s;
    }
  else
    {
    /* diagonal is negative */
    i = 0;
    if (M[1][1] > M[0][0]) i = 1;
    if (M[2][2] > M[i][i]) i = 2;

    j = nxt[i];
    k = nxt[j];
    s = sqrt ((M[i][i] - (M[j][j] + M[k][k])) + 1.0);
    q[i+1] = s * 0.5;
    if (s != 0.0) s = 0.5 / s;
    q[0] = (M[j][k] - M[k][j]) * s;
    q[j+1] = (M[i][j] + M[j][i]) * s;
    q[k+1] = (M[i][k] + M[k][i]) * s;
    }
  NormalizeQ(q);
  }

/* CreateQ_YPR
 Creates a unit quaternion from a yaw, pitch, and roll series of rotations.*/
void CreateQ_YPR(float q[4], float ypr[3])
  {
	float cr, cp, cy, sr, sp, sy, cpcy, spsy, yaw_2, pitch_2, roll_2;

  yaw_2 = ypr[0]*0.5;
  pitch_2 = ypr[1]*0.5;
  roll_2 = ypr[2]*0.5;

	cy = cos(yaw_2);
	cp = cos(pitch_2);
	cr = cos(roll_2);

	sy = sin(yaw_2);
	sp = sin(pitch_2);
	sr = sin(roll_2);

	cpcy = cp * cy;
	spsy = sp * sy;

	q[0] = cr * cpcy + sr * spsy;
	q[1] = sr * cpcy - cr * spsy;
	q[2] = cr * sp * cy + sr * cp * sy;
	q[3] = cr * cp * sy - sr * sp * cy;
  NormalizeQ(q);
  }

/* CopyQ
 Makes a copy of a quaternion.*/
void CopyQ(float o[4], float i[4])
  {
  o[0] = i[0];  o[1] = i[1];  o[2] = i[2];  o[3] = i[3];
  }

/* TransposeQ
 Transposes a quaternion.*/
void TransposeQ(float q[4])
  {
  q[1] = -q[1];
  q[2] = -q[2];
  q[3] = -q[3];
  }

void TransposeQ2(float q_out[4], float q[4])
  {
  q_out[0] =  q[0];
  q_out[1] = -q[1];
  q_out[2] = -q[2];
  q_out[3] = -q[3];
  }

/* NormalizeQ
 Changes a quaternion to unit length.*/
void NormalizeQ(float q[4])
  {
  float len;
  len = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if(len > SMALL)
    {
    len = 1.0/len;
    q[0] *= len;
    q[1] *= len;
    q[2] *= len;
    q[3] *= len;
    }
  }

/* MultQ_Q
 Multiplies two quaternions.
 Correct form is MultQ_Q(a2c, a2b, b2c) */
void MultQ_Q(float q[4], float l[4], float r[4])
  {
  float tq[4];
  tq[0] = l[0]*r[0] - l[1]*r[1] - l[2]*r[2] - l[3]*r[3];
  tq[1] = l[1]*r[0] + l[0]*r[1] - l[3]*r[2] + l[2]*r[3];
  tq[2] = l[2]*r[0] + l[3]*r[1] + l[0]*r[2] - l[1]*r[3];
  tq[3] = l[3]*r[0] - l[2]*r[1] + l[1]*r[2] + l[0]*r[3];
  q[0] = tq[0];
  q[1] = tq[1];
  q[2] = tq[2];
  q[3] = tq[3];
  }

void MultQ_Qc(float q[4], float l[4], float r[4])  //added to match DSP code
  {
  MultQ_Q(q, l, r);
  }

/* MultQ_V
 Multiplies a quaternion and a vector effectively rotating the vector
 through the transformation perscribed by the quaternion.*/
void MultQ_V(float vout[3], float q[4], float vin[3])
  {
  float a,b,c;
  c = 2*q[0];
  a = c*q[0]-1.0;
  b = 2*(q[1]*vin[0]+q[2]*vin[1]+q[3]*vin[2]);
  vout[0] = a*vin[0] + b*q[1] + c*(q[2]*vin[2] - q[3]*vin[1]);
  vout[1] = a*vin[1] + b*q[2] + c*(q[3]*vin[0] - q[1]*vin[2]);
  vout[2] = a*vin[2] + b*q[3] + c*(q[1]*vin[1] - q[2]*vin[0]);
  }

void MultQ_Vc(float vout[3], float q[4], float vin[3])  //added to match DSP code
  {
  float tmp[3];
  MultQ_V(tmp, q, vin);
  vout[0] = tmp[0];
  vout[1] = tmp[1];
  vout[2] = tmp[2];
  }

/* SolveQ_YPR
 Solves a quaternion for the yaw, pitch, and roll angles that it is
 specified by.*/
void SolveQ_YPR(float ypr[3], float q[4])
  {
  float w2 = q[0]*q[0];
  float x2 = q[1]*q[1];
  float y2 = q[2]*q[2];
  float z2 = q[3]*q[3];
  float f;

  ypr[0] = atan2(2.0 * (q[1]*q[2] + q[3]*q[0]),(x2 - y2 - z2 + w2));
  f = -2.0 * (q[1]*q[3] - q[2]*q[0]);
  if(f > 1.0)
    f = 1.0;
  else if(f < -1.0)
    f = -1.0;
  ypr[1] = asin(f);
  ypr[2] = atan2(2.0 * (q[2]*q[3] + q[1]*q[0]),(-x2 - y2 + z2 + w2));
  }

void PropagateQ(float q[4], float pqr[3], float dt)
  {
  float qnew[4];

  qnew[0] = q[0] + dt*0.5*(-q[1]*pqr[0] - q[2]*pqr[1] - q[3]*pqr[2]);
  qnew[1] = q[1] + dt*0.5*( q[0]*pqr[0] - q[3]*pqr[1] + q[2]*pqr[2]);
  qnew[2] = q[2] + dt*0.5*( q[3]*pqr[0] + q[0]*pqr[1] - q[1]*pqr[2]);
  qnew[3] = q[3] + dt*0.5*(-q[2]*pqr[0] + q[1]*pqr[1] + q[0]*pqr[2]);

  q[0] = qnew[0];
  q[1] = qnew[1];
  q[2] = qnew[2];
  q[3] = qnew[3];
  NormalizeQ(q);
  }


