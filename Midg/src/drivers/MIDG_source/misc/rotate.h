//rotate.h
//Chad Phillips
//Microbotics, Inc.
//Copyright 2002
//Information contained in this document is proprietary to Microbotics, Inc.
//Any use or distribution without written permission is prohibited.

#ifndef _Rotate_Header_
#define _Rotate_Header_

#define D2R 0.0174532925199
#define R2D 57.2957795131
#define D2RL 0.0174532925199L
#define R2DL 57.2957795131L
#define SMALL 1e-20

#ifdef __cplusplus
extern "C" {
#endif

/* Rotation Matrix Operations */
void CreateRot_YPR(float M[3][3], float ypr[3]);
void CreateRot_Q(float M[3][3], float q[4]);
void SolveRot_YPR(float ypr[3], float M[3][3]);
void CopyRot(float O[3][3], float I[3][3]);
void TransposeRot(float M[3][3]);
void MultRot_V(float vout[3], float M[3][3], float V[3]);
void MultRot_Rot(float O[3][3], float L[3][3], float R[3][3]);

/* Quaternion Operations */
void CreateQ_Rot(float q[4], float M[3][3]);
void CreateQ_YPR(float q[4], float ypr[3]);
void CopyQ(float o[4], float i[4]);
void TransposeQ(float q[4]);
void TransposeQ2(float q_out[4], float q[4]);
void NormalizeQ(float q[4]);
void MultQ_Q(float q[4], float l[4], float r[4]);
void MultQ_Qc(float q[4], float l[4], float r[4]);
void MultQ_V(float vout[3], float q[4], float vin[3]);
void MultQ_Vc(float vout[3], float q[4], float vin[3]);
void SolveQ_YPR(float ypr[3], float q[4]);
void PropagateQ(float q[4], float pqr[3], float dt);

#ifdef __cplusplus
}
#endif
#endif

