/**********************************************************************
 * Software Copyright Licensing Disclaimer
 *
 * This software module was originally developed by contributors to the
 * course of the development of ISO/IEC 14496-10 for reference purposes
 * and its performance may not have been optimized.  This software
 * module is an implementation of one or more tools as specified by
 * ISO/IEC 14496-10.  ISO/IEC gives users free license to this software
 * module or modifications thereof. Those intending to use this software
 * module in products are advised that its use may infringe existing
 * patents.  ISO/IEC have no liability for use of this software module
 * or modifications thereof.  The original contributors retain full
 * rights to modify and use the code for their own purposes, and to
 * assign or donate the code to third-parties.
 *
 * This copyright notice must be included in all copies or derivative
 * works.  Copyright (c) ISO/IEC 2004.
 **********************************************************************/
/*
***********************************************************************
* COPYRIGHT AND WARRANTY INFORMATION
*
* Copyright 2001, International Telecommunications Union, Geneva
*
* DISCLAIMER OF WARRANTY
*
* These software programs are available to the user without any
* license fee or royalty on an "as is" basis. The ITU disclaims
* any and all warranties, whether express, implied, or
* statutory, including any implied warranties of merchantability
* or of fitness for a particular purpose.  In no event shall the
* contributor or the ITU be liable for any incidental, punitive, or
* consequential damages of any kind whatsoever arising from the
* use of these programs.
*
* This disclaimer of warranty extends to the user of these programs
* and user's customers, employees, agents, transferees, successors,
* and assigns.
*
* The ITU does not represent or warrant that the programs furnished
* hereunder are free of infringement of any third-party patents.
* Commercial implementations of ITU-T Recommendations, including
* shareware, may be subject to royalty fees to patent holders.
* Information regarding the ITU-T patent policy is available from
* the ITU Web site at http://www.itu.int.
*
* THIS IS NOT A GRANT OF PATENT RIGHTS - SEE THE ITU-T PATENT POLICY.
************************************************************************
*/

/*!
 ***************************************************************************
 * \file
 *    ratectl.h
 *
 * \author
 *    Zhengguo LI
 *
 * \date
 *    14 Jan 2003
 *
 * \brief
 *    Headerfile for rate control 
 **************************************************************************
 */
#ifndef _RATE_CTL_H_
#define _RATE_CTL_H_


#define MIN(a,b)  (((a)<(b)) ? (a) : (b))//LIZG 28/10/2002
#define MAX(a,b)  (((a)<(b)) ? (b) : (a))//LIZG 28/10/2002

double bit_rate; 
double frame_rate;
double GAMMAP;//LIZG, JVT019r1
double BETAP;//LIZG, JVT019r1

int RC_MAX_QUANT;//LIZG 28/10/2002
int RC_MIN_QUANT;//LIZG 28/10/2002

double BufferSize; //LIZG 25/10/2002
double GOPTargetBufferLevel;
double CurrentBufferFullness; //LIZG 25/10/2002		// 虚拟缓冲满度实际值
double TargetBufferLevel;//LIZG 25/10/2002		// 虚拟缓冲满度目标值
double PreviousBit_Rate;//LIZG  25/10/2002
double AWp;
double AWb;
int MyInitialQp;
int PAverageQp;

/*LIZG JVT50V2 distortion prediction model*/
/*coefficients of the prediction model*/
double PreviousPictureMAD;
double MADPictureC1;	// MAD 线性预测模型第一个系数
double MADPictureC2;	// MAD 线性预测模型第二个系数
double PMADPictureC1;	// 参数值传递过程中的中间临时变量，可直接用 MADPictureC1 替换
double PMADPictureC2;	// 参数值传递过程中的中间临时变量，可直接用 MADPictureC2 替换
/* LIZG JVT50V2 picture layer MAD */
Boolean PictureRejected[21];
double PPictureMAD[21];	// 参数值传递过程中的中间临时变量，可直接用 PictureMAD 替换
double PictureMAD[21];	// 当前图像所有基本单元实际 MAD 值的逆序排列（最近编码的基本单元在最小下标）
double ReferenceMAD[21];	// 前一图像所有基本单元实际 MAD 值的逆序排列（最近编码的基本单元在最小下标）

/*quadratic rate-distortion model*/
Boolean   m_rgRejected[21];
double  m_rgQp[21];		// FIFO 队列用来存储各个基本单元的量化步长
double m_rgRp[21];		// FIFO 队列用来存储各个基本单元编码完成后的二次方程左边项
double m_X1;	// 二次模型第一个系数
double m_X2;	// 二次模型第二个系数
int m_Qc;	// 当前基本单元量化参数
double m_Qstep;
int m_Qp;	// 一个中间临时变量
int Pm_Qp;	// 一个中间临时变量
int PreAveMBHeader;
int CurAveMBHeader;
int PPreHeader;		// 前一图像头比特数（只在帧级 RC 时才会使用。在基本单元级 RC 时，该值虽然被赋值，但未被使用）
int PreviousQp1;
int PreviousQp2;
int NumberofBFrames;
/*basic unit layer rate control*/
int TotalFrameQP;	// 当前图像所有基本单元量化参数之和
int NumberofBasicUnit;
int PAveHeaderBits1;	// 当前图像中所有已编码基本单元的头比特数均值
int PAveHeaderBits2;	// 当前图像中下一个待编码基本单元头比特数估计值
int PAveHeaderBits3;	// 前一图像所有基本单元平均头比特数
int PAveFrameQP;
int TotalNumberofBasicUnit;
int CodedBasicUnit;
double MINVALUE;
double CurrentFrameMAD;
double CurrentBUMAD;
double TotalBUMAD;
double PreviousFrameMAD;
int m_Hp;
int m_windowSize;	// 前一次使用的二次模型参数更新窗口大小
int MADm_windowSize;	// 前一次使用的 MAD 线性模型参数更新窗口大小
int DDquant;
int MBPerRow;
double AverageMADPreviousFrame;
int TotalBasicUnitBits;
int QPLastPFrame;
int QPLastGOP;
//int MADn_windowSize;
//int n_windowSize;

double Pm_rgQp[20];	// 参数值传递过程中的中间临时变量，可直接用 m_rgQp 替换
double Pm_rgRp[20];	// 参数值传递过程中的中间临时变量，可直接用 m_rgRp 替换
double Pm_X1;	// 参数值传递过程中的中间临时变量，可直接用 m_X1 替换
double Pm_X2;	// 参数值传递过程中的中间临时变量，可直接用 m_X2 替换
int Pm_Hp;		// 本版本 JM 中该变量无用
/* adaptive field/frame coding*/
int FieldQPBuffer;
int FrameQPBuffer;
int FrameAveHeaderBits;
int FieldAveHeaderBits;
double BUPFMAD[6336];//LIZG		// 前一幅解码图像各基本单元实际 MAD 值
double BUCFMAD[6336];//LIZG		// 当前解码图像各基本单元实际 MAD 值
double FCBUCFMAD[6336];
double FCBUPFMAD[6336];

Boolean GOPOverdue;


//comput macroblock activity for rate control
int diffy[16][16];
int diffyy[16][16];
int diffy8[16][16];//for P8*8 mode 

extern int Iprev_bits;
extern int Pprev_bits;

void rc_init_seq();
void rc_init_GOP(int np, int nb);
void rc_update_pict_frame(int nbits);
void rc_init_pict(int fieldpic,int topfield, int targetcomputation);
void rc_update_pict(int nbits);
void setbitscount(int nbits);

int updateQuantizationParameter(int topfield);/*LIZG*/
void updateRCModel ();/*LIZG*/
void updateMADModel ();/*LIZG*/
Boolean skipThisFrame (); /*LIZG*/
void RCModelEstimator (int n_windowSize);/*LIZG*/
void MADModelEstimator (int n_windowSize);/*LIZG*/
double calc_MAD();/*LIZG*/
double ComputeFrameMAD();
int Qstep2QP( double Qstep );
double QP2Qstep( int QP );

#endif

