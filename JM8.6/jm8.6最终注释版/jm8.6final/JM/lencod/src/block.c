/**********************************************************************
 * Software Copyright Licensing Disclaimer
 *已读
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

/*!
 *************************************************************************************
 * \file block.c
 *
 * \brief
 *    Process one block
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Inge Lille-Lang鴜               <inge.lille-langoy@telenor.com>
 *    - Rickard Sjoberg                 <rickard.sjoberg@era.ericsson.se>
 *    - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 *    - Jani Lainema                    <jani.lainema@nokia.com>
 *    - Detlev Marpe                    <marpe@hhi.de>
 *    - Thomas Wedi                     <wedi@tnt.uni-hannover.de>
 *    - Ragip Kurceren                  <ragip.kurceren@nokia.com>
 *    - Greg Conklin                    <gregc@real.com>
 *************************************************************************************
 */

#include "contributors.h"


#include <math.h>
#include <stdlib.h>
#include <assert.h>

#include "block.h"
#include "refbuf.h"
#include "vlc.h"
#include "mb_access.h"
#include "image.h"


#define Q_BITS          15
#define DQ_BITS         6
#define DQ_ROUND        (1<<(DQ_BITS-1))


static const int quant_coef[6][4][4] = {
  {{13107, 8066,13107, 8066},{ 8066, 5243, 8066, 5243},{13107, 8066,13107, 8066},{ 8066, 5243, 8066, 5243}},
  {{11916, 7490,11916, 7490},{ 7490, 4660, 7490, 4660},{11916, 7490,11916, 7490},{ 7490, 4660, 7490, 4660}},
  {{10082, 6554,10082, 6554},{ 6554, 4194, 6554, 4194},{10082, 6554,10082, 6554},{ 6554, 4194, 6554, 4194}},
  {{ 9362, 5825, 9362, 5825},{ 5825, 3647, 5825, 3647},{ 9362, 5825, 9362, 5825},{ 5825, 3647, 5825, 3647}},
  {{ 8192, 5243, 8192, 5243},{ 5243, 3355, 5243, 3355},{ 8192, 5243, 8192, 5243},{ 5243, 3355, 5243, 3355}},
  {{ 7282, 4559, 7282, 4559},{ 4559, 2893, 4559, 2893},{ 7282, 4559, 7282, 4559},{ 4559, 2893, 4559, 2893}}
};

static const int dequant_coef[6][4][4] = {
  {{10, 13, 10, 13},{ 13, 16, 13, 16},{10, 13, 10, 13},{ 13, 16, 13, 16}},
  {{11, 14, 11, 14},{ 14, 18, 14, 18},{11, 14, 11, 14},{ 14, 18, 14, 18}},
  {{13, 16, 13, 16},{ 16, 20, 16, 20},{13, 16, 13, 16},{ 16, 20, 16, 20}},
  {{14, 18, 14, 18},{ 18, 23, 18, 23},{14, 18, 14, 18},{ 18, 23, 18, 23}},
  {{16, 20, 16, 20},{ 20, 25, 20, 25},{16, 20, 16, 20},{ 20, 25, 20, 25}},
  {{18, 23, 18, 23},{ 23, 29, 23, 29},{18, 23, 18, 23},{ 23, 29, 23, 29}}
};
static const int A[4][4] = {
  { 16, 20, 16, 20},
  { 20, 25, 20, 25},
  { 16, 20, 16, 20},
  { 20, 25, 20, 25}
};


// Notation for comments regarding prediction and predictors.
// The pels of the 4x4 block are labelled a..p. The predictor pels above
// are labelled A..H, from the left I..P, and from above left X, as follows:
//
//  X A B C D E F G H
//  I a b c d
//  J e f g h
//  K i j k l
//  L m n o p
//

// Predictor array index definitions
#define P_X (PredPel[0])
#define P_A (PredPel[1])
#define P_B (PredPel[2])
#define P_C (PredPel[3])
#define P_D (PredPel[4])
#define P_E (PredPel[5])
#define P_F (PredPel[6])
#define P_G (PredPel[7])
#define P_H (PredPel[8])
#define P_I (PredPel[9])
#define P_J (PredPel[10])
#define P_K (PredPel[11])
#define P_L (PredPel[12])

/*!
 ************************************************************************
 * \brief
 *    Make intra 4x4 prediction according to all 9 prediction modes.
 *    The routine uses left and upper neighbouring points from
 *    previous coded blocks to do this (if available). Notice that
 *    inaccessible neighbouring points are signalled with a negative
 *    value in the predmode array .
 *
 *  \par Input:
 *     Starting point of current 4x4 block image posision
 *
 *  \par Output:
 *      none
 ************************************************************************
 */
 /*亮度的帧内预测
 *img_x 图像的x轴坐标
 *img_y 图像的y轴坐标
 *left_available 指示子宏块的左边是否可得
 *up_available 指示子宏块的上边是否可得
 *all_available 指示子宏块的左边，上边，和左上是否可得
*/
void intrapred_luma(int img_x,int img_y, int *left_available, int *up_available, int *all_available)
{
  int i,j;
  int s0;
  int PredPel[13];  // array of predictor pels
  byte **imgY = enc_picture->imgY;  // For MB level frame/field coding tools -- set default to imgY

  int ioff = (img_x & 15);	// 当前4*4块左上角像素在本宏块内的横坐标（以像素为单位）
  int joff = (img_y & 15);	// 当前4*4块左上角像素在本宏块内的纵坐标（以像素为单位）
  int mb_nr=img->current_mb_nr;

  PixelPos pix_a[4];//此数据结构用来获取指定像素点的位置信息
  PixelPos pix_b, pix_c, pix_d;

  int block_available_up;//4*4子宏块的上部相邻像素是否可得
  int block_available_left;//4*4子宏块的左部相邻像素是否可得
  int block_available_up_left;//4*4子宏块的左上部相邻像素是否可得
  int block_available_up_right;//4*4子宏块的右上部相邻像素是否可得

  for (i=0;i<4;i++)
  {
    getNeighbour(mb_nr, ioff -1 , joff +i , 1, &pix_a[i]);//获取左面一排像素的位置信息
  }
  
  
  getNeighbour(mb_nr, ioff    , joff -1 , 1, &pix_b);//上边像素位置信息
  getNeighbour(mb_nr, ioff +4 , joff -1 , 1, &pix_c);//右上边像素位置信息
  getNeighbour(mb_nr, ioff -1 , joff -1 , 1, &pix_d);//左上角像素位置信息

  /*
  *注意一个宏块中的4*4子宏块的预测顺序如下:
  *  1  2  5  6
  *  3  4  7  8
  *  9  10 13 14
  *  11 12 15 16
  *因为预测顺序，导致4，8，12，16这几个子宏块在做4*4帧内预测时dd右上角的像素不可用。
  */
  pix_c.available = pix_c.available && !(((ioff==4)||(ioff==12)) && ((joff==4)||(joff==12)));

  /*在userConstrainedIntraPred模式下，帧间编码的宏块的像素不能用于帧内预测。
  */
  if (input->UseConstrainedIntraPred)
  {
  //此处需要判断左边四个像素的每个点的情况，主要是适应在MBAFF情况下，左边的宏块的顶场跟底场按照不同编码方式(帧内跟帧间)进行了编码。
    for (i=0, block_available_left=1; i<4;i++)
      block_available_left  &= pix_a[i].available ? img->intra_block[pix_a[i].mb_addr]: 0;
    block_available_up       = pix_b.available ? img->intra_block [pix_b.mb_addr] : 0;
    block_available_up_right = pix_c.available ? img->intra_block [pix_c.mb_addr] : 0;
    block_available_up_left  = pix_d.available ? img->intra_block [pix_d.mb_addr] : 0;
  }
  else
  {
    block_available_left     = pix_a[0].available;
    block_available_up       = pix_b.available;
    block_available_up_right = pix_c.available;
    block_available_up_left  = pix_d.available;
  }
  
  *left_available = block_available_left;
  *up_available   = block_available_up;
  *all_available  = block_available_up && block_available_left && block_available_up_left;

  i = (img_x & 15);
  j = (img_y & 15);

  // form predictor pels
  /*获取4*4子宏块上边部分的参考像素值*/
  if (block_available_up)
  {
    P_A = imgY[pix_b.pos_y][pix_b.pos_x+0];
    P_B = imgY[pix_b.pos_y][pix_b.pos_x+1];
    P_C = imgY[pix_b.pos_y][pix_b.pos_x+2];
    P_D = imgY[pix_b.pos_y][pix_b.pos_x+3];

  }
  else
  {
  /*如果上边部分不可得，则将它们设置为128，具体请参照标准*/
    P_A = P_B = P_C = P_D = 128;
  }
  /*获取右上部分的参考像素值*/
  if (block_available_up_right)
  {
    P_E = imgY[pix_c.pos_y][pix_c.pos_x+0];
    P_F = imgY[pix_c.pos_y][pix_c.pos_x+1];
    P_G = imgY[pix_c.pos_y][pix_c.pos_x+2];
    P_H = imgY[pix_c.pos_y][pix_c.pos_x+3];
  }
  else
  {
  /*如果不可得，则用D点的像素值进行填充，具体请参照标准*/
    P_E = P_F = P_G = P_H = P_D;
  }

  if (block_available_left)
  {
  /*获取左边参考像素的值*/
    P_I = imgY[pix_a[0].pos_y][pix_a[0].pos_x];
    P_J = imgY[pix_a[1].pos_y][pix_a[1].pos_x];
    P_K = imgY[pix_a[2].pos_y][pix_a[2].pos_x];
    P_L = imgY[pix_a[3].pos_y][pix_a[3].pos_x];
  }
  else
  {
    /*如果不可得，则设置成128。请参考标准*/
    P_I = P_J = P_K = P_L = 128;
  }

  /*获取左上点的参考像素值*/
  if (block_available_up_left)
  {
    P_X = imgY[pix_d.pos_y][pix_d.pos_x];
  }
  else
  {
    P_X = 128;
  }

 /*数组mprr用来保存4*4帧内预测的结果*/
  for(i=0;i<9;i++)
    img->mprr[i][0][0]=-1;//置为-1，表示清空。
  ///////////////////////////////
  // make DC prediction
  ///////////////////////////////
  s0 = 0;
  if (block_available_up && block_available_left)
  {   
    // no edge
    s0 = (P_A + P_B + P_C + P_D + P_I + P_J + P_K + P_L + 4)/(2*BLOCK_SIZE);
  }
  else if (!block_available_up && block_available_left)
  {
    // upper edge
    s0 = (P_I + P_J + P_K + P_L + 2)/BLOCK_SIZE;             
  }
  else if (block_available_up && !block_available_left)
  {
    // left edge
    s0 = (P_A + P_B + P_C + P_D + 2)/BLOCK_SIZE;             
  }
  else //if (!block_available_up && !block_available_left)
  {
    // top left corner, nothing to predict from
    s0 = 128;                           
  }

  for (j=0; j < BLOCK_SIZE; j++)
  {
    for (i=0; i < BLOCK_SIZE; i++)
    {
      // store DC prediction
      img->mprr[DC_PRED][i][j] = s0;
    }
  }

  ///////////////////////////////
  // make horiz and vert prediction
  ///////////////////////////////

  for (i=0; i < BLOCK_SIZE; i++)
  {
    img->mprr[VERT_PRED][0][i] = 
    img->mprr[VERT_PRED][1][i] = 
    img->mprr[VERT_PRED][2][i] = 
    img->mprr[VERT_PRED][3][i] = (&P_A)[i];
    img->mprr[HOR_PRED][i][0]  = 
    img->mprr[HOR_PRED][i][1]  = 
    img->mprr[HOR_PRED][i][2]  = 
    img->mprr[HOR_PRED][i][3]  = (&P_I)[i];
  }

  if(!block_available_up)img->mprr[VERT_PRED][0][0]=-1;
  if(!block_available_left)img->mprr[HOR_PRED][0][0]=-1;

  if (block_available_up) 
  {
    // Mode DIAG_DOWN_LEFT_PRED
    img->mprr[DIAG_DOWN_LEFT_PRED][0][0] = (P_A + P_C + 2*(P_B) + 2) / 4;
    img->mprr[DIAG_DOWN_LEFT_PRED][0][1] = 
    img->mprr[DIAG_DOWN_LEFT_PRED][1][0] = (P_B + P_D + 2*(P_C) + 2) / 4;
    img->mprr[DIAG_DOWN_LEFT_PRED][0][2] =
    img->mprr[DIAG_DOWN_LEFT_PRED][1][1] =
    img->mprr[DIAG_DOWN_LEFT_PRED][2][0] = (P_C + P_E + 2*(P_D) + 2) / 4;
    img->mprr[DIAG_DOWN_LEFT_PRED][0][3] = 
    img->mprr[DIAG_DOWN_LEFT_PRED][1][2] = 
    img->mprr[DIAG_DOWN_LEFT_PRED][2][1] = 
    img->mprr[DIAG_DOWN_LEFT_PRED][3][0] = (P_D + P_F + 2*(P_E) + 2) / 4;
    img->mprr[DIAG_DOWN_LEFT_PRED][1][3] = 
    img->mprr[DIAG_DOWN_LEFT_PRED][2][2] = 
    img->mprr[DIAG_DOWN_LEFT_PRED][3][1] = (P_E + P_G + 2*(P_F) + 2) / 4;
    img->mprr[DIAG_DOWN_LEFT_PRED][2][3] = 
    img->mprr[DIAG_DOWN_LEFT_PRED][3][2] = (P_F + P_H + 2*(P_G) + 2) / 4;
    img->mprr[DIAG_DOWN_LEFT_PRED][3][3] = (P_G + 3*(P_H) + 2) / 4;

    // Mode VERT_LEFT_PRED
    img->mprr[VERT_LEFT_PRED][0][0] = (P_A + P_B + 1) / 2;
    img->mprr[VERT_LEFT_PRED][0][1] = 
    img->mprr[VERT_LEFT_PRED][2][0] = (P_B + P_C + 1) / 2;
    img->mprr[VERT_LEFT_PRED][0][2] = 
    img->mprr[VERT_LEFT_PRED][2][1] = (P_C + P_D + 1) / 2;
    img->mprr[VERT_LEFT_PRED][0][3] = 
    img->mprr[VERT_LEFT_PRED][2][2] = (P_D + P_E + 1) / 2;
    img->mprr[VERT_LEFT_PRED][2][3] = (P_E + P_F + 1) / 2;
    img->mprr[VERT_LEFT_PRED][1][0] = (P_A + 2*P_B + P_C + 2) / 4;
    img->mprr[VERT_LEFT_PRED][1][1] = 
    img->mprr[VERT_LEFT_PRED][3][0] = (P_B + 2*P_C + P_D + 2) / 4;
    img->mprr[VERT_LEFT_PRED][1][2] = 
    img->mprr[VERT_LEFT_PRED][3][1] = (P_C + 2*P_D + P_E + 2) / 4;
    img->mprr[VERT_LEFT_PRED][1][3] = 
    img->mprr[VERT_LEFT_PRED][3][2] = (P_D + 2*P_E + P_F + 2) / 4;
    img->mprr[VERT_LEFT_PRED][3][3] = (P_E + 2*P_F + P_G + 2) / 4;

  }

  /*  Prediction according to 'diagonal' modes */
  if (block_available_left) 
  {
    // Mode HOR_UP_PRED
    img->mprr[HOR_UP_PRED][0][0] = (P_I + P_J + 1) / 2;
    img->mprr[HOR_UP_PRED][0][1] = (P_I + 2*P_J + P_K + 2) / 4;
    img->mprr[HOR_UP_PRED][0][2] = 
    img->mprr[HOR_UP_PRED][1][0] = (P_J + P_K + 1) / 2;
    img->mprr[HOR_UP_PRED][0][3] = 
    img->mprr[HOR_UP_PRED][1][1] = (P_J + 2*P_K + P_L + 2) / 4;
    img->mprr[HOR_UP_PRED][1][2] = 
    img->mprr[HOR_UP_PRED][2][0] = (P_K + P_L + 1) / 2;
    img->mprr[HOR_UP_PRED][1][3] = 
    img->mprr[HOR_UP_PRED][2][1] = (P_K + 2*P_L + P_L + 2) / 4;
    img->mprr[HOR_UP_PRED][3][0] = 
    img->mprr[HOR_UP_PRED][2][2] = 
    img->mprr[HOR_UP_PRED][2][3] = 
    img->mprr[HOR_UP_PRED][3][1] = 
    img->mprr[HOR_UP_PRED][3][2] = 
    img->mprr[HOR_UP_PRED][3][3] = P_L;
  }

  /*  Prediction according to 'diagonal' modes */
  if (block_available_up && block_available_left && block_available_up_left) 
  {
    // Mode DIAG_DOWN_RIGHT_PRED
    img->mprr[DIAG_DOWN_RIGHT_PRED][3][0] = (P_L + 2*P_K + P_J + 2) / 4; 
    img->mprr[DIAG_DOWN_RIGHT_PRED][2][0] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][3][1] = (P_K + 2*P_J + P_I + 2) / 4; 
    img->mprr[DIAG_DOWN_RIGHT_PRED][1][0] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][2][1] = 
    img->mprr[DIAG_DOWN_RIGHT_PRED][3][2] = (P_J + 2*P_I + P_X + 2) / 4; 
    img->mprr[DIAG_DOWN_RIGHT_PRED][0][0] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][1][1] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][2][2] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][3][3] = (P_I + 2*P_X + P_A + 2) / 4; 
    img->mprr[DIAG_DOWN_RIGHT_PRED][0][1] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][1][2] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][2][3] = (P_X + 2*P_A + P_B + 2) / 4;
    img->mprr[DIAG_DOWN_RIGHT_PRED][0][2] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][1][3] = (P_A + 2*P_B + P_C + 2) / 4;
    img->mprr[DIAG_DOWN_RIGHT_PRED][0][3] = (P_B + 2*P_C + P_D + 2) / 4;

     // Mode VERT_RIGHT_PRED
    img->mprr[VERT_RIGHT_PRED][0][0] = 
    img->mprr[VERT_RIGHT_PRED][2][1] = (P_X + P_A + 1) / 2;
    img->mprr[VERT_RIGHT_PRED][0][1] = 
    img->mprr[VERT_RIGHT_PRED][2][2] = (P_A + P_B + 1) / 2;
    img->mprr[VERT_RIGHT_PRED][0][2] = 
    img->mprr[VERT_RIGHT_PRED][2][3] = (P_B + P_C + 1) / 2;
    img->mprr[VERT_RIGHT_PRED][0][3] = (P_C + P_D + 1) / 2;
    img->mprr[VERT_RIGHT_PRED][1][0] = 
    img->mprr[VERT_RIGHT_PRED][3][1] = (P_I + 2*P_X + P_A + 2) / 4;
    img->mprr[VERT_RIGHT_PRED][1][1] = 
    img->mprr[VERT_RIGHT_PRED][3][2] = (P_X + 2*P_A + P_B + 2) / 4;
    img->mprr[VERT_RIGHT_PRED][1][2] = 
    img->mprr[VERT_RIGHT_PRED][3][3] = (P_A + 2*P_B + P_C + 2) / 4;
    img->mprr[VERT_RIGHT_PRED][1][3] = (P_B + 2*P_C + P_D + 2) / 4;
    img->mprr[VERT_RIGHT_PRED][2][0] = (P_X + 2*P_I + P_J + 2) / 4;
    img->mprr[VERT_RIGHT_PRED][3][0] = (P_I + 2*P_J + P_K + 2) / 4;

    // Mode HOR_DOWN_PRED
    img->mprr[HOR_DOWN_PRED][0][0] = 
    img->mprr[HOR_DOWN_PRED][1][2] = (P_X + P_I + 1) / 2;
    img->mprr[HOR_DOWN_PRED][0][1] = 
    img->mprr[HOR_DOWN_PRED][1][3] = (P_I + 2*P_X + P_A + 2) / 4;
    img->mprr[HOR_DOWN_PRED][0][2] = (P_X + 2*P_A + P_B + 2) / 4;
    img->mprr[HOR_DOWN_PRED][0][3] = (P_A + 2*P_B + P_C + 2) / 4;
    img->mprr[HOR_DOWN_PRED][1][0] = 
    img->mprr[HOR_DOWN_PRED][2][2] = (P_I + P_J + 1) / 2;
    img->mprr[HOR_DOWN_PRED][1][1] = 
    img->mprr[HOR_DOWN_PRED][2][3] = (P_X + 2*P_I + P_J + 2) / 4;
    img->mprr[HOR_DOWN_PRED][2][0] = 
    img->mprr[HOR_DOWN_PRED][3][2] = (P_J + P_K + 1) / 2;
    img->mprr[HOR_DOWN_PRED][2][1] = 
    img->mprr[HOR_DOWN_PRED][3][3] = (P_I + 2*P_J + P_K + 2) / 4;
    img->mprr[HOR_DOWN_PRED][3][0] = (P_K + P_L + 1) / 2;
    img->mprr[HOR_DOWN_PRED][3][1] = (P_J + 2*P_K + P_L + 2) / 4;
  }
}

/*!
 ************************************************************************
 * \brief
 *    16x16 based luma prediction
 *
 * \par Input:
 *    Image parameters
 *
 * \par Output:
 *    none
 ************************************************************************
 */
 /*亮度16*16帧内预测*/
void intrapred_luma_16x16()
{
  int s0=0,s1,s2;
  int s[16][2];
  int i,j;

  int ih,iv;
  int ib,ic,iaa;

  byte   **imgY_pred = enc_picture->imgY;  // For Mb level field/frame coding tools -- default to frame pred
  int          mb_nr = img->current_mb_nr;

  PixelPos up;          //!< pixel position p(0,-1)
  PixelPos left[17];    //!< pixel positions p(-1, -1..15)

  int up_avail, left_avail, left_up_avail;

  for (i=0;i<17;i++)
  {
    getNeighbour(mb_nr, -1 ,  i-1 , 1, &left[i]);
  }
  
  getNeighbour(mb_nr, 0     ,  -1 , 1, &up);

  if (!(input->UseConstrainedIntraPred))
  {
    up_avail   = up.available;
    left_avail = left[1].available;
    left_up_avail = left[0].available;
  }
  else
  {
    up_avail      = up.available ? img->intra_block[up.mb_addr] : 0;
    //此处需要判断左边四个像素的每个点的情况，主要是适应在MBAFF情况下，左边的宏块的顶场跟底场按照不同编码方式(帧内跟帧间)进行了编码。
    for (i=1, left_avail=1; i<17;i++)
      left_avail  &= left[i].available ? img->intra_block[left[i].mb_addr]: 0;
    left_up_avail = left[0].available ? img->intra_block[left[0].mb_addr]: 0;
  }

  s1=s2=0;
  // make DC prediction	// 参见标准：8.3.3.3 小节
  for (i=0; i < MB_BLOCK_SIZE; i++)
  {
    if (up_avail)
      s1 += imgY_pred[up.pos_y][up.pos_x+i];    // sum hor pix
    if (left_avail)
      s2 += imgY_pred[left[i+1].pos_y][left[i+1].pos_x];    // sum vert pix
  }
  if (up_avail && left_avail)
    s0=(s1+s2+16)/(2*MB_BLOCK_SIZE);             // no edge
  if (!up_avail && left_avail)
    s0=(s2+8)/MB_BLOCK_SIZE;                     // upper edge
  if (up_avail && !left_avail)
    s0=(s1+8)/MB_BLOCK_SIZE;                     // left edge
  if (!up_avail && !left_avail)
    s0=128;                                      // top left corner, nothing to predict from

  for (i=0; i < MB_BLOCK_SIZE; i++)
  {
    // vertical prediction	// 参见标准：8.3.3.1 小节
    if (up_avail)
      s[i][0]=imgY_pred[up.pos_y][up.pos_x+i];
    // horizontal prediction	// 参见标准：8.3.3.2 小节
    if (left_avail)
      s[i][1]=imgY_pred[left[i+1].pos_y][left[i+1].pos_x];
  }

  for (j=0; j < MB_BLOCK_SIZE; j++)
  {
    for (i=0; i < MB_BLOCK_SIZE; i++)
    {
    /*mprr数组用来保存16*16帧内预测的结果*/
      img->mprr_2[VERT_PRED_16][j][i]=s[i][0]; // store vertical prediction
      img->mprr_2[HOR_PRED_16 ][j][i]=s[j][1]; // store horizontal prediction
      img->mprr_2[DC_PRED_16  ][j][i]=s0;      // store DC prediction
    }
  }
  /*在如下情况下，只有水平，垂直，DC三个方向的预测可用*/
  if (!up_avail || !left_avail || !left_up_avail) // edge
    return;

  // 16 bit integer plan pred	// 参见标准：8.3.3.4 小节

  ih=0;
  iv=0;
  for (i=1;i<9;i++)
  {
    if (i<8)
      ih += i*(imgY_pred[up.pos_y][up.pos_x+7+i] - imgY_pred[up.pos_y][up.pos_x+7-i]);
    else
      ih += i*(imgY_pred[up.pos_y][up.pos_x+7+i] - imgY_pred[left[0].pos_y][left[0].pos_x]);
    
    iv += i*(imgY_pred[left[8+i].pos_y][left[8+i].pos_x] - imgY_pred[left[8-i].pos_y][left[8-i].pos_x]);
  }
  ib=(5*ih+32)>>6;
  ic=(5*iv+32)>>6;
  
  iaa=16*(imgY_pred[up.pos_y][up.pos_x+15]+imgY_pred[left[16].pos_y][left[16].pos_x]);

  for (j=0;j< MB_BLOCK_SIZE;j++)
  {
    for (i=0;i< MB_BLOCK_SIZE;i++)
    {
      img->mprr_2[PLANE_16][j][i]=max(0,min(255,(iaa+(i-7)*ib +(j-7)*ic + 16)/32));// store plane prediction
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    For new intra pred routines
 *
 * \par Input:
 *    Image par, 16x16 based intra mode
 *
 * \par Output:
 *    none 
 ************************************************************************
 */
 /*
 *16*16dct变换  mprr_2存放的【仅是】帧内 16*16四种预测模式的残差值做变换
 
 *注意这里没有包括rdo
 *new_intra_mode 使用的帧内预测方式
*/
int dct_luma_16x16(int new_intra_mode)
{
  int qp_const;
  int i,j;
  int ii,jj;
  int i1,j1;
  int M1[16][16];
  int M4[4][4];
  int M5[4],M6[4];
  int M0[4][4][4][4];
  int run,scan_pos,coeff_ctr,level;
  int qp_per,qp_rem,q_bits;
  int ac_coef = 0;

  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  int   b8, b4;
  int*  DCLevel = img->cofDC[0][0];
  int*  DCRun   = img->cofDC[0][1];
  int*  ACLevel;
  int*  ACRun;

  /*计算量化参数
  
  */
  qp_per    = (currMB->qp-MIN_QP)/6;
  qp_rem    = (currMB->qp-MIN_QP)%6;

  q_bits    = Q_BITS+qp_per;
  
  qp_const  = (1<<q_bits)/3;//  f  帧内预测

  for (j=0;j<16;j++)
  {
    for (i=0;i<16;i++)
    {
      /*计算残差*/
      M1[i][j]=imgY_org[img->opix_y+j][img->opix_x+i]-img->mprr_2[new_intra_mode][j][i];
      /*间隔抽取样点*/
      M0[i%4][i/4][j%4][j/4]=M1[i][j];//划分成4*4个子宏块 进行4*4大小的DCT变换
    }
  }

 /*对16个4*4的子宏块一起进行dct变换*/
  for (jj=0;jj<4;jj++)//子宏块索引，注意前面的间隔采样
  {
    for (ii=0;ii<4;ii++)//子宏块索引，注意前面的间隔采样
    {
      //这里才是二维DCT变换。先水平变换。
      for (j=0;j<4;j++)
      {
        //先是对每一行进行的一维变换
        //这里采用的是蝶形算法
        for (i=0;i<2;i++)
        {
          i1=3-i;
          M5[i]=  M0[i][ii][j][jj]+M0[i1][ii][j][jj];
          M5[i1]= M0[i][ii][j][jj]-M0[i1][ii][j][jj];
        }
        M0[0][ii][j][jj]=M5[0]+M5[1];
        M0[2][ii][j][jj]=M5[0]-M5[1];
        M0[1][ii][j][jj]=M5[3]*2+M5[2];
        M0[3][ii][j][jj]=M5[3]-M5[2]*2;
      }
      // vertical 垂直变换
      for (i=0;i<4;i++)
      {
        for (j=0;j<2;j++)
        {
          j1=3-j;
          M5[j] = M0[i][ii][j][jj]+M0[i][ii][j1][jj];
          M5[j1]= M0[i][ii][j][jj]-M0[i][ii][j1][jj];
        }
        M0[i][ii][0][jj]=M5[0]+M5[1];
        M0[i][ii][2][jj]=M5[0]-M5[1];
        M0[i][ii][1][jj]=M5[3]*2+M5[2];
        M0[i][ii][3][jj]=M5[3] -M5[2]*2;
      }
    }
  }

  // pick out DC coeff

  //抽取出每个4*4子宏块的DC分量
  for (j=0;j<4;j++)
    for (i=0;i<4;i++)
      M4[i][j]= M0[0][i][0][j];

  //对DC分量进行二维哈达马变换
  //先是行变换
  for (j=0;j<4;j++)
  {
    // 对一行的数据进行一维的哈达马变换，
    // 使用蝶形算法
    for (i=0;i<2;i++)
    {
      i1=3-i;
      M5[i]= M4[i][j]+M4[i1][j];
      M5[i1]=M4[i][j]-M4[i1][j];
    }
    M4[0][j]=M5[0]+M5[1];
    M4[2][j]=M5[0]-M5[1];
    M4[1][j]=M5[3]+M5[2];
    M4[3][j]=M5[3]-M5[2];
  }

  // vertical
  // 再是列变换
  for (i=0;i<4;i++)
  {
    for (j=0;j<2;j++)
    {
      j1=3-j;
      M5[j]= M4[i][j]+M4[i][j1];
      M5[j1]=M4[i][j]-M4[i][j1];
    }
    M4[i][0]=(M5[0]+M5[1])>>1;//这里右移一位，相当于公式前面的系数1/2
    M4[i][2]=(M5[0]-M5[1])>>1;
    M4[i][1]=(M5[3]+M5[2])>>1;
    M4[i][3]=(M5[3]-M5[2])>>1;
  }

  // quant
  run=-1;
  scan_pos=0;
 //对16个直流DC系数变化后的系数进行从排列，量化，游程编码
  for (coeff_ctr=0;coeff_ctr<16;coeff_ctr++)
  {
    if (img->field_picture || ( mb_adaptive && img->field_mode )) 
    {  // Alternate scan for field coding  场扫描
        i=FIELD_SCAN[coeff_ctr][0];
        j=FIELD_SCAN[coeff_ctr][1];
    }
    else //帧扫描
    {
        i=SNGL_SCAN[coeff_ctr][0];
        j=SNGL_SCAN[coeff_ctr][1];
    }
	
	run++;
    //这里进行游程编码，将量化后的残差系数编码成(level，run)数值对。
    //哈达玛量化公式请参照标准。
    level= (abs(M4[i][j]) * quant_coef[qp_rem][0][0] + 2*qp_const)>>(q_bits+1);

    if (level != 0)//游程编码
    {
      DCLevel[scan_pos] = sign(level,M4[i][j]);//量化
      DCRun  [scan_pos] = run;
      ++scan_pos;
      run=-1;
    }
    M4[i][j]=sign(level,M4[i][j]);
  }
  DCLevel[scan_pos]=0;

  //对直流分量进行2维反哈达马变换
  //先水平变化
  for (j=0;j<4;j++)
  {
    for (i=0;i<4;i++)
      M5[i]=M4[i][j];

    M6[0]=M5[0]+M5[2];
    M6[1]=M5[0]-M5[2];
    M6[2]=M5[1]-M5[3];
    M6[3]=M5[1]+M5[3];

    for (i=0;i<2;i++)
    {
      i1=3-i;
      M4[i][j]= M6[i]+M6[i1];
      M4[i1][j]=M6[i]-M6[i1];
    }
  }
  //再竖直变化
  for (i=0;i<4;i++)
  {
    for (j=0;j<4;j++)
      M5[j]=M4[i][j];

    M6[0]=M5[0]+M5[2];
    M6[1]=M5[0]-M5[2];
    M6[2]=M5[1]-M5[3];
    M6[3]=M5[1]+M5[3];

    for (j=0;j<2;j++)
    {
      j1=3-j;
      M0[0][i][0][j] = (((M6[j]+M6[j1])*dequant_coef[qp_rem][0][0]<<qp_per)+2)>>2;//这里包含了反量化过程。
      M0[0][i][0][j1]= (((M6[j]-M6[j1])*dequant_coef[qp_rem][0][0]<<qp_per)+2)>>2;
    }
  }

  // AC invers trans/quant for MB
  //到这里才对每个4*4宏块的AC分量(15个)进行量化，从排列，游程编码，反量化，反变化。
  for (jj=0;jj<4;jj++)//子宏块索引
  {
    for (ii=0;ii<4;ii++)//子宏块索引
    {
      run      = -1;
      scan_pos =  0;
      b8       = 2*(jj/2) + (ii/2);//计算8*8子宏块编号。
      b4       = 2*(jj%2) + (ii%2);//计算4*4子宏块编号，在上面8*8子宏块内部的编号。
      ACLevel  = img->cofAC [b8][b4][0];//注意ACLevel，ACRun都是指针。
      ACRun    = img->cofAC [b8][b4][1];

      for (coeff_ctr=1;coeff_ctr<16;coeff_ctr++) // set in AC coeff
      {
	//zigzag扫描，注意场扫描跟帧扫描方式的区别。
        if (img->field_picture || ( mb_adaptive && img->field_mode )) 
        {  // Alternate scan for field coding
          i=FIELD_SCAN[coeff_ctr][0];
          j=FIELD_SCAN[coeff_ctr][1];
        }
        else 
        {
          i=SNGL_SCAN[coeff_ctr][0];
          j=SNGL_SCAN[coeff_ctr][1];
        }
	 //游程编码
        run++;

	//量化  按照先对第一个8*8子块内部的4个4*4子块按照顺序进行量化，再下一个子块
        level= ( abs( M0[i][ii][j][jj]) * quant_coef[qp_rem][i][j] + qp_const) >> q_bits;

        if (level != 0)
        {
          ac_coef = 15;
          ACLevel[scan_pos] = sign(level,M0[i][ii][j][jj]);
          ACRun  [scan_pos] = run;//游程
          ++scan_pos;
          run=-1;
        }
	 //反量化
        M0[i][ii][j][jj]=sign(level*dequant_coef[qp_rem][i][j]<<qp_per,M0[i][ii][j][jj]);
      }
      ACLevel[scan_pos] = 0;


      // IDCT horizontal
      //反DCT变换
	//先水平
      for (j=0;j<4;j++)
      {
        for (i=0;i<4;i++)
        {
          M5[i]=M0[i][ii][j][jj];
        }

        M6[0]= M5[0]+M5[2];
        M6[1]= M5[0]-M5[2];
        M6[2]=(M5[1]>>1) -M5[3];
        M6[3]= M5[1]+(M5[3]>>1);

        for (i=0;i<2;i++)
        {
          i1=3-i;
          M0[i][ii][j][jj] =M6[i]+M6[i1];
          M0[i1][ii][j][jj]=M6[i]-M6[i1];
        }
      }

      // 再竖直
      for (i=0;i<4;i++)
      {
        for (j=0;j<4;j++)
          M5[j]=M0[i][ii][j][jj];

        M6[0]= M5[0]+M5[2];
        M6[1]= M5[0]-M5[2];
        M6[2]=(M5[1]>>1) -M5[3];
        M6[3]= M5[1]+(M5[3]>>1);

        for (j=0;j<2;j++)
        {
          j1=3-j;
          M0[i][ii][ j][jj]=M6[j]+M6[j1];
          M0[i][ii][j1][jj]=M6[j]-M6[j1];

        }
      }

    }
  }

 //重新排列好宏块的像素
  for (j=0;j<16; j++)
  {
    for (i=0;i<16;i++)
    {
      M1[i][j]=M0[i%4][i/4][j%4][j/4];
    }
  }

//拷贝到编码后的图像中(从新按照byte型组织图像)，需要注意的是使用在[0, 255]之间的限幅操作
  for (j=0;j<16;j++)
    for (i=0;i<16;i++)
      enc_picture->imgY[img->pix_y+j][img->pix_x+i]=(byte)min(255,max(0,(M1[i][j]+(img->mprr_2[new_intra_mode][j][i]<<DQ_BITS)+DQ_ROUND)>>DQ_BITS));

    return ac_coef;
}


/*!
 ************************************************************************
 * \brief
 *    The routine performs transform,quantization,inverse transform, adds the diff.
 *    to the prediction and writes the result to the decoded luma frame. Includes the
 *    RD constrained quantization also.
 *
 * \par Input:
 *    block_x,block_y: Block position inside a macro block (0,4,8,12).
 *
 * \par Output_
 *    nonzero: 0 if no levels are nonzero.  1 if there are nonzero levels.             \n
 *    coeff_cost: Counter for nonzero coefficients, used to discard expencive levels.
 根据coeff_cost的大小，决定这一宏块的系数是否舍去?
 ************************************************************************
 用来对子宏块(m7存放的预测残差值)【临时数据】进行DCT变换、量化、反量化、反变换、重构子宏块；
 加入了判决规则，决定选择哪种子宏块编码方式
 帧内、帧间的DCT变换
 */
 /*亮度的dct变换，包括了rdo*/
int dct_luma(int block_x,int block_y,int *coeff_cost, int old_intra_mode)
{
  int sign(int a,int b);

  int i,j,i1,j1,ilev,m5[4],m6[4],coeff_ctr;
  int qp_const,level,scan_pos,run;
  int nonzero;
  int qp_per,qp_rem,q_bits;

  int   pos_x   = block_x/BLOCK_SIZE;//计算子宏块在宏块中的相对坐标
  int   pos_y   = block_y/BLOCK_SIZE;
  int   b8      = 2*(pos_y/2) + (pos_x/2);
  int   b4      = 2*(pos_y%2) + (pos_x%2);
  int*  ACLevel = img->cofAC[b8][b4][0];
  int*  ACRun   = img->cofAC[b8][b4][1];

  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  //计算量化参数
  qp_per    = (currMB->qp-MIN_QP)/6;
  qp_rem    = (currMB->qp-MIN_QP)%6;
  q_bits    = Q_BITS+qp_per;

 //对于intra和inter预测方式，量化的死区修正值是不同的，请参考标准。
  if (img->type == I_SLICE)
    qp_const=(1<<q_bits)/3;    // intra 帧内	
  else
    qp_const=(1<<q_bits)/6;    // inter 帧间
  
  //二维DCT变换 4*4的
  //  Horizontal transform 水平变换
  for (j=0; j < BLOCK_SIZE; j++)
  {
    for (i=0; i < 2; i++)
    {
      i1=3-i;
      m5[i]=img->m7[i][j]+img->m7[i1][j];//请注意m7存放的是预测的残差值，用来做rdo
      m5[i1]=img->m7[i][j]-img->m7[i1][j];
    }
    img->m7[0][j]=(m5[0]+m5[1]);
    img->m7[2][j]=(m5[0]-m5[1]);
    img->m7[1][j]=m5[3]*2+m5[2];
    img->m7[3][j]=m5[3]-m5[2]*2;
  }

  //  竖直变换
  for (i=0; i < BLOCK_SIZE; i++)
  {
    for (j=0; j < 2; j++)
    {
      j1=3-j;
      m5[j]=img->m7[i][j]+img->m7[i][j1];
      m5[j1]=img->m7[i][j]-img->m7[i][j1];
    }
    img->m7[i][0]=(m5[0]+m5[1]);
    img->m7[i][2]=(m5[0]-m5[1]);
    img->m7[i][1]=m5[3]*2+m5[2];
    img->m7[i][3]=m5[3]-m5[2]*2;
  }

  // Quant 量化

  nonzero=FALSE;//指示量化后，level值是否全是零。

  run=-1;//游程
  scan_pos=0;
  
  for (coeff_ctr=0;coeff_ctr < 16;coeff_ctr++)
  {

    if (img->field_picture || ( img->MbaffFrameFlag && currMB->mb_field )) 
    {  // Alternate scan for field coding
        i=FIELD_SCAN[coeff_ctr][0];
        j=FIELD_SCAN[coeff_ctr][1];
    }
    else 
    {
        i=SNGL_SCAN[coeff_ctr][0];
        j=SNGL_SCAN[coeff_ctr][1];
    }
    
    run++;
    ilev=0;
    
    level = (abs (img->m7[i][j]) * quant_coef[qp_rem][i][j] + qp_const) >> q_bits;
    
    if (level != 0)
    {
      nonzero=TRUE;
      if (level > 1)
        *coeff_cost += MAX_VALUE;                // set high cost, shall not be discarded
      else
        *coeff_cost += COEFF_COST[run];//注意这里根据level，run来计算coeff_cost
        //频率越低，系数越重要
      ACLevel[scan_pos] = sign(level,img->m7[i][j]);
      ACRun  [scan_pos] = run;
      ++scan_pos;
      run=-1;                     // reset zero level counter
      ilev=level*dequant_coef[qp_rem][i][j]<<qp_per;//反量化
    }
    img->m7[i][j]=sign(ilev,img->m7[i][j]);//反量化
  }
  ACLevel[scan_pos] = 0;
  
  
  //     IDCT.
  //     horizontal

  for (j=0; j < BLOCK_SIZE; j++)
  {
    for (i=0; i < BLOCK_SIZE; i++)
    {
      m5[i]=img->m7[i][j];
    }
    m6[0]=(m5[0]+m5[2]);
    m6[1]=(m5[0]-m5[2]);
    m6[2]=(m5[1]>>1)-m5[3];
    m6[3]=m5[1]+(m5[3]>>1);

    for (i=0; i < 2; i++)
    {
      i1=3-i;
      img->m7[i][j]=m6[i]+m6[i1];
      img->m7[i1][j]=m6[i]-m6[i1];
    }
  }

  //  vertical

  for (i=0; i < BLOCK_SIZE; i++)
  {
    for (j=0; j < BLOCK_SIZE; j++)
    {
      m5[j]=img->m7[i][j];
    }
    m6[0]=(m5[0]+m5[2]);
    m6[1]=(m5[0]-m5[2]);
    m6[2]=(m5[1]>>1)-m5[3];
    m6[3]=m5[1]+(m5[3]>>1);

    for (j=0; j < 2; j++)
    {
      j1=3-j;
      img->m7[i][j] =min(255,max(0,(m6[j]+m6[j1]+(img->mpr[i+block_x][j+block_y] <<DQ_BITS)+DQ_ROUND)>>DQ_BITS));
      img->m7[i][j1]=min(255,max(0,(m6[j]-m6[j1]+(img->mpr[i+block_x][j1+block_y]<<DQ_BITS)+DQ_ROUND)>>DQ_BITS));
    }
  }

  //  Decoded block moved to frame memory

  for (j=0; j < BLOCK_SIZE; j++)
    for (i=0; i < BLOCK_SIZE; i++)
      enc_picture->imgY[img->pix_y+block_y+j][img->pix_x+block_x+i]=img->m7[i][j];


  return nonzero;
}



/*!
 ************************************************************************
 * \brief
 *    Transform,quantization,inverse transform for chroma.
 *    The main reason why this is done in a separate routine is the
 *    additional 2x2 transform of DC-coeffs. This routine is called
 *    ones for each of the chroma components.
 *
 * \par Input:
 *    uv    : Make difference between the U and V chroma component  \n
 *    cr_cbp: chroma coded block pattern 色度编码模式
 *
 * \par Output:
 *    cr_cbp: Updated chroma coded block pattern.
 ************************************************************************
 */
int dct_chroma(int uv,int cr_cbp)
{//uv 只能取0或1 
  int i,j,i1,j2,ilev,n2,n1,j1,mb_y,coeff_ctr,qp_const,level ,scan_pos,run;
  int m1[BLOCK_SIZE],m5[BLOCK_SIZE],m6[BLOCK_SIZE];
  int coeff_cost;
  int cr_cbp_tmp;
  int nn0,nn1;
  int DCcoded=0 ;
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  int qp_per,qp_rem,q_bits;

  int   b4;
  int*  DCLevel = img->cofDC[uv+1][0];
  int*  DCRun   = img->cofDC[uv+1][1];
  int*  ACLevel;
  int*  ACRun;

  int qpChroma=Clip3(0, 51, currMB->qp + active_pps->chroma_qp_index_offset);
  /*chroma_qp_index_offset色度分量的量化参数是根据亮度分量的量化参数计算出来的，
  本句法元素用以指明计算时用到的参数。*/
  //计算色度分量的量化参数，需要注意的是跟亮度量化参数间的转换关系，请参考标准。
  qp_per    = QP_SCALE_CR[qpChroma-MIN_QP]/6;
  qp_rem    = QP_SCALE_CR[qpChroma-MIN_QP]%6;
  q_bits    = Q_BITS+qp_per;

  if (img->type == I_SLICE)
    qp_const=(1<<q_bits)/3;    // intra 帧内
  else
    qp_const=(1<<q_bits)/6;    // inter 帧间

  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)  //n2最大值是BLOCK_SIZE=4
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {
      //4*4的色度分量DCT变化
      //  Horizontal transform.
      for (j=0; j < BLOCK_SIZE; j++)
      {
        mb_y=n2+j;
        for (i=0; i < 2; i++)
        {
          i1=3-i;
          m5[i]=img->m7[i+n1][mb_y]+img->m7[i1+n1][mb_y];
          m5[i1]=img->m7[i+n1][mb_y]-img->m7[i1+n1][mb_y];
        }
        img->m7[n1][mb_y]  =(m5[0]+m5[1]);
        img->m7[n1+2][mb_y]=(m5[0]-m5[1]);
        img->m7[n1+1][mb_y]=m5[3]*2+m5[2];
        img->m7[n1+3][mb_y]=m5[3]-m5[2]*2;
      }

      //  Vertical transform.

      for (i=0; i < BLOCK_SIZE; i++)
      {
        j1=n1+i;
        for (j=0; j < 2; j++)
        {
          j2=3-j;
          m5[j]=img->m7[j1][n2+j]+img->m7[j1][n2+j2];
          m5[j2]=img->m7[j1][n2+j]-img->m7[j1][n2+j2];
        }
        img->m7[j1][n2+0]=(m5[0]+m5[1]);
        img->m7[j1][n2+2]=(m5[0]-m5[1]);
        img->m7[j1][n2+1]=m5[3]*2+m5[2];
        img->m7[j1][n2+3]=m5[3]-m5[2]*2;
      }
    }
  }

  //     2X2 transform of DC coeffs.
  //   对直流分量进行2*2哈达马变换。需要注意的是这里的分量存储在m1中时已经进行了从排列
  m1[0]=(img->m7[0][0]+img->m7[4][0]+img->m7[0][4]+img->m7[4][4]);
  m1[1]=(img->m7[0][0]-img->m7[4][0]+img->m7[0][4]-img->m7[4][4]);
  m1[2]=(img->m7[0][0]+img->m7[4][0]-img->m7[0][4]-img->m7[4][4]);
  m1[3]=(img->m7[0][0]-img->m7[4][0]-img->m7[0][4]+img->m7[4][4]);

  //     Quant of chroma 2X2 coeffs.
  //对直流分量进行的量化，游程编码。
  run=-1;
  scan_pos=0;

  for (coeff_ctr=0; coeff_ctr < 4; coeff_ctr++)
  {
    run++;
    ilev=0;

    level =(abs(m1[coeff_ctr]) * quant_coef[qp_rem][0][0] + 2*qp_const) >> (q_bits+1);

    if (level  != 0)
    {
     //这里跟色度分量(level,run)表的查表方式有关，请参考标准附录。
      currMB->cbp_blk |= 0xf0000 << (uv << 2) ;    //什么功能? if one of the 2x2-DC levels is != 0 set the
	 // coded-bit all 4 4x4 blocks (bit 16-19 or 20-23)   
//如果有一个2x2-DC幅值不为0，则设置 所有4个4x4宏块的编码比特

	  cr_cbp=max(1,cr_cbp);    //什么功能?                
      DCcoded = 1 ;
      DCLevel[scan_pos] = sign(level ,m1[coeff_ctr]);
      DCRun  [scan_pos] = run;
      scan_pos++;
      run=-1;
      ilev=level*dequant_coef[qp_rem][0][0]<<qp_per;
    }
    m1[coeff_ctr]=sign(ilev,m1[coeff_ctr]);
  }
  DCLevel[scan_pos] = 0;

  //  Invers transform of 2x2 DC levels
  // 反哈达马变换
  img->m7[0][0]=(m1[0]+m1[1]+m1[2]+m1[3])>>1;
  img->m7[4][0]=(m1[0]-m1[1]+m1[2]-m1[3])>>1;
  img->m7[0][4]=(m1[0]+m1[1]-m1[2]-m1[3])>>1;
  img->m7[4][4]=(m1[0]-m1[1]-m1[2]+m1[3])>>1;

  //     Quant of chroma AC-coeffs.
  coeff_cost=0;
  cr_cbp_tmp=0;
  
  //这里对各个子宏块的AC分量进行处理
  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {
      //对一个子宏块进行处理
      //zigzag扫描
      b4      = 2*(n2/4) + (n1/4);
      ACLevel = img->cofAC[uv+4][b4][0];//第5、第6个8*8宏块 色度宏块
      ACRun   = img->cofAC[uv+4][b4][1];
      run=-1;
      scan_pos=0;

      for (coeff_ctr=1; coeff_ctr < 16; coeff_ctr++)// start change rd_quant
      {

        if (img->field_picture || ( img->MbaffFrameFlag && currMB->mb_field )) 
        {  // Alternate scan for field coding
          i=FIELD_SCAN[coeff_ctr][0];
          j=FIELD_SCAN[coeff_ctr][1];
        }
        else 
        {
          i=SNGL_SCAN[coeff_ctr][0];
          j=SNGL_SCAN[coeff_ctr][1];
        }
        ++run;
        ilev=0;
        //量化
        level=(abs(img->m7[n1+i][n2+j])*quant_coef[qp_rem][i][j]+qp_const)>>q_bits;//；量化区间的计算

        if (level  != 0)
        {
        //用来计算rdo
          currMB->cbp_blk |= 1 << (16 + (uv << 2) + ((n2 >> 1) + (n1 >> 2))) ;
          if (level > 1)
            coeff_cost += MAX_VALUE;                // set high cost, shall not be discarded
          else
            coeff_cost += COEFF_COST[run];

          cr_cbp_tmp=2;
          ACLevel[scan_pos] = sign(level,img->m7[n1+i][n2+j]);//量化
          ACRun  [scan_pos] = run;
          ++scan_pos;
          run=-1;
          ilev=level*dequant_coef[qp_rem][i][j]<<qp_per;
        }
        img->m7[n1+i][n2+j]=sign(ilev,img->m7[n1+i][n2+j]); // for use in IDCT
      }
      ACLevel[scan_pos] = 0;
    }
  }

  // * reset chroma coeffs   判决机制，决定色度分量的取舍
  // 如果色度分量的残差小于阈值_CHROMA_COEFF_COST_，则将其全部置成零。色度分量对图像质量的影响较小。
  if(coeff_cost < _CHROMA_COEFF_COST_)
  {
    cr_cbp_tmp = 0 ;
    for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
    {
      for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
      {
        b4      = 2*(n2/4) + (n1/4);
        ACLevel = img->cofAC[uv+4][b4][0];//并不是赋值语句，表示地址共用，ACLevel是cofAC[uv+4][b4][0]的引用
        ACRun   = img->cofAC[uv+4][b4][1];
        if( DCcoded == 0) currMB->cbp_blk &= ~(0xf0000 << (uv << 2));  // if no chroma DC's: then reset coded-bits of this chroma subblock
        nn0 = (n1>>2) + (uv<<1);//什么功能?是否可以舍去或注释掉   简单的测试可以注释掉
        nn1 = 4 + (n2>>2) ;
        ACLevel[0] = 0;
        for (coeff_ctr=1; coeff_ctr < 16; coeff_ctr++)// ac coeff  AC系数置为零
        {

          if (img->field_picture || ( img->MbaffFrameFlag && currMB->mb_field )) 
          {  // Alternate scan for field coding
            i=FIELD_SCAN[coeff_ctr][0];
            j=FIELD_SCAN[coeff_ctr][1];
          }
          else 
          {
            i=SNGL_SCAN[coeff_ctr][0];
            j=SNGL_SCAN[coeff_ctr][1];
          }
          img->m7[n1+i][n2+j]=0;
          ACLevel[coeff_ctr] = 0;
        }
      }
    }
  }
  if(cr_cbp_tmp==2)
    cr_cbp = 2;
  //反变化     IDCT.
  
      //     Horizontal.
  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {
      for (j=0; j < BLOCK_SIZE; j++)
      {
        for (i=0; i < BLOCK_SIZE; i++)
        {
          m5[i]=img->m7[n1+i][n2+j];
        }
        m6[0]=(m5[0]+m5[2]);
        m6[1]=(m5[0]-m5[2]);
        m6[2]=(m5[1]>>1)-m5[3];
        m6[3]=m5[1]+(m5[3]>>1);

        for (i=0; i < 2; i++)
        {
          i1=3-i;
          img->m7[n1+i][n2+j]=m6[i]+m6[i1];
          img->m7[n1+i1][n2+j]=m6[i]-m6[i1];
        }
      }

      //     Vertical.
      for (i=0; i < BLOCK_SIZE; i++)
      {
        for (j=0; j < BLOCK_SIZE; j++)
        {
          m5[j]=img->m7[n1+i][n2+j];
        }
        m6[0]=(m5[0]+m5[2]);
        m6[1]=(m5[0]-m5[2]);
        m6[2]=(m5[1]>>1)-m5[3];
        m6[3]=m5[1]+(m5[3]>>1);

        for (j=0; j < 2; j++)
        {
          j2=3-j;
          img->m7[n1+i][n2+j] =min(255,max(0,(m6[j]+m6[j2]+(img->mpr[n1+i][n2+j] <<DQ_BITS)+DQ_ROUND)>>DQ_BITS));
          img->m7[n1+i][n2+j2]=min(255,max(0,(m6[j]-m6[j2]+(img->mpr[n1+i][n2+j2]<<DQ_BITS)+DQ_ROUND)>>DQ_BITS));
        }
      }
    }
  }

  //  Decoded block moved to memory
  for (j=0; j < BLOCK_SIZE*2; j++)
    for (i=0; i < BLOCK_SIZE*2; i++)
      enc_picture->imgUV[uv][img->pix_c_y+j][img->pix_c_x+i]= img->m7[i][j];

  return cr_cbp;
}


/*!
 ************************************************************************
 * \brief
 *    The routine performs transform,quantization,inverse transform, adds the diff.
 *    to the prediction and writes the result to the decoded luma frame. Includes the
 *    RD constrained quantization also.
 *
 * \par Input:
 *    block_x,block_y: Block position inside a macro block (0,4,8,12).
 *
 * \par Output:
 *    nonzero: 0 if no levels are nonzero.  1 if there are nonzero levels.              \n
 *    coeff_cost: Counter for nonzero coefficients, used to discard expencive levels.
 *
 *对mpr保存的预测出的像素值进行处理  牵扯到SP帧，属于高级档次，不做研究
 ************************************************************************
 */
int dct_luma_sp(int block_x,int block_y,int *coeff_cost)
{
  int sign(int a,int b);//去掉没错

  int i,j,i1,j1,ilev,m5[4],m6[4],coeff_ctr;
  int qp_const,level,scan_pos,run;
  int nonzero;

  int predicted_block[BLOCK_SIZE][BLOCK_SIZE],c_err,qp_const2;
  int qp_per,qp_rem,q_bits;
  int qp_per_sp,qp_rem_sp,q_bits_sp;

  int   pos_x   = block_x/BLOCK_SIZE;
  int   pos_y   = block_y/BLOCK_SIZE;
  int   b8      = 2*(pos_y/2) + (pos_x/2);
  int   b4      = 2*(pos_y%2) + (pos_x%2);
  int*  ACLevel = img->cofAC[b8][b4][0];
  int*  ACRun   = img->cofAC[b8][b4][1];
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  // For encoding optimization  编码最优化 
  int c_err1, c_err2, level1, level2;
  double D_dis1, D_dis2;
  int len, info;
  double lambda_mode   = 0.85 * pow (2, (currMB->qp - SHIFT_QP)/3.0) * 4; /*指数*/

  qp_per    = (currMB->qp-MIN_QP)/6;
  qp_rem    = (currMB->qp-MIN_QP)%6;
  q_bits    = Q_BITS+qp_per;
  qp_per_sp    = (currMB->qpsp-MIN_QP)/6;
  qp_rem_sp    = (currMB->qpsp-MIN_QP)%6;
  q_bits_sp    = Q_BITS+qp_per_sp;
  
  qp_const=(1<<q_bits)/6;    // inter  帧间
  qp_const2=(1<<q_bits_sp)/2;  //sp_pred  SP帧帧间预测

  
  for (j=0; j< BLOCK_SIZE; j++)
    for (i=0; i< BLOCK_SIZE; i++)
    {
      img->m7[i][j]+=img->mpr[i+block_x][j+block_y];
      predicted_block[i][j]=img->mpr[i+block_x][j+block_y];
    }
	//	Horizontal transform

  for (j=0; j < BLOCK_SIZE; j++)
  {
    for (i=0; i < 2; i++)
    {
      i1=3-i;
      m5[i]=img->m7[i][j]+img->m7[i1][j];
      m5[i1]=img->m7[i][j]-img->m7[i1][j];
    }
    img->m7[0][j]=(m5[0]+m5[1]);
    img->m7[2][j]=(m5[0]-m5[1]);
    img->m7[1][j]=m5[3]*2+m5[2];
    img->m7[3][j]=m5[3]-m5[2]*2;
  }

  //  Vertival transform

  for (i=0; i < BLOCK_SIZE; i++)
  {
    for (j=0; j < 2; j++)
    {
      j1=3-j;
      m5[j]=img->m7[i][j]+img->m7[i][j1];
      m5[j1]=img->m7[i][j]-img->m7[i][j1];
    }
    img->m7[i][0]=(m5[0]+m5[1]);
    img->m7[i][2]=(m5[0]-m5[1]);
    img->m7[i][1]=m5[3]*2+m5[2];
    img->m7[i][3]=m5[3]-m5[2]*2;
  }
  //  Horizontal transform

  for (j=0; j < BLOCK_SIZE; j++)
  {
    for (i=0; i < 2; i++)
    {
      i1=3-i;
      m5[i]=predicted_block[i][j]+predicted_block[i1][j];
      m5[i1]=predicted_block[i][j]-predicted_block[i1][j];
    }
    predicted_block[0][j]=(m5[0]+m5[1]);
    predicted_block[2][j]=(m5[0]-m5[1]);
    predicted_block[1][j]=m5[3]*2+m5[2];
    predicted_block[3][j]=m5[3]-m5[2]*2;
  }

  //  Vertival transform

  for (i=0; i < BLOCK_SIZE; i++)
  {
    for (j=0; j < 2; j++)
    {
      j1=3-j;
      m5[j]=predicted_block[i][j]+predicted_block[i][j1];
      m5[j1]=predicted_block[i][j]-predicted_block[i][j1];
    }
    predicted_block[i][0]=(m5[0]+m5[1]);
    predicted_block[i][2]=(m5[0]-m5[1]);
    predicted_block[i][1]=m5[3]*2+m5[2];
    predicted_block[i][3]=m5[3]-m5[2]*2;
  }

  // Quant
  nonzero=FALSE;

  run=-1;
  scan_pos=0;
  
  for (coeff_ctr=0;coeff_ctr < 16;coeff_ctr++)     // 8 times if double scan, 16 normal scan
  {

    if (img->field_picture || ( mb_adaptive && img->field_mode )) 
    {  // Alternate scan for field coding
        i=FIELD_SCAN[coeff_ctr][0];
        j=FIELD_SCAN[coeff_ctr][1];
    }
    else 
    {
        i=SNGL_SCAN[coeff_ctr][0];
        j=SNGL_SCAN[coeff_ctr][1];
    }
    
    run++;
    ilev=0;
    
    // decide prediction  选择预测方式
    
    // case 1  具体见【陈靖:深入理解视频编解码技术第150页】
    level1 = (abs (predicted_block[i][j]) * quant_coef[qp_rem_sp][i][j] + qp_const2) >> q_bits_sp; 
    level1 = (level1 << q_bits_sp) / quant_coef[qp_rem_sp][i][j];  //反量化               
    c_err1 = img->m7[i][j]-sign(level1, predicted_block[i][j]);    //反量化后的DCT系数与原始DCT系数的差值               
    level1 = (abs (c_err1) * quant_coef[qp_rem][i][j] + qp_const) >> q_bits;   //量化  量化步长改变
    
    // case 2
    c_err2=img->m7[i][j]-predicted_block[i][j];
    level2 = (abs (c_err2) * quant_coef[qp_rem][i][j] + qp_const) >> q_bits;
    
    // select prediction
    if ((level1 != level2) && (level1 != 0) && (level2 != 0))
    {
      D_dis1 = img->m7[i][j] - ((sign(level1,c_err1)*dequant_coef[qp_rem][i][j]*A[i][j]<< qp_per) >>6) - predicted_block[i][j]; 
      levrun_linfo_inter(level1, run, &len, &info);
      D_dis1 = D_dis1*D_dis1 + lambda_mode * len;
      
      D_dis2 = img->m7[i][j] - ((sign(level2,c_err2)*dequant_coef[qp_rem][i][j]*A[i][j]<< qp_per) >>6) - predicted_block[i][j]; 
      levrun_linfo_inter(level2, run, &len, &info);
      D_dis2 = D_dis2 * D_dis2 + lambda_mode * len;
      
      if (D_dis1 == D_dis2)
        level = (abs(level1) < abs(level2)) ? level1 : level2;
      else
      {
        if (D_dis1 < D_dis2)
          level = level1;
        else
          level = level2;
      }
      c_err = (level == level1) ? c_err1 : c_err2;
    }
    else if (level1 == level2)
    {
      level = level1;
      c_err = c_err1;
    }
    else
    {
      level = (level1 == 0) ? level1 : level2;
      c_err = (level1 == 0) ? c_err1 : c_err2;
    }
    
    if (level != 0)
    {
      nonzero=TRUE;
      if (level > 1)
        *coeff_cost += MAX_VALUE;                // set high cost, shall not be discarded
      else
        *coeff_cost += COEFF_COST[run];
      ACLevel[scan_pos] = sign(level,c_err);
      ACRun  [scan_pos] = run;
      ++scan_pos;
      run=-1;                     // reset zero level counter
      ilev=((sign(level,c_err)*dequant_coef[qp_rem][i][j]*A[i][j]<< qp_per) >>6);//反量化
    }
    ilev+=predicted_block[i][j] ; 
    img->m7[i][j] = sign((abs(ilev) * quant_coef[qp_rem_sp][i][j] + qp_const2)>> q_bits_sp, ilev) * dequant_coef[qp_rem_sp][i][j] << qp_per_sp;
  }
  ACLevel[scan_pos] = 0;
  
    
  //     IDCT.
  //     horizontal

  for (j=0; j < BLOCK_SIZE; j++)
  {
    for (i=0; i < BLOCK_SIZE; i++)
    {
      m5[i]=img->m7[i][j];
    }
    m6[0]=(m5[0]+m5[2]);
    m6[1]=(m5[0]-m5[2]);
    m6[2]=(m5[1]>>1)-m5[3];
    m6[3]=m5[1]+(m5[3]>>1);

    for (i=0; i < 2; i++)
    {
      i1=3-i;
      img->m7[i][j]=m6[i]+m6[i1];
      img->m7[i1][j]=m6[i]-m6[i1];
    }
  }

  //  vertical

  for (i=0; i < BLOCK_SIZE; i++)
  {
    for (j=0; j < BLOCK_SIZE; j++)
    {
      m5[j]=img->m7[i][j];
    }
    m6[0]=(m5[0]+m5[2]);
    m6[1]=(m5[0]-m5[2]);
    m6[2]=(m5[1]>>1)-m5[3];
    m6[3]=m5[1]+(m5[3]>>1);

    for (j=0; j < 2; j++)
    {
      j1=3-j;
      img->m7[i][j] =min(255,max(0,(m6[j]+m6[j1]+DQ_ROUND)>>DQ_BITS));
      img->m7[i][j1]=min(255,max(0,(m6[j]-m6[j1]+DQ_ROUND)>>DQ_BITS));
    }
  }

  //  Decoded block moved to frame memory

  for (j=0; j < BLOCK_SIZE; j++)
  for (i=0; i < BLOCK_SIZE; i++)
    enc_picture->imgY[img->pix_y+block_y+j][img->pix_x+block_x+i]=img->m7[i][j];

  return nonzero;
}

/*!
 ************************************************************************
 * \brief
 *    Transform,quantization,inverse transform for chroma.
 *    The main reason why this is done in a separate routine is the
 *    additional 2x2 transform of DC-coeffs. This routine is called
 *    ones for each of the chroma components.
 *
 * \par Input:
 *    uv    : Make difference between the U and V chroma component               \n
 *    cr_cbp: chroma coded block pattern
 *
 * \par Output:
 *    cr_cbp: Updated chroma coded block pattern.
 同上，牵扯到SP帧，不做研究,以后有机会深入研究
 ************************************************************************
 */
int dct_chroma_sp(int uv,int cr_cbp)
{
  int i,j,i1,j2,ilev,n2,n1,j1,mb_y,coeff_ctr,qp_const,c_err,level ,scan_pos,run;
  int m1[BLOCK_SIZE],m5[BLOCK_SIZE],m6[BLOCK_SIZE];
  int coeff_cost;
  int cr_cbp_tmp;
  int predicted_chroma_block[MB_BLOCK_SIZE/2][MB_BLOCK_SIZE/2],qp_const2,mp1[BLOCK_SIZE];
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  int qp_per,qp_rem,q_bits;
  int qp_per_sp,qp_rem_sp,q_bits_sp;

  int   b4;
  int*  DCLevel = img->cofDC[uv+1][0];
  int*  DCRun   = img->cofDC[uv+1][1];
  int*  ACLevel;
  int*  ACRun;

  int c_err1, c_err2, level1, level2;
  int len, info;
  double D_dis1, D_dis2;
  double lambda_mode   = 0.85 * pow (2, (currMB->qp -SHIFT_QP)/3.0) * 4; 


  int qpChroma=Clip3(0, 51, currMB->qp + active_pps->chroma_qp_index_offset);
  int qpChromaSP=Clip3(0, 51, currMB->qpsp + active_pps->chroma_qp_index_offset);

  qp_per    = ((qpChroma<0?qpChroma:QP_SCALE_CR[qpChroma])-MIN_QP)/6;
  qp_rem    = ((qpChroma<0?qpChroma:QP_SCALE_CR[qpChroma])-MIN_QP)%6;
  q_bits    = Q_BITS+qp_per;
  qp_const=(1<<q_bits)/6;    // inter
  qp_per_sp    = ((qpChromaSP<0?currMB->qpsp:QP_SCALE_CR[qpChromaSP])-MIN_QP)/6;
  qp_rem_sp    = ((qpChromaSP<0?currMB->qpsp:QP_SCALE_CR[qpChromaSP])-MIN_QP)%6;
  q_bits_sp    = Q_BITS+qp_per_sp;
  qp_const2    = (1<<q_bits_sp)/2;  //sp_pred
  

  for (j=0; j < MB_BLOCK_SIZE/2; j++)
    for (i=0; i < MB_BLOCK_SIZE/2; i++)
    {
      img->m7[i][j]+=img->mpr[i][j];
      predicted_chroma_block[i][j]=img->mpr[i][j];
    }

  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {

      //  Horizontal transform.
      for (j=0; j < BLOCK_SIZE; j++)
      {
        mb_y=n2+j;
        for (i=0; i < 2; i++)
        {
          i1=3-i;
          m5[i]=img->m7[i+n1][mb_y]+img->m7[i1+n1][mb_y];
          m5[i1]=img->m7[i+n1][mb_y]-img->m7[i1+n1][mb_y];
        }
        img->m7[n1][mb_y]  =(m5[0]+m5[1]);
        img->m7[n1+2][mb_y]=(m5[0]-m5[1]);
        img->m7[n1+1][mb_y]=m5[3]*2+m5[2];
        img->m7[n1+3][mb_y]=m5[3]-m5[2]*2;
      }

      //  Vertical transform.

      for (i=0; i < BLOCK_SIZE; i++)
      {
        j1=n1+i;
        for (j=0; j < 2; j++)
        {
          j2=3-j;
          m5[j]=img->m7[j1][n2+j]+img->m7[j1][n2+j2];
          m5[j2]=img->m7[j1][n2+j]-img->m7[j1][n2+j2];
        }
        img->m7[j1][n2+0]=(m5[0]+m5[1]);
        img->m7[j1][n2+2]=(m5[0]-m5[1]);
        img->m7[j1][n2+1]=m5[3]*2+m5[2];
        img->m7[j1][n2+3]=m5[3]-m5[2]*2;
      }
    }
  }
  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {

      //  Horizontal transform.
      for (j=0; j < BLOCK_SIZE; j++)
      {
        mb_y=n2+j;
        for (i=0; i < 2; i++)
        {
          i1=3-i;
          m5[i]=predicted_chroma_block[i+n1][mb_y]+predicted_chroma_block[i1+n1][mb_y];
          m5[i1]=predicted_chroma_block[i+n1][mb_y]-predicted_chroma_block[i1+n1][mb_y];
        }
        predicted_chroma_block[n1][mb_y]  =(m5[0]+m5[1]);
        predicted_chroma_block[n1+2][mb_y]=(m5[0]-m5[1]);
        predicted_chroma_block[n1+1][mb_y]=m5[3]*2+m5[2];
        predicted_chroma_block[n1+3][mb_y]=m5[3]-m5[2]*2;
      }

      //  Vertical transform.

      for (i=0; i < BLOCK_SIZE; i++)
      {
        j1=n1+i;
        for (j=0; j < 2; j++)
        {
          j2=3-j;
          m5[j]=predicted_chroma_block[j1][n2+j]+predicted_chroma_block[j1][n2+j2];
          m5[j2]=predicted_chroma_block[j1][n2+j]-predicted_chroma_block[j1][n2+j2];
        }
        predicted_chroma_block[j1][n2+0]=(m5[0]+m5[1]);
        predicted_chroma_block[j1][n2+2]=(m5[0]-m5[1]);
        predicted_chroma_block[j1][n2+1]=m5[3]*2+m5[2];
        predicted_chroma_block[j1][n2+3]=m5[3]-m5[2]*2;
      }
    }
  }

  //     2X2 transform of DC coeffs.
  m1[0]=(img->m7[0][0]+img->m7[4][0]+img->m7[0][4]+img->m7[4][4]);
  m1[1]=(img->m7[0][0]-img->m7[4][0]+img->m7[0][4]-img->m7[4][4]);
  m1[2]=(img->m7[0][0]+img->m7[4][0]-img->m7[0][4]-img->m7[4][4]);
  m1[3]=(img->m7[0][0]-img->m7[4][0]-img->m7[0][4]+img->m7[4][4]);

  //     2X2 transform of DC coeffs.
  mp1[0]=(predicted_chroma_block[0][0]+predicted_chroma_block[4][0]+predicted_chroma_block[0][4]+predicted_chroma_block[4][4]);
  mp1[1]=(predicted_chroma_block[0][0]-predicted_chroma_block[4][0]+predicted_chroma_block[0][4]-predicted_chroma_block[4][4]);
  mp1[2]=(predicted_chroma_block[0][0]+predicted_chroma_block[4][0]-predicted_chroma_block[0][4]-predicted_chroma_block[4][4]);
  mp1[3]=(predicted_chroma_block[0][0]-predicted_chroma_block[4][0]-predicted_chroma_block[0][4]+predicted_chroma_block[4][4]);

  run=-1;
  scan_pos=0;

  for (coeff_ctr=0; coeff_ctr < 4; coeff_ctr++)
  {
    run++;
    ilev=0;

  // case 1
    c_err1 = (abs (mp1[coeff_ctr]) * quant_coef[qp_rem_sp][0][0] + 2 * qp_const2) >> (q_bits_sp + 1);
    c_err1 = (c_err1 << (q_bits_sp + 1)) / quant_coef[qp_rem_sp][0][0];
    c_err1 = m1[coeff_ctr] - sign(c_err1, mp1[coeff_ctr]);
    level1 = (abs(c_err1) * quant_coef[qp_rem][0][0] + 2 * qp_const) >> (q_bits+1);

  // case 2
    c_err2 = m1[coeff_ctr] - mp1[coeff_ctr];
    level2 = (abs(c_err2) * quant_coef[qp_rem][0][0] + 2 * qp_const) >> (q_bits+1);

  if (level1 != level2 && level1 != 0 && level2 != 0)
  {
    D_dis1 = m1[coeff_ctr] - ((sign(level1,c_err1)*dequant_coef[qp_rem][0][0]*A[0][0]<< qp_per) >>5)- mp1[coeff_ctr];
    levrun_linfo_c2x2(level1, run, &len, &info);
    D_dis1 = D_dis1 * D_dis1 + lambda_mode * len;

    D_dis2 = m1[coeff_ctr] - ((sign(level2,c_err2)*dequant_coef[qp_rem][0][0]*A[0][0]<< qp_per) >>5)- mp1[coeff_ctr];
    levrun_linfo_c2x2(level2, run, &len, &info);
    D_dis2 = D_dis2 * D_dis2 + lambda_mode * len;

    if (D_dis1 == D_dis2)
      level = (abs(level1) < abs(level2)) ? level1 : level2;
    else
    {
      if (D_dis1 < D_dis2)
        level = level1;
      else
        level = level2;
    }
    c_err = (level == level1) ? c_err1 : c_err2;
  }
  else if (level1 == level2)
  {
    level = level1;
    c_err = c_err1;
  }
  else
  {
    level = (level1 == 0) ? level1 : level2;
    c_err = (level1 == 0) ? c_err1 : c_err2;
  }

    if (level  != 0)
    {
      currMB->cbp_blk |= 0xf0000 << (uv << 2) ;  // if one of the 2x2-DC levels is != 0 the coded-bit
      cr_cbp=max(1,cr_cbp);
      DCLevel[scan_pos] = sign(level ,c_err);
      DCRun  [scan_pos] = run;
      scan_pos++;
      run=-1;
      ilev=((sign(level,c_err)*dequant_coef[qp_rem][0][0]*A[0][0]<< qp_per) >>5);
    }
    ilev+= mp1[coeff_ctr];
    m1[coeff_ctr]=sign((abs(ilev)  * quant_coef[qp_rem_sp][0][0] + 2 * qp_const2) >> (q_bits_sp+1), ilev) * dequant_coef[qp_rem_sp][0][0] << qp_per_sp;
  }
  DCLevel[scan_pos] = 0;

  //  Invers transform of 2x2 DC levels

  img->m7[0][0]=(m1[0]+m1[1]+m1[2]+m1[3])/2;
  img->m7[4][0]=(m1[0]-m1[1]+m1[2]-m1[3])/2;
  img->m7[0][4]=(m1[0]+m1[1]-m1[2]-m1[3])/2;
  img->m7[4][4]=(m1[0]-m1[1]-m1[2]+m1[3])/2;

  //     Quant of chroma AC-coeffs.
  coeff_cost=0;
  cr_cbp_tmp=0;

  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {
      b4      = 2*(n2/4) + (n1/4);
      ACLevel = img->cofAC[uv+4][b4][0];
      ACRun   = img->cofAC[uv+4][b4][1];

      run      = -1;
      scan_pos =  0;

      for (coeff_ctr=1; coeff_ctr < 16; coeff_ctr++)// start change rd_quant
      {

        if (img->field_picture || ( mb_adaptive && img->field_mode )) 
        {  // Alternate scan for field coding
          i=FIELD_SCAN[coeff_ctr][0];
          j=FIELD_SCAN[coeff_ctr][1];
        }
        else 
        {
          i=SNGL_SCAN[coeff_ctr][0];
          j=SNGL_SCAN[coeff_ctr][1];
        }
        ++run;
        ilev=0;

    // quantization on prediction
    c_err1 = (abs(predicted_chroma_block[n1+i][n2+j]) * quant_coef[qp_rem_sp][i][j] + qp_const2) >> q_bits_sp;
    c_err1 = (c_err1 << q_bits_sp) / quant_coef[qp_rem_sp][i][j];
    c_err1 = img->m7[n1+i][n2+j] - sign(c_err1, predicted_chroma_block[n1+i][n2+j]);
    level1 = (abs(c_err1) * quant_coef[qp_rem][i][j] + qp_const) >> q_bits;

    // no quantization on prediction
    c_err2 = img->m7[n1+i][n2+j] - predicted_chroma_block[n1+i][n2+j];
    level2 = (abs(c_err2) * quant_coef[qp_rem][i][j] + qp_const) >> q_bits;

    if (level1 != level2 && level1 != 0 && level2 != 0)
    {
      D_dis1 = img->m7[n1+i][n2+j] - ((sign(level1,c_err1)*dequant_coef[qp_rem][i][j]*A[i][j]<< qp_per) >>6) - predicted_chroma_block[n1+i][n2+j]; 

      levrun_linfo_inter(level1, run, &len, &info);
      D_dis1 = D_dis1 * D_dis1 + lambda_mode * len;

      D_dis2 = img->m7[n1+i][n2+j] - ((sign(level2,c_err2)*dequant_coef[qp_rem][i][j]*A[i][j]<< qp_per) >>6) - predicted_chroma_block[n1+i][n2+j]; 
      levrun_linfo_inter(level2, run, &len, &info);
      D_dis2 = D_dis2 * D_dis2 + lambda_mode * len;
      
      if (D_dis1 == D_dis2)
        level = (abs(level1) < abs(level2)) ? level1 : level2;
      else
      {
        if (D_dis1 < D_dis2)
          level = level1;
        else
          level = level2;
      }
      c_err = (level == level1) ? c_err1 : c_err2;
    }
    else if (level1 == level2)
    {
      level = level1;
      c_err = c_err1;
    }
    else
    {
      level = (level1 == 0) ? level1 : level2;
      c_err = (level1 == 0) ? c_err1 : c_err2;
    }

        if (level  != 0)
        {
          currMB->cbp_blk |=  1 << (16 + (uv << 2) + ((n2 >> 1) + (n1 >> 2))) ;
          if (level > 1)
            coeff_cost += MAX_VALUE;                // set high cost, shall not be discarded
          else
            coeff_cost += COEFF_COST[run];

          cr_cbp_tmp=2;
          ACLevel[scan_pos] = sign(level,c_err);
          ACRun  [scan_pos] = run;
          ++scan_pos;
          run=-1;
          ilev=((sign(level,c_err)*dequant_coef[qp_rem][i][j]*A[i][j]<< qp_per) >>6);
        }
        ilev+=predicted_chroma_block[n1+i][n2+j];
        img->m7[n1+i][n2+j] = sign((abs(ilev) * quant_coef[qp_rem_sp][i][j] + qp_const2) >> q_bits_sp,ilev) * dequant_coef[qp_rem_sp][i][j] << qp_per_sp;
      }
      ACLevel[scan_pos] = 0;
    }
  }

  // * reset chroma coeffs

  if(cr_cbp_tmp==2)
      cr_cbp=2;
  //     IDCT.

      //     Horizontal.
  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {
      for (j=0; j < BLOCK_SIZE; j++)
      {
        for (i=0; i < BLOCK_SIZE; i++)
        {
          m5[i]=img->m7[n1+i][n2+j];
        }
        m6[0]=(m5[0]+m5[2]);
        m6[1]=(m5[0]-m5[2]);
        m6[2]=(m5[1]>>1)-m5[3];
        m6[3]=m5[1]+(m5[3]>>1);

        for (i=0; i < 2; i++)
        {
          i1=3-i;
          img->m7[n1+i][n2+j]=m6[i]+m6[i1];
          img->m7[n1+i1][n2+j]=m6[i]-m6[i1];
        }
      }

      //     Vertical.
      for (i=0; i < BLOCK_SIZE; i++)
      {
        for (j=0; j < BLOCK_SIZE; j++)
        {
          m5[j]=img->m7[n1+i][n2+j];
        }
        m6[0]=(m5[0]+m5[2]);
        m6[1]=(m5[0]-m5[2]);
        m6[2]=(m5[1]>>1)-m5[3];
        m6[3]=m5[1]+(m5[3]>>1);

        for (j=0; j < 2; j++)
        {
          j2=3-j;
          img->m7[n1+i][n2+j] =min(255,max(0,(m6[j]+m6[j2]+DQ_ROUND)>>DQ_BITS));
          img->m7[n1+i][n2+j2]=min(255,max(0,(m6[j]-m6[j2]+DQ_ROUND)>>DQ_BITS));
        }
      }
    }
  }

  //  Decoded block moved to memory
  for (j=0; j < BLOCK_SIZE*2; j++)
    for (i=0; i < BLOCK_SIZE*2; i++)
    {
      enc_picture->imgUV[uv][img->pix_c_y+j][img->pix_c_x+i]= img->m7[i][j];
    }

  return cr_cbp;
}

/*!
 ************************************************************************
 * \brief
 *    The routine performs transform,quantization,inverse transform, adds the diff.
 *    to the prediction and writes the result to the decoded luma frame. Includes the
 *    RD constrained quantization also.
 *
 * \par Input:
 *    block_x,block_y: Block position inside a macro block (0,4,8,12).
 *
 * \par Output:
 *    nonzero: 0 if no levels are nonzero.  1 if there are nonzero levels.            \n
 *    coeff_cost: Counter for nonzero coefficients, used to discard expencive levels.
 ************************************************************************
 */
void copyblock_sp(int block_x,int block_y)
{
  int sign(int a,int b);

  int i,j,i1,j1,m5[4],m6[4];

  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  int predicted_block[BLOCK_SIZE][BLOCK_SIZE];
  int qp_per = (currMB->qpsp-MIN_QP)/6;
  int qp_rem = (currMB->qpsp-MIN_QP)%6;
  int q_bits    = Q_BITS+qp_per;
  int qp_const2 = (1<<q_bits)/2;  //sp_pred

  //  Horizontal transform
  for (j=0; j< BLOCK_SIZE; j++)
    for (i=0; i< BLOCK_SIZE; i++)
    {
      predicted_block[i][j]=img->mpr[i+block_x][j+block_y];
    }

  for (j=0; j < BLOCK_SIZE; j++)
  {
    for (i=0; i < 2; i++)
    {
      i1=3-i;
      m5[i]=predicted_block[i][j]+predicted_block[i1][j];
      m5[i1]=predicted_block[i][j]-predicted_block[i1][j];
    }
    predicted_block[0][j]=(m5[0]+m5[1]);
    predicted_block[2][j]=(m5[0]-m5[1]);
    predicted_block[1][j]=m5[3]*2+m5[2];
    predicted_block[3][j]=m5[3]-m5[2]*2;
  }

  //  Vertival transform

  for (i=0; i < BLOCK_SIZE; i++)
  {
    for (j=0; j < 2; j++)
    {
      j1=3-j;
      m5[j]=predicted_block[i][j]+predicted_block[i][j1];
      m5[j1]=predicted_block[i][j]-predicted_block[i][j1];
    }
    predicted_block[i][0]=(m5[0]+m5[1]);
    predicted_block[i][2]=(m5[0]-m5[1]);
    predicted_block[i][1]=m5[3]*2+m5[2];
    predicted_block[i][3]=m5[3]-m5[2]*2;
  }

  // Quant
  for (j=0;j < BLOCK_SIZE; j++)
    for (i=0; i < BLOCK_SIZE; i++)
       img->m7[i][j]=sign((abs(predicted_block[i][j])* quant_coef[qp_rem][i][j]+qp_const2)>> q_bits,predicted_block[i][j])*dequant_coef[qp_rem][i][j]<<qp_per;

  //     IDCT.
  //     horizontal

  for (j=0;j<BLOCK_SIZE;j++)
  {
    for (i=0;i<BLOCK_SIZE;i++)
    {
      m5[i]=img->m7[i][j];
    }
    m6[0]=(m5[0]+m5[2]);
    m6[1]=(m5[0]-m5[2]);
    m6[2]=(m5[1]>>1)-m5[3];
    m6[3]=m5[1]+(m5[3]>>1);

    for (i=0;i<2;i++)
    {
      i1=3-i;
      img->m7[i][j]=m6[i]+m6[i1];
      img->m7[i1][j]=m6[i]-m6[i1];
    }
  }
  // vertical
  for (i=0;i<BLOCK_SIZE;i++)
  {
    for (j=0;j<BLOCK_SIZE;j++)
      m5[j]=img->m7[i][j];

    m6[0]=(m5[0]+m5[2]);
    m6[1]=(m5[0]-m5[2]);
    m6[2]=(m5[1]>>1)-m5[3];
    m6[3]=m5[1]+(m5[3]>>1);

    for (j=0;j<2;j++)
    {
      j1=3-j;
      img->m7[i][j] =min(255,max(0,(m6[j]+m6[j1]+DQ_ROUND)>>DQ_BITS));
      img->m7[i][j1]=min(255,max(0,(m6[j]-m6[j1]+DQ_ROUND)>>DQ_BITS));
    }
  }

  //  Decoded block moved to frame memory

  for (j=0; j < BLOCK_SIZE; j++)
    for (i=0; i < BLOCK_SIZE; i++)
      enc_picture->imgY[img->pix_y+block_y+j][img->pix_x+block_x+i]=img->m7[i][j];
}
