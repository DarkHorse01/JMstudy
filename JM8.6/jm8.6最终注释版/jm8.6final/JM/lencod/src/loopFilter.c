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

/*!
 *************************************************************************************
 * \file loopFilter.c
 *
 * \brief
 *    Filter to reduce blocking artifacts on a macroblock level.
 *    The filter strengh is QP dependent.
 *
 * \author
 *    Contributors:
 *    - Peter List       Peter.List@t-systems.de:  Original code                                 (13-Aug-2001)
 *    - Jani Lainema     Jani.Lainema@nokia.com:   Some bug fixing, removal of recusiveness      (16-Aug-2001)
 *    - Peter List       Peter.List@t-systems.de:  inplace filtering and various simplifications (10-Jan-2002)
 *    - Anthony Joch     anthony@ubvideo.com:      Simplified switching between filters and 
 *                                                 non-recursive default filter.                 (08-Jul-2002)
 *    - Cristina Gomila  cristina.gomila@thomson.net: Simplification of the chroma deblocking
 *                                                    from JVT-E089                              (21-Nov-2002)
 *************************************************************************************
 */
#include <stdlib.h>
#include <string.h>
#include "global.h"
#include "image.h"
#include "mb_access.h"

extern const byte QP_SCALE_CR[52] ;

byte mixedModeEdgeFlag, fieldModeFilteringFlag;

/*********************************************************************************************************/

#define  IClip( Min, Max, Val) (((Val)<(Min))? (Min):(((Val)>(Max))? (Max):(Val)))

// NOTE: to change the tables below for instance when the QP doubling is changed from 6 to 8 values 
//       send an e-mail to Peter.List@t-systems.com to get a little programm that calculates them automatically 
//alpha,beta是滤波器滤波的阈值,为什么要该阈值，
//是由于H.264规定超过该阈值，就认为是自然物体边界，不需要滤波.
byte ALPHA_TABLE[52]  = {0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,4,4,5,6,  7,8,9,10,12,13,15,17,  20,22,25,28,32,36,40,45,  50,56,63,71,80,90,101,113,  127,144,162,182,203,226,255,255} ;
byte  BETA_TABLE[52]  = {0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,2,2,2,3,  3,3,3, 4, 4, 4, 6, 6,   7, 7, 8, 8, 9, 9,10,10,  11,11,12,12,13,13, 14, 14,   15, 15, 16, 16, 17, 17, 18, 18} ;
//滤波的CLIP范围表
byte CLIP_TAB[52][5]  =
{
  { 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},
  { 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},
  { 0, 0, 0, 0, 0},{ 0, 0, 0, 1, 1},{ 0, 0, 0, 1, 1},{ 0, 0, 0, 1, 1},{ 0, 0, 0, 1, 1},{ 0, 0, 1, 1, 1},{ 0, 0, 1, 1, 1},{ 0, 1, 1, 1, 1},
  { 0, 1, 1, 1, 1},{ 0, 1, 1, 1, 1},{ 0, 1, 1, 1, 1},{ 0, 1, 1, 2, 2},{ 0, 1, 1, 2, 2},{ 0, 1, 1, 2, 2},{ 0, 1, 1, 2, 2},{ 0, 1, 2, 3, 3},
  { 0, 1, 2, 3, 3},{ 0, 2, 2, 3, 3},{ 0, 2, 2, 4, 4},{ 0, 2, 3, 4, 4},{ 0, 2, 3, 4, 4},{ 0, 3, 3, 5, 5},{ 0, 3, 4, 6, 6},{ 0, 3, 4, 6, 6},
  { 0, 4, 5, 7, 7},{ 0, 4, 5, 8, 8},{ 0, 4, 6, 9, 9},{ 0, 5, 7,10,10},{ 0, 6, 8,11,11},{ 0, 6, 8,13,13},{ 0, 7,10,14,14},{ 0, 8,11,16,16},
  { 0, 9,12,18,18},{ 0,10,13,20,20},{ 0,11,15,23,23},{ 0,13,17,25,25}
};

void GetStrength(byte Strength[16],ImageParameters *img,int MbQAddr,int dir,int edge, int mvlimit);
void EdgeLoop(byte** Img, byte Strength[16],ImageParameters *img, int MbQAddr, int AlphaC0Offset, int BetaOffset, int dir, int edge, int width, int yuv);
void DeblockMb(ImageParameters *img, byte **imgY, byte ***imgUV, int MbQAddr) ;

/*!
 *****************************************************************************************
 * \brief
 *    Filter all macroblocks in order of increasing macroblock address.
 *****************************************************************************************
 */
void DeblockFrame(ImageParameters *img, byte **imgY, byte ***imgUV)
{
  unsigned i;

  for (i=0; i<img->PicSizeInMbs; i++)
  {
    DeblockMb( img, imgY, imgUV, i ) ; //以宏块为单位滤波
  }
} 


/*!
 *****************************************************************************************
 * \brief
 *    Deblocking filter for one macroblock.
 *****************************************************************************************
*/
void DeblockMb(ImageParameters *img, byte **imgY, byte ***imgUV, int MbQAddr)
{
  int           EdgeCondition; //判断是否当前滤波的位置处于图像边界
  int           dir,edge;             //滤波方向(垂直or水平)
  byte          Strength[16]; 
  int           mb_x, mb_y;

  int           filterLeftMbEdgeFlag;
  int           filterTopMbEdgeFlag;
  int           fieldModeMbFlag;
  int           mvlimit=4;
  int           i, StrengthSum;
  Macroblock    *MbQ;
  
  img->DeblockCall = 1;    //设置滤波标志，表明该图像正在做去块滤波
  get_mb_pos (MbQAddr, &mb_x, &mb_y); //得到宏块绝对地址

  //因为对于一幅图像的第一行和第一列是没有上宏块和左宏块的,所以要做下边界判断
  filterLeftMbEdgeFlag  = (mb_x != 0); //宏块左边界是否滤波
  filterTopMbEdgeFlag   = (mb_y != 0);//宏块上边界是否滤波

  MbQ  = &(img->mb_data[MbQAddr]) ; //得到当前宏块的参数，MbQ为当前要滤波的宏块

  if (img->MbaffFrameFlag && mb_y==16 && MbQ->mb_field)
    filterTopMbEdgeFlag = 0;  //第一行场宏块对的底场宏块上边界不滤波

  fieldModeMbFlag       = (img->structure!=FRAME) || (img->MbaffFrameFlag && MbQ->mb_field);
  if (fieldModeMbFlag)  //如果是场宏块，mvlimit为2，用在计算边缘强度
    mvlimit = 2;

  //return, if filter is disabled
  //LFDisableIdc -- 配置文件中的LoopFilterDisable，1为禁用滤波
  if (MbQ->LFDisableIdc==1) {
    img->DeblockCall = 0;
    return;
  }

  if (MbQ->LFDisableIdc==2)
  {
    // don't filter at slice boundaries
    //当LFDisableIdc等于2时，来自不同Slice的宏块视为不可用
    //在程序中判断一个宏块的ABCD是否可用，就是判断是否在同一个Slice中
    filterLeftMbEdgeFlag = MbQ->mbAvailA;
    filterTopMbEdgeFlag  = MbQ->mbAvailB;;
  }

  img->current_mb_nr = MbQAddr; 
  CheckAvailabilityOfNeighbors();  //重新检查邻块可用性，上一次是在start_macroblock中
  //之所以重新检查是因为有可能需要忽视slice边界的存在，而MbQ->LFDisableIdc==2的情况
  //通过上面的if语句单独考虑

  //滤波的顺序是先垂直后水平,dir 0为垂直，1为水平
  for( dir=0 ; dir<2 ; dir++ )// vertical edges, than horicontal edges
  {
    //第一个条件:最上面一行宏块的上边界(e)不滤波，第二个条件:最左边一行宏块左边界(a)不滤波
    EdgeCondition = (dir && filterTopMbEdgeFlag) || (!dir && filterLeftMbEdgeFlag); // can not filter beyond picture boundaries
    for( edge=0 ; edge<4 ; edge++ )                                    // first 4 vertical strips of 16 pel
    {                                                                                         // then  4 horicontal
      if( edge || EdgeCondition ) //判断边界条件图像边界不做deblocking
      {
        //这里一条边可以得到16个边界条件，每个点一个了，
        //其实是不需要的，一条边一个bS，不过这样实现也不影响判断，只是计算冗余了
        GetStrength(Strength,img,MbQAddr,dir,edge, mvlimit); // Strength for 4 blks in 1 stripe
        StrengthSum = Strength[0];
        for (i = 1; i < 16; i++) StrengthSum += Strength[i];
        //StrengthSum为0时不做滤波，其他情况都要做
        if( StrengthSum )                      // only if one of the 16 Strength bytes is != 0
        {
          EdgeLoop( imgY, Strength, img, MbQAddr, MbQ->LFAlphaC0Offset, MbQ->LFBetaOffset, dir, edge, img->width, 0) ; 
          if( (imgUV != NULL) && !(edge & 1) )//edge为0,2时进行色度边界滤波
          {
            EdgeLoop( imgUV[0], Strength, img, MbQAddr, MbQ->LFAlphaC0Offset, MbQ->LFBetaOffset, dir, edge/2, img->width_cr, 1 ) ; 
            EdgeLoop( imgUV[1], Strength, img, MbQAddr, MbQ->LFAlphaC0Offset, MbQ->LFBetaOffset, dir, edge/2, img->width_cr, 1 ) ; 
          }
        }

        //下面是在MBAFF时的extra horizontal edge滤波
        //1、是水平边界，2、非图像边界，3、MbQ是帧宏块编码，4、上方邻块是场宏块编码
        // 以上条件同时满足，则执行下面的extra horizontal edge滤波
        if (dir && !edge && !MbQ->mb_field && mixedModeEdgeFlag) {
          // this is the extra horizontal edge between a frame macroblock pair and a field above it
          img->DeblockCall = 2;  
          GetStrength(Strength,img,MbQAddr,dir,4, mvlimit); // Strength for 4 blks in 1 stripe
          if( *((int*)Strength) )                      // only if one of the 4 Strength bytes is != 0
          {
            EdgeLoop( imgY, Strength, img, MbQAddr, MbQ->LFAlphaC0Offset, MbQ->LFBetaOffset, dir, 4, img->width, 0) ; 
            if( (imgUV != NULL) && !(edge & 1) )
            {
              EdgeLoop( imgUV[0], Strength, img, MbQAddr, MbQ->LFAlphaC0Offset, MbQ->LFBetaOffset, dir, 4, img->width_cr, 1 ) ; 
              EdgeLoop( imgUV[1], Strength, img, MbQAddr, MbQ->LFAlphaC0Offset, MbQ->LFBetaOffset, dir, 4, img->width_cr, 1 ) ; 
            }
          }
          img->DeblockCall = 1;
        }

      }
    }//end edge
  }//end loop dir
  img->DeblockCall = 0; //置为0，表明去块滤波过程已结束

}

  /*!
 *********************************************************************************************
 * \brief
 *    returns a buffer of 16 Strength values for one stripe in a mb (for different Frame types)
 *********************************************************************************************
 */

int  ININT_STRENGTH[4] = {0x04040404, 0x03030303, 0x03030303, 0x03030303} ; 
byte BLK_NUM[2][4][4]  = {{{0,4,8,12},{1,5,9,13},{2,6,10,14},{3,7,11,15}},{{0,1,2,3},{4,5,6,7},{8,9,10,11},{12,13,14,15}}} ;
byte BLK_4_TO_8[16]    = {0,0,1,1,0,0,1,1,2,2,3,3,2,2,3,3} ;
#define ANY_INTRA (MbP->mb_type==I4MB||MbP->mb_type==I16MB||MbP->mb_type==IPCM||MbQ->mb_type==I4MB||MbQ->mb_type==I16MB||MbQ->mb_type==IPCM)
/*
  输出Bs，处理以4x4块为单位
  dir=0表示现在处理的是垂直方向的边界，dir=1表示现在处理的是水平方向的边界
  Q表示当前块,P表示左方或上方的邻近块
*/
void GetStrength(byte Strength[16],ImageParameters *img,int MbQAddr,int dir,int edge, int mvlimit)
{
  int    blkP, blkQ, idx;
  int    blk_x, blk_x2, blk_y, blk_y2 ;
  int    ***list0_mv = enc_picture->mv[LIST_0];  // 1/4象素为单位，list0
  int    ***list1_mv = enc_picture->mv[LIST_1];  // 供b帧使用的list1
  /*  ref_idx:参考图像在参考图像列表里面的索引号
      ref_pic_id：参考图像的图像号
  */
  int    **list0_refIdxArr = enc_picture->ref_idx[LIST_0]; 
  int    **list1_refIdxArr = enc_picture->ref_idx[LIST_1];  
  int64    **list0_refPicIdArr = enc_picture->ref_pic_id[LIST_0];
  int64    **list1_refPicIdArr = enc_picture->ref_pic_id[LIST_1];
  int    xQ, xP, yQ, yP;
  int    mb_x, mb_y;
  Macroblock    *MbQ;
  Macroblock    *MbP;
  PixelPos pixP;

  MbQ = &(img->mb_data[MbQAddr]);

  for( idx=0 ; idx<16 ; idx++ )
  {                                                                
    xQ = dir ? idx : edge << 2;  //滤波边界上像素的横坐标,按照横向和纵向边界画出图就一目了然啦
    yQ = dir ? (edge < 4 ? edge << 2 : 1) : idx;  //滤波边界上像素的坐标纵坐标
    //检查相邻像素可用性，分两种情况:
    //dir为0时,因为是垂直滤波所以检查左边相邻像素的可用性
    //dir为1时,因为是水平滤波所以检查上边相邻像素的可用性，得到的坐标存入pixP
    getNeighbour(MbQAddr, xQ - (1 - dir), yQ - dir, 1, &pixP);
    xP = pixP.x;
    yP = pixP.y;
    MbP = &(img->mb_data[pixP.mb_addr]);//得到P宏块信息
   //Q和P宏块的帧场模式不同时mixedModeEdgeFlag为1
    mixedModeEdgeFlag = MbQ->mb_field != MbP->mb_field;
    
    blkQ = ((yQ>>2)<<2) + (xQ>>2);//((yQ>>2)<<2)可能取值0,4,8,12 (xQ>>2)可能取值0,1,2,3
    blkP = ((yP>>2)<<2) + (xP>>2);//即blkQ表示当前Q点所属4x4块在Q宏块中的序号，blkP表示P点所属4x4块在P宏块中的序号
    

    if ((img->type==SP_SLICE)||(img->type==SI_SLICE) )
    {
      Strength[idx] = (edge == 0 && (((!img->MbaffFrameFlag && (img->structure==FRAME)) ||
      (img->MbaffFrameFlag && !MbP->mb_field && !MbQ->mb_field)) ||
      ((img->MbaffFrameFlag || (img->structure!=FRAME)) && !dir))) ? 4 : 3;
    }
    else
    {
      // Start with Strength=3. or Strength=4 for Mb-edge
      //当边界两边一个或两个块为帧内预测并且边界为宏块边界，bS=4
      //若不是宏块边界，则bS=3，edge == 0为宏块边界判断条件
      Strength[idx] = (edge == 0 && (((!img->MbaffFrameFlag && (img->structure==FRAME)) ||
        (img->MbaffFrameFlag && !MbP->mb_field && !MbQ->mb_field)) ||
        ((img->MbaffFrameFlag || (img->structure!=FRAME)) && !dir))) ? 4 : 3;

      //如果P和Q都不是I宏块才执行下面的代码，将bS设置为2,1,0
      if(  !(MbP->mb_type==I4MB || MbP->mb_type==I16MB || MbP->mb_type==IPCM)
        && !(MbQ->mb_type==I4MB || MbQ->mb_type==I16MB || MbQ->mb_type==IPCM) )
      {
        // cbp_blk用来记录宏块内每个4x4小块是否包含量化系数，每个小块用一位表示，
        // 1 << blkQ然后跟cbp_blk做"与"操作用来检验blkQ小块是否有量化系数,
        // 如果Q小块或者P小块有量化系数，则bS设为2
        if( ((MbQ->cbp_blk &  (1 << blkQ )) != 0) || ((MbP->cbp_blk &  (1 << blkP)) != 0) )
          Strength[idx] = 2 ;
        else
        {                                                     // if no coefs, but vector difference >= 1 set Strength=1 
          // if this is a mixed mode edge then one set of reference pictures will be frame and the
          // other will be field
          if (mixedModeEdgeFlag)//等于0
          {
            (Strength[idx] = 1);
          }
          else
          {
            get_mb_block_pos (MbQAddr, &mb_x, &mb_y); //mb_x和mb_y是当前宏块的以宏块为单位的坐标
            blk_y  = (mb_y<<2) + (blkQ >> 2) ; //Q块在整个图像中的绝对坐标
            blk_x  = (mb_x<<2) + (blkQ  & 3) ; 
            blk_y2 = pixP.pos_y >> 2;  //P块在整个图像中的绝对坐标
            blk_x2 = pixP.pos_x >> 2;
            if( (img->type == B_SLICE) ) //B帧的情况
            {
              int64 ref_p0,ref_p1,ref_q0,ref_q1;
              //ref_p0:p块前向参考帧号；ref_p1:p块后向参考帧号
              //ref_q0:q块前向参考帧号；ref_q1:q块后向参考帧号；-1代表参考帧不可用
              ref_p0 = list0_refIdxArr[blk_x][blk_y]<0 ? -1 : list0_refPicIdArr[blk_x][blk_y];
              ref_q0 = list0_refIdxArr[blk_x2][blk_y2]<0 ? -1 : list0_refPicIdArr[blk_x2][blk_y2];
              ref_p1 = list1_refIdxArr[blk_x][blk_y]<0 ? -1 : list1_refPicIdArr[blk_x][blk_y];
              ref_q1 = list1_refIdxArr[blk_x2][blk_y2]<0 ? -1 : list1_refPicIdArr[blk_x2][blk_y2];
              //P、Q前后向参考帧号对应相等或交叉相等(包含4个参考帧号全部相等的情况)
              if ( ((ref_p0==ref_q0) && (ref_p1==ref_q1)) ||
                ((ref_p0==ref_q1) && (ref_p1==ref_q0))) 
              {
                Strength[idx]=0;
                // L0 and L1 reference pictures of p are different; q0 as well
                if (ref_p0 != ref_p1) //P的前后参考帧号不相等，排除了4帧全相等的可能
                { 
                  // compare MV for the same reference picture
                  if (ref_p0==ref_q0)  //如果是前后参考帧号对应相等的情况，则比较对应的前后向MV，只要有超过一个整像素差别的情况，则Strength赋为1
                  {
                    Strength[idx] =  (abs( list0_mv[blk_x][blk_y][0] - list0_mv[blk_x2][blk_y2][0]) >= 4) |  //P、Q前向MV横坐标差值大于一个整像素
                      (abs( list0_mv[blk_x][blk_y][1] - list0_mv[blk_x2][blk_y2][1]) >= mvlimit) |  //P、Q前向MV纵坐标差大于一个整像素，参考下面P帧情况的描述
                      (abs( list1_mv[blk_x][blk_y][0] - list1_mv[blk_x2][blk_y2][0]) >= 4) |  //P、Q后向MV横坐标差值大于一个整像素
                      (abs( list1_mv[blk_x][blk_y][1] - list1_mv[blk_x2][blk_y2][1]) >= mvlimit);   //P、Q后向MV横坐标差值大于一个整像素
                  }
                  else  //否则，为交叉相等的情况，P的前后MV分别与Q的后前MV比较
                  {
                    Strength[idx] =  (abs( list0_mv[blk_x][blk_y][0] - list1_mv[blk_x2][blk_y2][0]) >= 4) | //该赋值与上面类似
                      (abs( list0_mv[blk_x][blk_y][1] - list1_mv[blk_x2][blk_y2][1]) >= mvlimit) |
                      (abs( list1_mv[blk_x][blk_y][0] - list0_mv[blk_x2][blk_y2][0]) >= 4) |
                      (abs( list1_mv[blk_x][blk_y][1] - list0_mv[blk_x2][blk_y2][1]) >= mvlimit);
                  } 
                }
                else  //这就是4个帧号都相等的情况
                { //L0 and L1 reference pictures of p0 are the same; q0 as well
                  //如果4个帧号都相等，只有当上两种情况都有大于一个整像素的情况，Strength才赋为1
                  Strength[idx] =  ((abs( list0_mv[blk_x][blk_y][0] - list0_mv[blk_x2][blk_y2][0]) >= 4) |
                    (abs( list0_mv[blk_x][blk_y][1] - list0_mv[blk_x2][blk_y2][1]) >= mvlimit ) |
                    (abs( list1_mv[blk_x][blk_y][0] - list1_mv[blk_x2][blk_y2][0]) >= 4) |
                    (abs( list1_mv[blk_x][blk_y][1] - list1_mv[blk_x2][blk_y2][1]) >= mvlimit))
                    &&
                    ((abs( list0_mv[blk_x][blk_y][0] - list1_mv[blk_x2][blk_y2][0]) >= 4) |
                    (abs( list0_mv[blk_x][blk_y][1] - list1_mv[blk_x2][blk_y2][1]) >= mvlimit) |
                    (abs( list1_mv[blk_x][blk_y][0] - list0_mv[blk_x2][blk_y2][0]) >= 4) |
                    (abs( list1_mv[blk_x][blk_y][1] - list0_mv[blk_x2][blk_y2][1]) >= mvlimit));
                }       
              }
              else  //如果不是 "P、Q前后向参考帧对应相等或交叉相等" 的情况
              {
                Strength[idx] = 1;        
              } 
            }
            else   //P帧的情况
            { // P slice
              int64 ref_p0,ref_q0;      
              //ref_p0和ref_q0是Q和P块的参考帧号
              ref_p0 = list0_refIdxArr[blk_x][blk_y]<0 ? -1 : list0_refPicIdArr[blk_x][blk_y];
              ref_q0 = list0_refIdxArr[blk_x2][blk_y2]<0 ? -1 : list0_refPicIdArr[blk_x2][blk_y2];
              //下面几个条件满足一个则bS为1，否则为0，
              // 1、如果两块的参考帧不相等
              // 2、两块x轴方向的mv之差的绝对值大于4，即大于一个整像素
              // 3、两块y轴方向的mv之差的绝对值大于mvlimit(mvlimit在帧编码时为4，场编码时为2)，即大于一个整像素
              Strength[idx] =  (ref_p0 != ref_q0 ) |
                (abs( list0_mv[blk_x][blk_y][0] - list0_mv[blk_x2][blk_y2][0]) >= 4 ) |
                (abs( list0_mv[blk_x][blk_y][1] - list0_mv[blk_x2][blk_y2][1]) >= mvlimit );
            }
          }
        }
      }
    }
  }
}

//active_pps->chroma_qp_index_offset即配置文件中的ChromaQPOffset一项
//作用:可通过配置调节色度的QP范围进而影响图像质量
//色度量化参数=亮度量化参数+chroma_qp_index_offset
#define CQPOF(qp) (Clip3(0, 51, qp + active_pps->chroma_qp_index_offset))

/*!
 *****************************************************************************************
 * \brief
 *    Filters one edge of 16 (luma) or 8 (chroma) pel
 *    滤波函数
 *****************************************************************************************
 */
void EdgeLoop(byte** Img, byte Strength[16],ImageParameters *img, int MbQAddr, int AlphaC0Offset, int BetaOffset,
              int dir, int edge, int width, int yuv)
{
  int      pel, ap = 0, aq = 0, Strng ;
  int      incP, incQ;
  int      C0, c0, Delta, dif, AbsDelta ;
  int      L2 = 0, L1, L0, R0, R1, R2 = 0, RL0, L3, R3 ;
  int      Alpha = 0, Beta = 0 ;
  byte*    ClipTab = NULL;   
  int      small_gap;
  int      indexA, indexB;
  int      PelNum;
  int      StrengthIdx;
  byte     *SrcPtrP, *SrcPtrQ;
  int      QP;
  int      xP, xQ, yP, yQ;
  Macroblock *MbQ, *MbP;
  PixelPos pixP, pixQ;
  
  PelNum = yuv ? 8 : 16 ;//参数yuv代表是亮度(0)还是色度(1),亮色度的宽度不同
  //dir取0,1时，分别滤波垂直和水平方向边界
  for( pel=0 ; pel<PelNum ; pel++ )
  {
    xQ = dir ? pel : edge << 2; //滤波边界上像素的坐标，这是相对坐标，整像素单位
    yQ = dir ? (edge < 4 ? edge << 2 : 1) : pel;
     //检查相邻像素可用性，分两种情况:
    //dir为0时,因为是垂直滤波所以检查左边相邻像素的可用性
    //dir为1时,因为是水平滤波所以检查上边相邻像素的可用性，得到的坐标存入pixP
    getNeighbour(MbQAddr, xQ, yQ, 1-yuv, &pixQ);
    getNeighbour(MbQAddr, xQ - (1 - dir), yQ - dir, 1-yuv, &pixP);
    xP = pixP.x;
    yP = pixP.y;
    MbQ = &(img->mb_data[MbQAddr]);      //当前宏块地址得到当前宏块信息
    MbP = &(img->mb_data[pixP.mb_addr]);//根据P点所属宏块地址得到P点所属宏块信息
    fieldModeFilteringFlag = MbQ->mb_field || MbP->mb_field;
    //亮度的StrengthIdx就是pel序号(0-15)，色度的是直接从对应的亮度得到
    StrengthIdx = yuv ? ((MbQ->mb_field && !MbP->mb_field) ? pel<<1 : ((pel>>1)<<2)+(pel%2)) : pel ;

    if (pixP.available || (MbQ->LFDisableIdc== 0)) {
      /*
      incQ和incP是做滤波的象素点之间的间距,
      做垂直滤波时(dir-0)，用到的象素是水平排列的，所以间距为1，
      做水平滤波时(dir-1)，用到的象素是垂直排列的，所以间距为width，
      帧场混合模式时，可能为两倍width
    */
      incQ = dir ? ((fieldModeFilteringFlag && !MbQ->mb_field) ? 2 * width : width) : 1;
      incP = dir ? ((fieldModeFilteringFlag && !MbP->mb_field) ? 2 * width : width) : 1;
      //SrcPtrQ--q0象素点,SrcPtrP--p0象素点
      SrcPtrQ = &(Img[pixQ.pos_y][pixQ.pos_x]);
      SrcPtrP = &(Img[pixP.pos_y][pixP.pos_x]);

      // Average QP of the two blocks
      // QP取相邻两块的平均值,QP_SCALE_CR是色度QP
      QP  = yuv ? (QP_SCALE_CR[CQPOF(MbP->qp)] + QP_SCALE_CR[CQPOF(MbQ->qp)] + 1) >> 1 : (MbP->qp + MbQ->qp + 1) >> 1;

      //AlphaC0Offset即配置文件中的LoopFilterAlphaC0Offset，可以用来调节滤波阈值
      //同理,BetaOffset即配置文件中的LoopFilterBetaOffset，也是用来调节滤波阈值的
      indexA = IClip(0, MAX_QP, QP + AlphaC0Offset); //chj 计算indexA/B
      indexB = IClip(0, MAX_QP, QP + BetaOffset);
    
      Alpha=ALPHA_TABLE[indexA];  //滤波阈值alpha
      Beta=BETA_TABLE[indexB];       //滤波阈值beta
      ClipTab=CLIP_TAB[indexA];  //clip 范围表

      L0  = SrcPtrP[0] ;                       //即p0
      R0  = SrcPtrQ[0] ;                      //q0
      L1  = SrcPtrP[-incP] ;                //p1
      R1  = SrcPtrQ[ incQ] ;                //q1
      L2  = SrcPtrP[-incP*2] ;            //p2
      R2  = SrcPtrQ[ incQ*2] ;            //q2
      L3  = SrcPtrP[-incP*3] ;            //p3
      R3  = SrcPtrQ[ incQ*3] ;            //q3
      if( (Strng = Strength[StrengthIdx]) )//Strength[StrengthIdx]即bS，不为0即进行滤波
      {
        AbsDelta  = abs( Delta = R0 - L0 )  ; //Delta赋值，后面要用
      
        if( AbsDelta < Alpha ) //判断真假边界的第一个条件
        {
          C0  = ClipTab[ Strng ] ; //clip 范围
          if( ((abs( R0 - R1) - Beta )  & (abs(L0 - L1) - Beta )) < 0  ) //第二第三个条件
          {
            if( !yuv) //只对亮度，用这个条件选择不同的滤波器
            {
              //当两个条件都成立的时候，说明边界变化强度不大，
              //滤波强度的设定值相对于实际滤波来说偏大
              aq  = (abs( R0 - R2) - Beta ) < 0  ;
              ap  = (abs( L0 - L2) - Beta ) < 0  ;
            }
          
            RL0  = L0 + R0 ;//因为后面要多次用到这个值，所以先算出来以节省运算量

            if(Strng == 4 )    // INTRA strong filtering  bS为4的情况
            {
              if( yuv)  // Chroma
              {
                SrcPtrQ[0] = ((R1 << 1) + R0 + L1 + 2) >> 2; 
                SrcPtrP[0] = ((L1 << 1) + L0 + R1 + 2) >> 2;                                           
              }
              else  // Luma
              {                
                small_gap = (AbsDelta < ((Alpha >> 2) + 2));
              
                aq &= small_gap;
                ap &= small_gap;
                /*
              根据情况选择滤波器，L=P R=Q
              p0’ = (p2 + 2p1 + 2p0 + 2q0 + q1 + 4) >> 3  
              q0’ = (q2 + 2q1 + 2p0 + 2q0 + q1 + 4) >> 3  
              .... 下面没什么好说的，都是公式的实现
         */
                SrcPtrQ[0]   = aq ? ( L1 + ((R1 + RL0) << 1) +  R2 + 4) >> 3 : ((R1 << 1) + R0 + L1 + 2) >> 2 ;
                SrcPtrP[0]   = ap ? ( R1 + ((L1 + RL0) << 1) +  L2 + 4) >> 3 : ((L1 << 1) + L0 + R1 + 2) >> 2 ;
              
                SrcPtrQ[ incQ] =   aq  ? ( R2 + R0 + R1 + L0 + 2) >> 2 : R1;
                SrcPtrP[-incP] =   ap  ? ( L2 + L1 + L0 + R0 + 2) >> 2 : L1;
              
                SrcPtrQ[ incQ*2] = aq ? (((R3 + R2) <<1) + R2 + R1 + RL0 + 4) >> 3 : R2;
                SrcPtrP[-incP*2] = ap ? (((L3 + L2) <<1) + L2 + L1 + RL0 + 4) >> 3 : L2;
              }
            }
            else   // normal filtering， bS为1，2，3的时候
            {
              c0               = yuv? (C0+1):(C0 + ap + aq) ;
              dif              = IClip( -c0, c0, ( (Delta << 2) + (L1 - R1) + 4) >> 3 ) ;
              SrcPtrP[0]  = IClip(0, 255, L0 + dif) ;
              SrcPtrQ[0]  = IClip(0, 255, R0 - dif) ;
            
              if( !yuv )//只对亮度
              {
                if( ap )
                  SrcPtrP[-incP] += IClip( -C0,  C0, ( L2 + ((RL0 + 1) >> 1) - (L1<<1)) >> 1 ) ;
                if( aq  )
                  SrcPtrQ[ incQ] += IClip( -C0,  C0, ( R2 + ((RL0 + 1) >> 1) - (R1<<1)) >> 1 ) ;
              } 
            } 
          } ; 
        } ;
      } ;
    } ;
  }
}

