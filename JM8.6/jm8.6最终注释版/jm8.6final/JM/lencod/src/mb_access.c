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
 * \file mb_access.c
 *
 * \brief
 *    Functions for macroblock neighborhoods
 *已读
 *  \author
 *      Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Karsten S黨ring          <suehring@hhi.de>
 *************************************************************************************
 */

#include "global.h"

/*!
 ************************************************************************
 * \brief
 *    returns 1 if the macroblock at the given address is available
 *    检验宏块mbAddr是否可用
 ************************************************************************
 */
int mb_is_available(int mbAddr, int currMbAddr)
{
  if ((mbAddr < 0) || (mbAddr > ((int)img->PicSizeInMbs - 1)))
    return 0;

  // the following line checks both: slice number and if the mb has been decoded
  //判断当前帧是否正在deblocking
  if (!img->DeblockCall)
  { //判断mbAddr跟currMbAddr是否属于一个slice
    if (img->mb_data[mbAddr].slice_nr != img->mb_data[currMbAddr].slice_nr)
      return 0;
  }  
  return 1;
}


/*!
 ************************************************************************
 * \brief
 *    Checks the availability of neighboring macroblocks of
 *    the current macroblock for prediction and context determination;
 *    该函数在start_macroblock和deblockmb中调用，在每个encode_one_macroblock或者deblock执行之前，
 *    都要检查周围宏块的可用性(A,B,C,D)
 ************************************************************************
 */
 /*
 *     MBaddrD    MBaddrB    MBaddrC
 *
 *
 *     MBaddrA    MBaddrCur
 *
*/
void CheckAvailabilityOfNeighbors()
{
  const int mb_nr = img->current_mb_nr;
  Macroblock *currMB = &img->mb_data[mb_nr];

  // mark all neighbors as unavailable
  currMB->mb_available_up   = NULL;
  currMB->mb_available_left = NULL;

  if (img->MbaffFrameFlag)
  {
    currMB->mbAddrA = 2 * (mb_nr/2 - 1);//左边宏块对的top宏块
    currMB->mbAddrB = 2 * (mb_nr/2 - img->PicWidthInMbs);//上边宏块对的top宏块
    currMB->mbAddrC = 2 * (mb_nr/2 - img->PicWidthInMbs + 1); //右上宏块对的top宏块
    currMB->mbAddrD = 2 * (mb_nr/2 - img->PicWidthInMbs - 1);//左上宏块对的top宏块
    //mbAddrA是否可用且当前块不是最左边一列
    currMB->mbAvailA = mb_is_available(currMB->mbAddrA, mb_nr) && (((mb_nr/2) % img->PicWidthInMbs)!=0);
    //mbAddrB是否可用
    currMB->mbAvailB = mb_is_available(currMB->mbAddrB, mb_nr);
    //mbAddrC是否可用且当前块不是最右边一列
    currMB->mbAvailC = mb_is_available(currMB->mbAddrC, mb_nr) && (((mb_nr/2 +1) % img->PicWidthInMbs)!=0);
    //mbAddrD是否可用且当前块不是最左边一列
    currMB->mbAvailD = mb_is_available(currMB->mbAddrD, mb_nr) && (((mb_nr/2) % img->PicWidthInMbs)!=0);
  }
  else
  {
    currMB->mbAddrA = mb_nr - 1; //左边宏块
    currMB->mbAddrB = mb_nr - img->PicWidthInMbs; //上边宏块
    currMB->mbAddrC = mb_nr - img->PicWidthInMbs + 1; //右上宏块
    currMB->mbAddrD = mb_nr - img->PicWidthInMbs - 1; //左上宏块
    //mbAddrA是否可用且当前块不是最左边一列
    currMB->mbAvailA = mb_is_available(currMB->mbAddrA, mb_nr) && ((mb_nr % img->PicWidthInMbs)!=0);
    //mbAddrB是否可用
    currMB->mbAvailB = mb_is_available(currMB->mbAddrB, mb_nr);
    //mbAddrC是否可用且当前块不是最右边一列
    currMB->mbAvailC = mb_is_available(currMB->mbAddrC, mb_nr) && (((mb_nr+1) % img->PicWidthInMbs)!=0);
    //mbAddrD是否可用且当前块不是最左边一列
    currMB->mbAvailD = mb_is_available(currMB->mbAddrD, mb_nr) && ((mb_nr % img->PicWidthInMbs)!=0);
  }

  if (currMB->mbAvailA) currMB->mb_available_left = &(img->mb_data[currMB->mbAddrA]);
  if (currMB->mbAvailB) currMB->mb_available_up   = &(img->mb_data[currMB->mbAddrB]);
}


/*!
 ************************************************************************
 * \brief
 *    returns the x and y macroblock coordinates for a given MbAddress
 *    得到地址为mb_addr的宏块的宏块坐标
 ************************************************************************
 */
void get_mb_block_pos (int mb_addr, int *x, int*y)
{
  //PicWidthInMbs 是以宏块为单位的图像宽度
  if (img->MbaffFrameFlag) 
  {
    *x = ((mb_addr/2) % img->PicWidthInMbs); //得到以MB为单位的位置MBAFF
    *y = ( ((mb_addr/2) / img->PicWidthInMbs)  * 2 + (mb_addr%2));
  }
  else 
  {
    *x = (mb_addr % img->PicWidthInMbs);//得到以MB为单位的位置non-MBAFF
    *y = (mb_addr / img->PicWidthInMbs);
  }
}

/*!
 ************************************************************************
 * \brief
 *    returns the x and y sample coordinates for a given MbAddress
 *    得到地址为mb_addr的宏块的绝对地址(采样坐标)
 ************************************************************************
 */
void get_mb_pos (int mb_addr, int *x, int*y)
{
  get_mb_block_pos(mb_addr, x, y);
  
  (*x) *= MB_BLOCK_SIZE;//得到宏块号为mb_addr的宏块的绝对地址
  (*y) *= MB_BLOCK_SIZE;
}

/*!
 ************************************************************************
 * \brief
 * 得到Non-MBAFF时相邻像素点的绝对坐标和在其所属宏块内的相对坐标
 *    get neighbouring positions for non-aff coding
 * \param curr_mb_nr
 *   current macroblock number (decoding order)
 * \param xN
 *    input x position
 * \param yN
 *    input y position
 * \param luma
 *    1 if luma coding, 0 for chroma
 * \param pix
 *    returns position informations
 ************************************************************************
 */
void getNonAffNeighbour(unsigned int curr_mb_nr, int xN, int yN, int luma, PixelPos *pix)
{
  Macroblock *currMb = &img->mb_data[curr_mb_nr];
  int maxWH;

  if (luma)
    maxWH = 16;
  else
    maxWH = 8;

  //mbAddrX和mbAvailX都是在CheckAvailabilityOfNeighbors计算得到的
  //计算坐标(xN,yN)处像素点的可用性
  if ((xN<0)&&(yN<0))
  {
    pix->mb_addr   = currMb->mbAddrD;
    pix->available = currMb->mbAvailD;
  }
  else
  if ((xN<0)&&((yN>=0)&&(yN<maxWH)))
  {
    pix->mb_addr  = currMb->mbAddrA;
    pix->available = currMb->mbAvailA;
  }
  else
  if (((xN>=0)&&(xN<maxWH))&&(yN<0))
  {
    pix->mb_addr  = currMb->mbAddrB;
    pix->available = currMb->mbAvailB;
  }
  else
  if (((xN>=0)&&(xN<maxWH))&&((yN>=0)&&(yN<maxWH)))
  {
    pix->mb_addr  = curr_mb_nr;
    pix->available = 1;
  }
  else
  if ((xN>=maxWH)&&(yN<0))
  {
    pix->mb_addr  = currMb->mbAddrC;
    pix->available = currMb->mbAvailC;
  }
  else 
  {
    pix->available = 0;
  }
  //如果相邻块可用或者当前图像正在做deblocking
  if (pix->available || img->DeblockCall)
  {
    pix->x = (xN + maxWH) % maxWH;   //得到在其所在宏块内的相对坐标
    pix->y = (yN + maxWH) % maxWH; 
    get_mb_pos(pix->mb_addr, &(pix->pos_x), &(pix->pos_y)); //得到相邻块所属宏块的宏块坐标
    if (luma)
    {
      pix->pos_x += pix->x; //得到亮度相邻像素绝对坐标
      pix->pos_y += pix->y;
    }
    else
    {
      pix->pos_x = (pix->pos_x/2) + pix->x;  //得到色度相邻像素绝对坐标
      pix->pos_y = (pix->pos_y/2) + pix->y;
    }
  }
}

/*!
 ************************************************************************
 * \brief
 * 得到MBAFF时相邻像素点的绝对坐标和在其所属宏块内的相对坐标
 * 阅读前须知:
 * 1、宏块对中两个宏块的可用性是一致的 
 *    get neighbouring positions for aff coding
 * \param curr_mb_nr
 *   current macroblock number (decoding order)
 * \param xN
 *    input x position
 * \param yN
 *    input y position
 * \param luma
 *    1 if luma coding, 0 for chroma
 * \param pix
 *    returns position informations
 ************************************************************************
 */
void getAffNeighbour(unsigned int curr_mb_nr, int xN, int yN, int luma, PixelPos *pix)
{
  Macroblock *currMb = &img->mb_data[curr_mb_nr];
  int maxWH;
  int yM = -1;   //MbAff时变化就在y坐标上，x坐标没什么花样

  if (luma)
    maxWH = 16;   //亮度宏块边长
  else
    maxWH = 8;     //色度宏块边长

  // initialize to "not available"
  pix->available = 0;

  if(yN > (maxWH - 1))  //当前宏块下方
  {
    return;
  }
  if ((xN > (maxWH -1)) && ((yN >= 0)&&(yN < (maxWH )))) //当前宏块右边
  {
    return;
  }

  if (xN < 0)  //左边
  {
    if (yN < 0)  //左上，yN<0意味着yN等于-1
    {
      if(!currMb->mb_field)  //当前宏块是帧宏块，当前宏块对是帧宏块对
      {
        // frame
        if (curr_mb_nr%2 == 0)  //当前宏块是top宏块
        {
          // top
          pix->mb_addr  = currMb->mbAddrD  + 1;  //取mbAddrD的底宏块作为相邻宏块
          pix->available = currMb->mbAvailD;
           yM      = yN;  //不管mbAddrD是帧/场宏块对，都保持yN
        }
        else    //当前宏块是bottom宏块
        {
          // bottom
          pix->mb_addr  = currMb->mbAddrA;   //取mbAddrA的顶宏块作为相邻宏块
          pix->available = currMb->mbAvailA; 
          if (currMb->mbAvailA)   //如果mbAddrA宏块可用
          {
            if(!img->mb_data[currMb->mbAddrA].mb_field) //且不为场宏块
            {
               yM = yN;   //则临像素坐标仍取yN不变
            }
            else   //而若临块为场宏块(当前块为帧宏块)
            {
              (pix->mb_addr)++;   //取mbAddrA的底场宏块作为相邻宏块!
               yM = (yN + maxWH) >> 1; //(-1+16)/2=7,正好是mbAddrA上半部最下面的场宏块行
            }
          }
        }
      }
      else   //当前宏块是场宏块，当前宏块对是场宏块对
      {
        // field
        if(curr_mb_nr % 2 == 0)  //当前宏块是顶场宏块
        {
          // top
          pix->mb_addr  = currMb->mbAddrD; //取mbAddrD的top宏块作为相邻宏块
          pix->available = currMb->mbAvailD;
          if (currMb->mbAvailD)     //如果mbAddrD宏块可用
          {
            if(!img->mb_data[currMb->mbAddrD].mb_field) //mbAddrD为帧宏块对
            {
              (pix->mb_addr)++;  //取底帧宏块作为相邻宏块
               yM = 2 * yN;             //相邻像素纵坐标yM为底帧宏块倒数第二行
            }
            else   //mbAddrD为场宏块对
            {
               yM = yN;   //仍取顶场宏块作为相邻宏块，yM取顶场宏块最后一行
            }   //不管是底帧宏块倒数第二行还是顶场宏块最后一行都是宏块对倒数第二行
          }
        }
        else   //当前宏块是底场宏块
        {
          // bottom
          pix->mb_addr  = currMb->mbAddrD+1;  //取bottom宏块作为相邻宏块,不管帧/场
          pix->available = currMb->mbAvailD;
           yM      = yN; //不管帧/场，yM都取宏块对最后一行
        }
      }
    }
    else
    { // xN < 0 && yN >= 0
      if ((yN >= 0) && (yN <maxWH))  //左边
      {
        if (!currMb->mb_field)   //当前块是帧宏块
        {
          // frame
          if(curr_mb_nr % 2 == 0)  //top宏块
          {
            // top
            pix->mb_addr  = currMb->mbAddrA; //取mbAddrA的top宏块作为相邻宏块
            pix->available = currMb->mbAvailA;
            if (currMb->mbAvailA)  //可用
            {
              if(!img->mb_data[currMb->mbAddrA].mb_field) //mbAddrA是帧宏块(对)
              {
                 yM = yN;
              }
              else    //mbAddrA是场宏块(对)
              {
                if (yN %2 == 0)  //yN属于顶场
                {
                   yM = yN>> 1;
                }
                else
                {
                  (pix->mb_addr)++;  //yN属于底场,则相邻像素yM取底场宏块
                   yM = yN>> 1;
                }
              }
            }
          }
          else   //bottom宏块
          {
            // bottom
            pix->mb_addr  = currMb->mbAddrA;
            pix->available = currMb->mbAvailA;
            if (currMb->mbAvailA)
            {
              if(!img->mb_data[currMb->mbAddrA].mb_field) //mbAddrA是帧宏块(对)
              {
                (pix->mb_addr)++;  //取底帧宏块
                 yM = yN;
              }
              else     //mbAddrA是场宏块(对)
              {
                if (yN %2 == 0)  //yN属于顶场
                {
                   yM = (yN + maxWH) >> 1;
                }
                else
                {
                  (pix->mb_addr)++;          //yN属于底场,则相邻像素yM取底场宏块
                   yM = (yN + maxWH) >> 1;
                }
              }
            }
          }
        }
        else   //当前块是场宏块
        {
          // field
          if (curr_mb_nr % 2 == 0)  //当前块是顶场宏块
          {
            // top
            pix->mb_addr  = currMb->mbAddrA;   //先赋值为顶帧(场)宏块
            pix->available = currMb->mbAvailA;
            if (currMb->mbAvailA)
            {
              if(!img->mb_data[currMb->mbAddrA].mb_field)   //mbAddrA是帧宏块(对)
              {
                if (yN < (maxWH / 2))  //yN属于顶场宏块的上半部分
                {
                   yM = yN << 1;  //则相临像素yM属于mbAddrA的顶帧宏块
                }
                else            //yN属于顶场宏块的下半部分
                {
                  (pix->mb_addr)++;            //则相临像素yM属于mbAddrA的底帧宏块
                   yM = (yN << 1 ) - maxWH;  //计算yM在所属宏块的相对位置
                }
              }
              else     //mbAddrA是场宏块(对)
              {
                 yM = yN;  //顶场对顶场，最简单的情况
              }
            }
          }
          else    //当前块是底场宏块
          {
            // bottom
            pix->mb_addr  = currMb->mbAddrA;  
            pix->available = currMb->mbAvailA;
            if (currMb->mbAvailA)
            {
              if(!img->mb_data[currMb->mbAddrA].mb_field)  //mbAddrA是帧宏块(对)
              {
                if (yN < (maxWH / 2)) //yN属于底场宏块的上半部分
                {
                   yM = (yN << 1) + 1;   //跟上面类似，因为是底场所以加1
                }
                else   //与上面过程类似
                {
                  (pix->mb_addr)++;
                   yM = (yN << 1 ) + 1 - maxWH; 
                }
              }
              else                //mbAddrA是场宏块(对)
              {
                 (pix->mb_addr)++;   //把相邻像素所属块改为底场
                 yM = yN;      //底场对底场，最简单情况
              }
            }
          }
        }
      }
    }
  }
  else
  {
    // xN >= 0
    if ((xN >= 0)&&(xN <maxWH))  
    {
      if (yN<0)   //上方
      {
        if (!currMb->mb_field)  //当前块是帧宏块
        {
          //frame
          if (curr_mb_nr % 2 == 0)   //当前块是顶帧宏块
          {
            //top
            pix->mb_addr  = currMb->mbAddrB;
            // for the deblocker if the current MB is a frame and the one above is a field
            // then the neighbor is the top MB of the pair
            if (currMb->mbAvailB)
            {
              if (!(img->DeblockCall == 1 && (img->mb_data[currMb->mbAddrB]).mb_field))
                pix->mb_addr  += 1; //如果不是(正在做Deblocking而且mbAddrB是场宏块)，邻块为底宏块
            }

            pix->available = currMb->mbAvailB;
             yM      = yN;
          }
          else   //当前块是底帧宏块
          {
            // bottom
            pix->mb_addr  = curr_mb_nr - 1;  //则邻块是顶帧宏块
            pix->available = 1; //必然可用，因为与当前块属于一个宏块对的，可用性一致
             yM      = yN;
          }
        }
        else    //当前块是场宏块
        {
          // field
          if (curr_mb_nr % 2 == 0)  //顶场
          {
            // top
            pix->mb_addr  = currMb->mbAddrB;
            pix->available = currMb->mbAvailB;
            if (currMb->mbAvailB)
            {
              if(!img->mb_data[currMb->mbAddrB].mb_field) //mbAddrB是帧宏块(对)
              {
                (pix->mb_addr)++;   //取mbAddrB宏块对的底帧宏块
                 yM = 2* yN;   //##  相当于把mbAddrB先转成场宏块再取顶场对应位置
              }
              else
              {
                 yM = yN;  //和上面标注##处语句所指示的位置其实是一样的!!
              }
            }
          }
          else      //底场
          {
            // bottom
            pix->mb_addr  = currMb->mbAddrB + 1;   //不管mbAddrB是帧or场都取底宏块
            pix->available = currMb->mbAvailB;
             yM      = yN; //相当于把mbAddrB先转成场宏块再取底场对应位置
          }
        }
      }
      else
      {
        // yN >=0
        // for the deblocker if this is the extra edge then do this special stuff
        //DeblockCall==2的四个条件:
        // 1、是水平边界，2、非图像边界，3、MbQ是场宏块编码，4、临块是帧宏块编码
        if (yN == 0 && img->DeblockCall == 2)
        {
          pix->mb_addr  = currMb->mbAddrB + 1;  //取mbAddrB的底帧宏块
          pix->available = 1;
           yM      = yN - 1;
        }

        else if ((yN >= 0) && (yN <maxWH))
        {
          pix->mb_addr  = curr_mb_nr;
          pix->available = 1;
           yM      = yN;
        }
      }
    }
    else
    {
      // xN >= maxWH
      if(yN < 0)
      {
        if (!currMb->mb_field)
        {
          // frame
          if (curr_mb_nr % 2 == 0)
          {
            // top
            pix->mb_addr  = currMb->mbAddrC + 1;
            pix->available = currMb->mbAvailC;
             yM      = yN;
          }
          else
          {
            // bottom
            pix->available = 0;
          }
        }
        else
        {
          // field
          if (curr_mb_nr % 2 == 0)
          {
            // top
            pix->mb_addr  = currMb->mbAddrC;
            pix->available = currMb->mbAvailC;
            if (currMb->mbAvailC)
            {
              if(!img->mb_data[currMb->mbAddrC].mb_field)
              {
                (pix->mb_addr)++;
                 yM = 2* yN;
              }
              else
              {
                 yM = yN;
              }
            }
          }
          else
          {
            // bottom
            pix->mb_addr  = currMb->mbAddrC + 1;
            pix->available = currMb->mbAvailC;
             yM      = yN;
          }
        }
      }
    }
  }
  if (pix->available || img->DeblockCall)  //该过程跟NonAff中的一样,见NonAff函数
  {
    pix->x = (xN + maxWH) % maxWH;
    pix->y = (yM + maxWH) % maxWH;
    get_mb_pos(pix->mb_addr, &(pix->pos_x), &(pix->pos_y));
    if (luma)
    {
      pix->pos_x += pix->x;
      pix->pos_y += pix->y;
    }
    else
    {
      pix->pos_x = (pix->pos_x/2) + pix->x;
      pix->pos_y = (pix->pos_y/2) + pix->y;
    }
  }
}


/*!
 ************************************************************************
 * \brief
 *    get neighbouring positions. MB AFF is automatically used from img structure
 *    得到相邻像素点的绝对坐标和在其所属宏块内的相对坐标（AFF和NonAFF两种情况）
 * \param curr_mb_nr
 *   current macroblock number (decoding order)
 * \param xN
 *    input x position
 * \param yN
 *    input y position
 * \param luma
 *    1 if luma coding, 0 for chroma
 * \param pix
 *    returns position informations
 ************************************************************************
 */
void getNeighbour(int curr_mb_nr, int xN, int yN, int luma, PixelPos *pix)
{
  if (curr_mb_nr<0) ;
    //error ("getNeighbour: invalid macroblock number", 100);

  if (img->MbaffFrameFlag)
    getAffNeighbour(curr_mb_nr, xN, yN, luma, pix);  //跟下面的作用是一样的，区别是AFF
  else
    getNonAffNeighbour(curr_mb_nr, xN, yN, luma, pix); //得到相邻像素绝对坐标和宏块内坐标存入pix
}


/*!
 ************************************************************************
 * \brief
 *    get neighbouring  get neighbouring 4x4 luma block
 * \param curr_mb_nr
 *   current macroblock number (decoding order)
 * \param block_x
 *    input x block position
 * \param block_y
 *    input y block position
 * \param rel_x
 *    relative x position of neighbor
 * \param rel_y
 *    relative y position of neighbor
 * \param pix
 *    returns position informations
 ************************************************************************
 */
void getLuma4x4Neighbour (int curr_mb_nr, int block_x, 
                          int block_y, int rel_x, int rel_y, PixelPos *pix)
{
  int x = 4* block_x + rel_x;  //block_x, block_y是MB内相对4x4块坐标
  int y = 4* block_y + rel_y;  //这两句得到相邻像素点相对当前宏块原点的坐标

  getNeighbour(curr_mb_nr, x, y, 1, pix);//检测相邻像素点所属宏块的可用性

  if (pix->available)
  {
    pix->x /= 4; //pix->x pix->y是相邻像素宏块内坐标
    pix->y /= 4; //这两句得到相邻像素所属的4x4小块的宏块内相对坐标
    pix->pos_x /= 4; //pix->pos_x pix->pos_y是相邻像素的绝对坐标
    pix->pos_y /= 4; //所以，这两句得到相邻像素所属的4x4小块的绝对坐标
  }
}

/*!
 ************************************************************************
 * \brief
 *    get neighbouring 4x4 chroma block
 * \param curr_mb_nr
 *   current macroblock number (decoding order)
 * \param block_x
 *    input x block position
 * \param block_y
 *    input y block position
 * \param rel_x
 *    relative x position of neighbor
 * \param rel_y
 *    relative y position of neighbor
 * \param pix
 *    returns position informations
 ************************************************************************
 */
void getChroma4x4Neighbour (int curr_mb_nr, int block_x, int block_y, int rel_x, int rel_y, PixelPos *pix)
{
  int x = 4* block_x + rel_x;
  int y = 4* block_y + rel_y;

  getNeighbour(curr_mb_nr, x, y, 0, pix);

  if (pix->available)
  {
    pix->x /= 4;
    pix->y /= 4;
    pix->pos_x /= 4;
    pix->pos_y /= 4;
  }
}
