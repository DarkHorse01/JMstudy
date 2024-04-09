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
 * \file mv-search.c
 *
 * \brief
 *    Motion Vector Search, unified for B and P Pictures
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 *      - Inge Lille-Lang鴜               <inge.lille-langoy@telenor.com>
 *      - Rickard Sjoberg                 <rickard.sjoberg@era.ericsson.se>
 *      - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 *      - Jani Lainema                    <jani.lainema@nokia.com>
 *      - Detlev Marpe                    <marpe@hhi.de>
 *      - Thomas Wedi                     <wedi@tnt.uni-hannover.de>
 *      - Heiko Schwarz                   <hschwarz@hhi.de>
 *
 *************************************************************************************
*/

#include "contributors.h"

#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <limits.h>

#include "global.h"
#include "image.h"
#include "mv-search.h"
#include "refbuf.h"
#include "memalloc.h"
#include "mb_access.h"
#include "fast_me.h"

#include <time.h>
#include <sys/timeb.h>

// These procedure pointers are used by motion_search() and one_eigthpel()
static pel_t  (*PelY_14)     (pel_t**, int, int, int, int);
static pel_t *(*PelYline_11) (pel_t *, int, int, int, int);

// Statistics, temporary
int     max_mvd;
int*    spiral_search_x;
int*    spiral_search_y;//螺旋搜索
int*    mvbits;
int*    refbits;
int*    byte_abs;
int**** motion_cost;


void SetMotionVectorPredictor (int  pmv[2],
                               int  ***refPic,
                               int  ****tmp_mv,
                               int  ref_frame,
                               int  list,
                               int  block_x,
                               int  block_y,
                               int  blockshape_x, //模型
                               int  blockshape_y);

#ifdef _FAST_FULL_ME_

/*****
 *****  static variables for fast integer motion estimation
 *****
 */
static int  **search_setup_done;  //!< flag if all block SAD's have been calculated yet
static int  **search_center_x;    //!< absolute 绝对 search center for fast full motion search
static int  **search_center_y;    //!< absolute search center for fast full motion search
static int  **pos_00;             //!< position of (0,0) vector
//其中一个坐标是list0/list1
static int  *****BlockSAD;        //!< SAD for all blocksize, ref. frames and motion vectors
//[list0/list1][num_references][7种模式:1-16*16 2-16*8 3-8*16 4-8*8 5-8*4 6-4*8 7-4*4]
//[16个4*4块]
static int  **max_search_range;

extern ColocatedParams *Co_located;

/*!
 ***********************************************************************
 * \brief
 *    function creating arrays for fast integer motion estimation
 ***********************************************************************
 */
void
InitializeFastFullIntegerSearch ()
{
  int  i, j, k, list;
  int  search_range = input->search_range;
  int  max_pos      = (2*search_range+1) * (2*search_range+1);

  if ((BlockSAD = (int*****)malloc (2 * sizeof(int****))) == NULL)
  	/*malloc向系统申请分配指定size个字节的内存空间。返回类型是 int* 类型。*/
    no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");

  for (list=0; list<2;list++)
  {
    if ((BlockSAD[list] = (int****)malloc ((img->max_num_references+1) * sizeof(int***))) == NULL)//多分配一个，下标0不用
      no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
    for (i = 0; i <= img->max_num_references; i++)
    {
      if ((BlockSAD[list][i] = (int***)malloc (8 * sizeof(int**))) == NULL)//7种模式，下标0不用
        no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
      for (j = 1; j < 8; j++)
      {
        if ((BlockSAD[list][i][j] = (int**)malloc (16 * sizeof(int*))) == NULL)//16个4x4小块
          no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
        for (k = 0; k < 16; k++)
        {
          if ((BlockSAD[list][i][j][k] = (int*)malloc (max_pos * sizeof(int))) == NULL)//所有的搜索位置
            no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
        }
      }
    }
  }

  if ((search_setup_done = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_setup_done");
  if ((search_center_x = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_x");
  if ((search_center_y = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_y");
  if ((pos_00 = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: pos_00");
  if ((max_search_range = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: max_search_range");

  for (list=0; list<2; list++)
  {
  if ((search_setup_done[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_setup_done");
  if ((search_center_x[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_x");
  if ((search_center_y[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_y");
  if ((pos_00[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: pos_00");
  if ((max_search_range[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: max_search_range");
  }

  // assign max search ranges for reference frames
  //b 关于full_search 的说明
  /* full_search == 2 代表无搜索范围限制
    full_search == 1 代表是old参考帧,即不是最近的参考帧
    full_search == 0 代表是old参考帧和小于宏块的块*/
  if (input->full_search == 2)
  {
    for (list=0;list<2;list++)
      for (i=0; i<=img->max_num_references; i++)  
        max_search_range[list][i] = search_range;
  }
  else
  {
    for (list=0;list<2;list++)
    {
      //b 只有最近的参考帧的搜索范围是search_range
      //b 其余的是search_range的一半
      max_search_range[list][0] = max_search_range[list][img->max_num_references] = search_range;
      for (i=1; i< img->max_num_references; i++)  max_search_range[list][i] = search_range / 2;
    }
  }

}



/*!
 ***********************************************************************
 * \brief
 *    function for deleting the arrays for fast integer motion estimation
 ***********************************************************************
 */
void
ClearFastFullIntegerSearch ()
{//释放申请的动态内存
  int  i, j, k, list;

  for (list=0; list<2; list++)
  {
    for (i = 0; i <= img->max_num_references; i++)
    {
      for (j = 1; j < 8; j++)
      {
        for (k = 0; k < 16; k++)
        {
          free (BlockSAD[list][i][j][k]);
		  //free(void *p)函数是用来释放程序动态申请的内存。其参数是个指针类型。
        }
        free (BlockSAD[list][i][j]);
      }
      free (BlockSAD[list][i]);
    }
    free (BlockSAD[list]);
  }
  free (BlockSAD);

  for (list=0; list<2; list++)
  {
    free (search_setup_done[list]);
    free (search_center_x[list]);
    free (search_center_y[list]);
    free (pos_00[list]);
    free (max_search_range[list]);
  }
  free (search_setup_done);
  free (search_center_x);
  free (search_center_y);
  free (pos_00);
  free (max_search_range);

}


/*!
 ***********************************************************************
 * \brief
 *    function resetting flags for fast integer motion estimation
 *    (have to be called in start_macroblock())
 ***********************************************************************
 */
void
ResetFastFullIntegerSearch ()
{
  int i,list;

  for (list=0; list<2; list++)
    for (i = 0; i <= img->max_num_references; i++)
      search_setup_done [list][i] = 0;
}

/*!
 ***********************************************************************
 * \brief
 *    calculation of SAD for larger blocks on the basis of 4x4 blocks
 ***********************************************************************
 */
void
SetupLargerBlocks (int list, int refindex, int max_pos)
{//用宏块范围内4*4小块的SAD计算其他分块模式个分块的SAD
#define ADD_UP_BLOCKS()   _o=*_bo; _i=*_bi; _j=*_bj; for(pos=0;pos<max_pos;pos++) _o[pos] = _i[pos] + _j[pos];
#define INCREMENT(inc)    _bo+=inc; _bi+=inc; _bj+=inc;

  int    pos, **_bo, **_bi, **_bj;
  register int *_o,   *_i,   *_j;

  //--- blocktype 6 --- 4x8
  _bo = BlockSAD[list][refindex][6];//用来存放4*8分块模式的SAD
  _bi = BlockSAD[list][refindex][7];//已求出的各4*4小块的SAD
  _bj = _bi + 4;//加4指向下一行4x4小块
  /*每一个ADD_UP_BLOCKS计算一个4x8小块的SAD
    INCREMENT(1),指针后移一个4x4小块
    INCREMENT(5)，指针后移五个4x4小块
  */
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(5);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS();

  //--- blocktype 5 ---8x4
  _bo = BlockSAD[list][refindex][5];//用来存放8*4分块模式的SAD
  _bi = BlockSAD[list][refindex][7];
  _bj = _bi + 1;
  /*参考上面的代码，很容易理解，下面代码类似*/
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS();

  //--- blocktype 4 ---8x8
  _bo = BlockSAD[list][refindex][4];
  _bi = BlockSAD[list][refindex][6];
  _bj = _bi + 1;
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(6);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS();

  //--- blocktype 3 --- 8x16 
  _bo = BlockSAD[list][refindex][3];
  _bi = BlockSAD[list][refindex][4];
  _bj = _bi + 8;
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS();

  //--- blocktype 2 --- 16x8
  _bo = BlockSAD[list][refindex][2];
  _bi = BlockSAD[list][refindex][4];
  _bj = _bi + 2;
  ADD_UP_BLOCKS(); INCREMENT(8);
  ADD_UP_BLOCKS();

  //--- blocktype 1 ---16x16
  _bo = BlockSAD[list][refindex][1];
  _bi = BlockSAD[list][refindex][3];
  _bj = _bi + 2;
  ADD_UP_BLOCKS();
}


/*!
 ***********************************************************************
 * \brief  根据预测出的MV给出的搜索中心，计算搜索范围内的所有宏块组合方式的SAD值
 *    Setup the fast search for an macroblock
 *    计算过程: 首先计算 16 个 4x4 块在搜索范围
 *	    内所有位置的 SAD，然后计算出其他 6 种分割模式各自的 SAD，
 *	    比如4x8，8x4的SAD就是两个4x4块的SAD加起来
 ***********************************************************************
 */
void SetupFastFullPelSearch (int ref, int list)  // <--  reference frame parameter, list0 or 1
{
  int     pmv[2];    //存储预测运动向量单位是1/4像素
  //orig_blocks正好存一个宏块的数据16x16
  pel_t   orig_blocks[256], *orgptr=orig_blocks, *refptr; 
  int     offset_x, offset_y, x, y, range_partly_outside, ref_x, ref_y, pos, abs_x, abs_y, bindex, blky;
  int     LineSadBlk0, LineSadBlk1, LineSadBlk2, LineSadBlk3;  //用来存储一行像素的SAD
  
  int     max_width, max_height;
  int     img_width, img_height;
  
  StorablePicture *ref_picture;//参考帧 
  pel_t   *ref_pic; //参考像素

   //[7]为各种分块模式的运动估计SAD值,时刻牢记7种分块模式
  int**   block_sad     = BlockSAD[list][ref][7]; //全局变量反复用

  /*获得当前块对ref参考帧的搜索半径
   可以有不同搜索半径,前面已经说过，可参考max_search_range的赋值
   在函数InitializeFastFullIntegerSearch中
  */
  int     search_range  = max_search_range[list][ref];
  
  //最大搜索范围
  int     max_pos       = (2*search_range+1) * (2*search_range+1);

  //参考listX赋值的地方,在函数init_lists和init_mbaff_lists
  int     list_offset   = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

  /*是否进行有权重的预测-weighted prediction(WP)，
    weighted_pred_flag和weighted_bipred_idc分别是config里面的
    WeightedPrediction和WeightedBiprediction 
    WP是用来编码fading sequence的，就是渐进渐出的场景，效果非常好
    WP只有H.264的mainline和extented才有，
    可以参考这篇论文:
    <<WEIGHTED PREDICTION IN THE H.264/MPEG AVC VIDEO CODING STANDARD>>by:Jill M. Boyce 
    WeightedPrediction和WeightedBiprediction默认值都为0，所以不使用WP编码
 */
  int     apply_weights = ( (active_pps->weighted_pred_flag && (img->type == P_SLICE || img->type == SP_SLICE)) ||
                            (active_pps->weighted_bipred_idc && (img->type == B_SLICE)));
  
  //获得参考帧
  ref_picture     = listX[list+list_offset][ref];
  
  if (apply_weights)
    ref_pic       = ref_picture->imgY_11_w;
  else
    ref_pic       = ref_picture->imgY_11;

  max_width     = ref_picture->size_x - 17;
  max_height    = ref_picture->size_y - 17;
  
  img_width     = ref_picture->size_x;
  img_height    = ref_picture->size_y;

  //===== get search center: predictor of 16x16 block =====
   //预测MV，1/4象素为单位
  SetMotionVectorPredictor (pmv, enc_picture->ref_idx, enc_picture->mv, ref, list, 0, 0, 16, 16);
  
  //search_center跟pmv的关系
  search_center_x[list][ref] = pmv[0] / 4;
  search_center_y[list][ref] = pmv[1] / 4;//换算成整像素

  if (!input->rdopt)
  {
    //--- correct center so that (0,0) vector is inside ---
    //避免越界
    search_center_x[list][ref] = max(-search_range, min(search_range, search_center_x[list][ref]));
    search_center_y[list][ref] = max(-search_range, min(search_range, search_center_y[list][ref]));
  }

  //预测mv加原始坐标得到search_center的绝对坐标(对整幅图像左上角(0,0)位置) 
  search_center_x[list][ref] += img->opix_x;
  search_center_y[list][ref] += img->opix_y;

  offset_x = search_center_x[list][ref];
  offset_y = search_center_y[list][ref];

  //===== copy original block for fast access =====
  //复制原始宏块，并转换为一维的形式
  for   (y = img->opix_y; y < img->opix_y+16; y++)
    for (x = img->opix_x; x < img->opix_x+16; x++)
      *orgptr++ = imgY_org [y][x];//一维比较快


  //===== check if whole search range is inside image =====
  //检查是否越界，
  //例如:第1个宏块搜索时左边就会越界 
  if (offset_x >= search_range && offset_x <= max_width  - search_range &&
      offset_y >= search_range && offset_y <= max_height - search_range   )
  {
    range_partly_outside = 0; PelYline_11 = FastLine16Y_11;//未越界
  }
  else
  {
    range_partly_outside = 1;//越界
  } 

  //===== determine position of (0,0)-vector =====
  if (!input->rdopt)
  {
    ref_x = img->opix_x - offset_x;
    ref_y = img->opix_y - offset_y;//相对坐标

    for (pos = 0; pos < max_pos; pos++)
    {
      if (ref_x == spiral_search_x[pos] &&//螺旋搜索
          ref_y == spiral_search_y[pos])
      {
        pos_00[list][ref] = pos;//确定当前MV在螺旋搜索中的位置
        break;
      }
    }
  }

  //===== loop over search range (spiral search): get blockwise SAD =====
  for (pos = 0; pos < max_pos; pos++)
  {
    //abs_y,abs_x为参考宏块绝对坐标
    abs_y = offset_y + spiral_search_y[pos];
    abs_x = offset_x + spiral_search_x[pos];

    if (range_partly_outside)
    {
      //根据是否越界设置函数指针
      if (abs_y >= 0 && abs_y <= max_height &&
          abs_x >= 0 && abs_x <= max_width    )
      {
        PelYline_11 = FastLine16Y_11;//未越界
      }
      else
      {
        PelYline_11 = UMVLine16Y_11; //加了padding判断，慢，但是不怕出界
      }
    }
    
	/* 计算一个宏块内16 个 
	   4x4 块在搜索范围内所有位置的 SAD
	   循环一次算一个4x4小块
	*/
    orgptr = orig_blocks;
    bindex = 0;
    for (blky = 0; blky < 4; blky++)
    {
      //每一行4x4块，第一个4x4块为Blk0,接着Blk1...
      LineSadBlk0 = LineSadBlk1 = LineSadBlk2 = LineSadBlk3 = 0; //清零
      for (y = 0; y < 4; y++)//循环4次后就完成了4个block的SAD
      {
        // 取参考块像素  计算第1行16个像素值的SAD，再计算下一行16个像素值的SAD
        refptr = PelYline_11 (ref_pic, abs_y++, abs_x, img_height, img_width);//abs_y自加
        //byte_abs定义在875行
        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];//计算一个点的SAD
        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];//循环到第四次，计算Block0的SAD
        LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];//循环到第四次，计算Block1的SAD
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];//循环到第四次，计算Block2的SAD
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];//循环到第四次，计算Block3的SAD
      }
      block_sad[bindex++][pos] = LineSadBlk0;//[4*4子宏块位置索引][螺旋搜索位置索引]
      block_sad[bindex++][pos] = LineSadBlk1;
      block_sad[bindex++][pos] = LineSadBlk2;
      block_sad[bindex++][pos] = LineSadBlk3;
    }//循环结束时完成了16个小块的SAD
  }

  //===== combine SAD's for larger block types =====
  //利用上面的计算结果再计算出另外 6 种分割模式的 SAD
  SetupLargerBlocks (list, ref, max_pos);

  //===== set flag marking that search setup have been done =====
  //置标志为1，同一个参考帧就不要再计算了
  search_setup_done[list][ref] = 1;  
}
#endif // _FAST_FULL_ME_

/*!
 ************************************************************************
 * \brief
 *    Set motion vector predictor
 ************************************************************************
 */
 //需要根据子块周围的4X4邻块A,B,C,D的状况，
 //进行MV预测模式选择,结果存放在pmv[2]中
void SetMotionVectorPredictor (int  pmv[2],//返回预测的MV
                               int  ***refPic,      //当前编码帧每个4X4子块的参考帧索引
                               int  ****tmp_mv,//已经做过运动估计的子块的mv
                               int  ref_frame,    //参考帧索引号
                               int  list,//参考帧列表
                               int  block_x,//4*4块在宏块内的相对坐标
                               int  block_y,
                               int  blockshape_x,//预测块的宽
                               int  blockshape_y)
{
  int mb_x                 = 4*block_x; //获得MB内相对坐标(单位:象素) 
  int mb_y                 = 4*block_y;
  
  int mb_nr                = img->current_mb_nr;  //当前宏块地址
  int mv_a, mv_b, mv_c, pred_vec=0;
  int mvPredType, rFrameL, rFrameU, rFrameUR;
  int hv;
  //mvPredType --预测模式
  PixelPos block_a, block_b, block_c, block_d;  //A,B,C,D子块的坐标，尺寸,以4X4为单位
  
  int SAD_a=0, SAD_b=0, SAD_c=0, SAD_d=0;
  int temp_pred_SAD[2];

  if (input->FMEnable) pred_SAD_space=0; 

  /*
  (-1,0)计算左边紧邻宏块--block_a,
  (0,-1)计算上边紧邻宏块--block_b,
  (blockshape_x,-1)计算右上角紧邻宏块--block_c,
  (-1,-1)计算左上角紧邻宏块--block_d
  参考H.264乐园中帖子: MV预测过程详解
 */
  //得到4x4小块block_a的绝对坐标和宏块内坐标
  getLuma4x4Neighbour(mb_nr, block_x, block_y,           -1,  0, &block_a);
  //得到4x4小块block_b的绝对坐标和宏块内坐标
  getLuma4x4Neighbour(mb_nr, block_x, block_y,            0, -1, &block_b);
  //得到4x4小块block_c的绝对坐标和宏块内坐标
  getLuma4x4Neighbour(mb_nr, block_x, block_y, blockshape_x, -1, &block_c);
  //得到4x4小块block_d的绝对坐标和宏块内坐标
  getLuma4x4Neighbour(mb_nr, block_x, block_y,           -1, -1, &block_d);

  /*下面这段代码的作用是取得小于16x16的分块的相邻block_c的可用性问题
   block_c为什么要专门拿出来判断呢？画出图来就一目了然了:
   简单分析下block_a，block_b，block_d为什么没有这个困扰吧，
   因为无论是多大的分块，在进行ME之前，它的block_a，block_b，block_d都必然已经
   做过ME了，而block_c不一定，考虑如下情形: 当前MB分为两个16x8，下面那个16x8的block_c可用吗?
   当然没有，因为它的block_c在右边宏块的内部，还没做ME呢，因此有了下面这段代码
  */
  if (mb_y > 0)
  {
    if (mb_x < 8)  // first column of 8x8 blocks
    {
      if (mb_y==8)
      {
        if (blockshape_x == 16)      block_c.available  = 0;
        else                         block_c.available &= 1;
      }
      else
      {
        if (mb_x+blockshape_x != 8)  block_c.available &= 1;
        else                         block_c.available  = 0;
      }
    }
    else
    {
      if (mb_x+blockshape_x != 16)   block_c.available &= 1;
      else                           block_c.available  = 0;
    }
  }

  //block_c不可用就那block_d代替，block_d当然永远可用
  //但是，标准里是这样的吗? 也许只是代码实现的权宜之计吧
  if (!block_c.available)
  {
    block_c=block_d;
  }

  mvPredType = MVPRED_MEDIAN;

  if (!img->MbaffFrameFlag)//不是MBAFF
  { /*refPic是参考帧中4x4小块的索引号
    当相邻块不可用时，返回-1  计算周围宏块参考帧索引号
  */
    rFrameL    = block_a.available    ? refPic[list][block_a.pos_x][block_a.pos_y] : -1;
    rFrameU    = block_b.available    ? refPic[list][block_b.pos_x][block_b.pos_y] : -1;
    rFrameUR   = block_c.available    ? refPic[list][block_c.pos_x][block_c.pos_y] : -1;
  }
  else  //如果是MBAFF
  {
    if (img->mb_data[img->current_mb_nr].mb_field)//若当前块是场宏块(宏块对中的一个)
    {
      rFrameL    = block_a.available    ? 
        img->mb_data[block_a.mb_addr].mb_field ?      //如果相邻块也是field MB，
        refPic[list][block_a.pos_x][block_a.pos_y]:        //是:返回参考帧索引号
        refPic[list][block_a.pos_x][block_a.pos_y] * 2:  //否:返回两倍的参考帧索引号
        -1;
      rFrameU    = block_b.available    ? 
        img->mb_data[block_b.mb_addr].mb_field ? 
        refPic[list][block_b.pos_x][block_b.pos_y]:
        refPic[list][block_b.pos_x][block_b.pos_y] * 2: 
        -1;
      rFrameUR    = block_c.available    ? 
        img->mb_data[block_c.mb_addr].mb_field ? 
        refPic[list][block_c.pos_x][block_c.pos_y]:
        refPic[list][block_c.pos_x][block_c.pos_y] * 2: 
        -1;
    }
    else  //若当前块是帧宏块
    {
      rFrameL    = block_a.available    ? 
        img->mb_data[block_a.mb_addr].mb_field ?       //如果相邻块是field MB，
        refPic[list][block_a.pos_x][block_a.pos_y] >>1:  //是:返回参考帧索引号的一半
        refPic[list][block_a.pos_x][block_a.pos_y] :         //否:返回参考帧索引号
        -1;
      rFrameU    = block_b.available    ? 
        img->mb_data[block_b.mb_addr].mb_field ? 
        refPic[list][block_b.pos_x][block_b.pos_y] >>1:
        refPic[list][block_b.pos_x][block_b.pos_y] : 
        -1;
      rFrameUR    = block_c.available    ? 
        img->mb_data[block_c.mb_addr].mb_field ? 
        refPic[list][block_c.pos_x][block_c.pos_y] >>1:
        refPic[list][block_c.pos_x][block_c.pos_y] : 
        -1;
    }
  }
  /*总结: 当前块和相邻块同是帧宏块或场宏块，都只需返回参考帧索引号即可，
   若两者一个为帧宏块一个为场宏块，则帧宏块一方的参考帧索引号是场宏块的2倍
 */

  /* Prediction if only one of the neighbors uses the reference frame
   * we are checking
   * 若只有一个相邻块可用，pred_mv就用它
   */
  if(rFrameL == ref_frame && rFrameU != ref_frame && rFrameUR != ref_frame)       
      mvPredType = MVPRED_L;
  else if(rFrameL != ref_frame && rFrameU == ref_frame && rFrameUR != ref_frame)  
     mvPredType = MVPRED_U;
  else if(rFrameL != ref_frame && rFrameU != ref_frame && rFrameUR == ref_frame)  
      mvPredType = MVPRED_UR;
  // Directional predictions 
  /*如果当前宏块为 8*16 分割类型,
   对于左边 8*16 分割，如果 A 与当前分割为同一参考帧，
   则采用 A 的 MV 为该分割的最终 MV 预测值；
   对于右边 8*16 分割，如果 C 与当前分割为同一参考帧，
   则采用 C 的 MV 为该分割的最终 MV 预测值。
 */
  if(blockshape_x == 8 && blockshape_y == 16)
  {
    if(mb_x == 0)
    {
      if(rFrameL == ref_frame)
        mvPredType = MVPRED_L;   //A
    }
    else
    {
      if( rFrameUR == ref_frame)
        mvPredType = MVPRED_UR;  //C
    }
  }
  /*如果当前宏块为 16*8 分割类型,
   对于上边 16*8 分割，如果 B 与当前分割为同一参考帧，
   则采用 B 的 MV 为该分割的最终 MV 预测值；
   对于下边 16*8 分割，如果 A 与当前分割为同一参考帧，
   则采用 A 的 MV 为该分割的最终 MV 预测值。
 */  
  else if(blockshape_x == 16 && blockshape_y == 8)
  {
    if(mb_y == 0)
    {
      if(rFrameU == ref_frame)
        mvPredType = MVPRED_U;  //B
    }
    else
    {
      if(rFrameL == ref_frame)
        mvPredType = MVPRED_L;  //A
    }
  }

  //获取block_a,block_b,block_c的mv，tmp_mv就是enc_picture->mv，
  //而enc_picture->mv则是在函数PartitionMotionSearch中得到的各个分块的MV
  for (hv=0; hv < 2; hv++)
  {
    if (!img->MbaffFrameFlag || hv==0) //hv=0时不分帧和场，因为横向MV是一样的
    {
      mv_a = block_a.available  ? tmp_mv[list][block_a.pos_x][block_a.pos_y][hv] : 0;
      mv_b = block_b.available  ? tmp_mv[list][block_b.pos_x][block_b.pos_y][hv] : 0;
      mv_c = block_c.available  ? tmp_mv[list][block_c.pos_x][block_c.pos_y][hv] : 0;
    }
    else
    {      
      if (img->mb_data[img->current_mb_nr].mb_field)//场宏块
      {
        mv_a = block_a.available  ? img->mb_data[block_a.mb_addr].mb_field?
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv]:
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv] / 2:  //纵向帧宏块的MV是场宏块的2倍
          0;
        mv_b = block_b.available  ? img->mb_data[block_b.mb_addr].mb_field?
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv]:
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv] / 2: 
          0;
        mv_c = block_c.available  ? img->mb_data[block_c.mb_addr].mb_field?
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv]:
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv] / 2: 
          0;
      }
      else
      {//帧宏块
        mv_a = block_a.available  ? img->mb_data[block_a.mb_addr].mb_field?
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv] * 2://纵向帧宏块的MV是场宏块的2倍
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv]: 
          0;
        mv_b = block_b.available  ? img->mb_data[block_b.mb_addr].mb_field?
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv] * 2:
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv]: 
          0;
        mv_c = block_c.available  ? img->mb_data[block_c.mb_addr].mb_field?
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv] * 2:
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv]: 
          0;
      }
    }

  if(input->FMEnable)//FastME使能
  {//all_bwmincost 存储后向预测数据，  all_mincost存储MV和SAD信息
    SAD_a = block_a.available ? ((list==1) ? all_bwmincost[block_a.pos_x][block_a.pos_y][0][FME_blocktype][0]:all_mincost[block_a.pos_x][block_a.pos_y][ref_frame][FME_blocktype][0]):0;
    SAD_b = block_b.available ? ((list==1) ? all_bwmincost[block_b.pos_x][block_b.pos_y][0][FME_blocktype][0]:all_mincost[block_b.pos_x][block_b.pos_y][ref_frame][FME_blocktype][0]):0;
    SAD_d = block_d.available ? ((list==1) ? all_bwmincost[block_d.pos_x][block_d.pos_y][0][FME_blocktype][0]:all_mincost[block_d.pos_x][block_d.pos_y][ref_frame][FME_blocktype][0]):0;
    SAD_c = block_c.available ? ((list==1) ? all_bwmincost[block_c.pos_x][block_c.pos_y][0][FME_blocktype][0]:all_mincost[block_c.pos_x][block_c.pos_y][ref_frame][FME_blocktype][0]):SAD_d;
  }

    switch (mvPredType)  //根据mvPredType计算pmv
    {
    case MVPRED_MEDIAN:
      if(!(block_b.available || block_c.available))//若B、C都不可用，pred_vec就用mv_a
      {
        pred_vec = mv_a;
        if(input->FMEnable) temp_pred_SAD[hv] = SAD_a;
      }
      else    //否则，取三者中值，注意中值与平均值的区别
      {
        pred_vec = mv_a+mv_b+mv_c-min(mv_a,min(mv_b,mv_c))-max(mv_a,max(mv_b,mv_c));
      }
      if(input->FMEnable)
      {
         if (pred_vec == mv_a && SAD_a != 0) temp_pred_SAD[hv] = SAD_a;
         else if (pred_vec == mv_b && SAD_b!=0) temp_pred_SAD[hv] = SAD_b;
              else temp_pred_SAD[hv] = SAD_c;
      }
      break;
    case MVPRED_L:
      pred_vec = mv_a;
      if(input->FMEnable) temp_pred_SAD[hv] = SAD_a;
      break;
    case MVPRED_U:
      pred_vec = mv_b;
      if(input->FMEnable) temp_pred_SAD[hv] = SAD_b;
      break;
    case MVPRED_UR:
      pred_vec = mv_c;
      if(input->FMEnable) temp_pred_SAD[hv] = SAD_c;
      break;
    default:
      break;
    }

    pmv[hv] = pred_vec;

  }
//取小的SAD
  if(input->FMEnable) pred_SAD_space = temp_pred_SAD[0]>temp_pred_SAD[1]?temp_pred_SAD[1]:temp_pred_SAD[0];
}

/*!
 ************************************************************************
 * \brief
 *    Initialize the motion search
 spiral_search_x，spiral_search_y(螺旋搜索坐标初始化)，byte_abs，refbits，mvbits初始化
 这么复杂的初始化什么作用???
 ************************************************************************
 */
void
Init_Motion_Search_Module ()
{
  int bits, i, imin, imax, k, l;
//floor 下取整 ceil上取整  
  int search_range               = input->search_range;
  int number_of_reference_frames = img->max_num_references;
  int max_search_points          = (2*search_range+1)*(2*search_range+1);//最大的搜索号
  int max_ref_bits               = 1 + 2 * (int)floor(log(max(16,number_of_reference_frames+1)) / log(2) + 1e-10);//前向和后向
  int max_ref                    = (1<<((max_ref_bits>>1)+1))-1;//得到最大参考帧号
  int number_of_subpel_positions = 4 * (2*search_range+3);//螺旋搜索
  int max_mv_bits                = 3 + 2 * (int)ceil (log(number_of_subpel_positions+1) / log(2) + 1e-10);//x,y两个坐标需要的bit数
  max_mvd                        = (1<<( max_mv_bits >>1)   )-1;


  //=====   CREATE ARRAYS   =====
  //-----------------------------
  //calloc在内存的动态存储区中分配n个长度为size的连续空间，函数返回一个指向分配起始地址的指针
  //与malloc的区别   malloc不初始化，里面数据是随机的垃圾数据 calloc初始化为0
  if ((spiral_search_x = (int*)calloc(max_search_points, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: spiral_search_x");
  if ((spiral_search_y = (int*)calloc(max_search_points, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: spiral_search_y");
  if ((mvbits = (int*)calloc(2*max_mvd+1, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: mvbits");
  if ((refbits = (int*)calloc(max_ref, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: refbits");
  if ((byte_abs = (int*)calloc(512, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: byte_abs");

  get_mem4Dint (&motion_cost, 8, 2, img->max_num_references+1, 4);

  //设置矩阵初始偏移量
  mvbits   += max_mvd;//入口地址偏移max_mvd，左max_mvd 右max_mvd
  byte_abs += 256;


  //=====   INIT ARRAYS   =====
  //---------------------------
  //--- init array: motion vector bits ---
  mvbits[0] = 1;
  for (bits=3; bits<=max_mv_bits; bits+=2)
  {
    imax = 1    << (bits >> 1);
    imin = imax >> 1;

    for (i = imin; i < imax; i++)   mvbits[-i] = mvbits[i] = bits;
  }
  //--- init array: reference frame bits ---
  // 计算各参考帧索引号以 te(v) 方式编码（参见标准：9.1 小节）所需要使用的比特数。例如，索引号
  // 如果为 4，即当前块参考帧为前向参考列表中的第 5 帧时，编码 4 所需要的比特数为 5 个比特
  refbits[0] = 1;
  for (bits=3; bits<=max_ref_bits; bits+=2)
  {
    imax = (1   << ((bits >> 1) + 1)) - 1;
    imin = imax >> 1;

    for (i = imin; i < imax; i++)   refbits[i] = bits;
  }
  //--- init array: absolute value ---
  byte_abs[0] = 0;
  for (i=1; i<256; i++)   byte_abs[i] = byte_abs[-i] = i;
  //--- init array: search pattern ---
  spiral_search_x[0] = spiral_search_y[0] = 0;
  for (k=1, l=1; l<=max(1,search_range); l++)
  {
    for (i=-l+1; i< l; i++)
    {
      spiral_search_x[k] =  i;  spiral_search_y[k++] = -l;
      spiral_search_x[k] =  i;  spiral_search_y[k++] =  l;
    }
    for (i=-l;   i<=l; i++)
    {
      spiral_search_x[k] = -l;  spiral_search_y[k++] =  i;
      spiral_search_x[k] =  l;  spiral_search_y[k++] =  i;
    }
  }

#ifdef _FAST_FULL_ME_
  if(!input->FMEnable)
    InitializeFastFullIntegerSearch ();
#endif
}


/*!
 ************************************************************************
 * \brief
 *    Free memory used by motion search
 ************************************************************************
 */
void
Clear_Motion_Search_Module ()
{
  //--- correct array offset ---
  mvbits   -= max_mvd;
  byte_abs -= 256;

  //--- delete arrays ---
  free (spiral_search_x);
  free (spiral_search_y);
  free (mvbits);
  free (refbits);
  free (byte_abs);
  free_mem4Dint (motion_cost, 8, 2);

#ifdef _FAST_FULL_ME_
  if(!input->FMEnable)
    ClearFastFullIntegerSearch ();
#endif
}



/*!
 ***********************************************************************
 * \brief按照螺旋搜索顺序进行全搜索
 *    Full pixel block motion search
 *    目标是得到(mv_x,mv_y)和min_mcost，(mv_x,mv_y)指示从哪里开始做分像素搜索，search center
 *    后者用来跟分像素搜索结果做比较
 ***********************************************************************
 */
int                                               //  ==> minimum motion cost after search
FullPelBlockMotionSearch (pel_t**   orig_pic,     // <--  original pixel values for the AxB block
                          int       ref,          // <--  reference frame (0... or -1 (backward))
                          int       list,
                          int       pic_pix_x,    // <--  absolute x-coordinate of regarded AxB blockAxB宏块原点在图像中的绝对坐标
                          int       pic_pix_y,    // <--  absolute y-coordinate of regarded AxB block
                          int       blocktype,    // <--  block type (1-16x16 ... 7-4x4)
                          int       pred_mv_x,    // <--  motion vector predictor (x) in sub-pel units
                          int       pred_mv_y,    // <--  motion vector predictor (y) in sub-pel units
                          int*      mv_x,         // <--> in: search center (x) / out: motion vector (x) - in pel units
                          int*      mv_y,         // <--> in: search center (y) / out: motion vector (y) - in pel units
                          int       search_range, // <--  1-d search range in pel units
                          int       min_mcost,    // <--  minimum motion cost (cost for center or huge value)
                          double    lambda)       // <--  lagrangian parameter for determining motion cost
{
  int   pos, cand_x, cand_y, y, x4, mcost;
  
  pel_t *orig_line, *ref_line;
  pel_t *(*get_ref_line)(int, pel_t*, int, int, int, int);//
//参考帧偏移量 帧场自适应且宏块地址为偶数=4 帧场自适应宏块地址为奇数=2 非帧场自适应=0
  int   list_offset   = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;
  pel_t *ref_pic			= listX[list+list_offset][ref]->imgY_11;
  int   img_width     = listX[list+list_offset][ref]->size_x;
  int   img_height    = listX[list+list_offset][ref]->size_y;

  int   best_pos      = 0;                                        // position with minimum motion cost
  //计算最大需要搜索的位置个数
  int   max_pos       = (2*search_range+1)*(2*search_range+1);    // number of search positions
  int   lambda_factor = LAMBDA_FACTOR (lambda);                   // factor for determining lagragian motion cost
  int   blocksize_y   = input->blc_size[blocktype][1];            // vertical block size
  int   blocksize_x   = input->blc_size[blocktype][0];            // horizontal block size
  int   blocksize_x4  = blocksize_x >> 2;                         // horizontal block size in 4-pel units
  int   pred_x        = (pic_pix_x << 2) + pred_mv_x;       // predicted position x (in sub-pel units)1/4子像素为单位的预测MV
  int   pred_y        = (pic_pix_y << 2) + pred_mv_y;       // predicted position y (in sub-pel units)
  int   center_x      = pic_pix_x + *mv_x;                        // center position x (in pel units)
  int   center_y      = pic_pix_y + *mv_y;                        // center position y (in pel units)
  int   check_for_00  = (blocktype==1 && !input->rdopt && img->type!=B_SLICE && ref==0);

  //===== set function for getting reference picture lines =====
  //通过判断搜索范围会不会出界，设置获取参考像素值的函数
  if ((center_x > search_range) && (center_x < img->width -1-search_range-blocksize_x) &&
      (center_y > search_range) && (center_y < img->height-1-search_range-blocksize_y)   )
  {
     get_ref_line = FastLineX;//未出界
  }
  else
  {
     get_ref_line = UMVLineX;//出界
  }


  //===== loop over all search positions =====
  //max_pos是搜索位置的个数,计算见上面
  for (pos=0; pos<max_pos; pos++)
  {
    //--- set candidate position (absolute position in pel units) ---
    /*(center_x,center_y)是由预测MV估计出来的搜索中心，在以它为中心的范围内，
    对按照螺旋形顺序排列的候选点进行搜索，
    每个候选点都是一个可能参考块的左上角起始点
  */
    cand_x = center_x + spiral_search_x[pos];//螺旋搜索
    cand_y = center_y + spiral_search_y[pos];

    //--- initialize motion cost (cost for motion vector) and check ---
    //计算MVD的代价，换算成四分之一像素(cand--candidate候选点)
    mcost = MV_COST (lambda_factor, 2, cand_x, cand_y, pred_x, pred_y);
    if (check_for_00 && cand_x==pic_pix_x && cand_y==pic_pix_y)
    {//螺旋搜索到的点为预测MV指向的点
      mcost -= WEIGHTED_COST (lambda_factor, 16);
    }
    //如果只是MV的代价就已经大于现有的最小代价就舍弃
    if (mcost >= min_mcost)   continue;

    //--- add residual cost to motion cost ---
    //blocksize_y blocksize_x4 是分块大小16x16 16x8 8x16......
    for (y=0; y<blocksize_y; y++) 
    {
      //(cand_x,cand_y+y)是一行的起始坐标，y++ 遍历每一行
      ref_line  = get_ref_line (blocksize_x, ref_pic, cand_y+y, cand_x, img_height, img_width);
      orig_line = orig_pic [y];
      //计算当前帧和参考帧的像素残差
      for (x4=0; x4<blocksize_x4; x4++) //以4个为一组计算
      {
        mcost += byte_abs[ *orig_line++ - *ref_line++ ];
        mcost += byte_abs[ *orig_line++ - *ref_line++ ];
        mcost += byte_abs[ *orig_line++ - *ref_line++ ];
        mcost += byte_abs[ *orig_line++ - *ref_line++ ];
      }

      if (mcost >= min_mcost)  //如果已经比最小代价大，就没必要计算下面的行了
      {
        break;
      }
    }
	
    //--- check if motion cost is less than minimum cost ---
    //记录下最小代价和最佳匹配位置
    if (mcost < min_mcost)
    {
      best_pos  = pos;
      min_mcost = mcost;
    }
  }


  //===== set best motion vector and return minimum motion cost =====
  if (best_pos)
  {
    *mv_x += spiral_search_x[best_pos];  //因为螺旋搜索数组中记录的是该位置的点
    *mv_y += spiral_search_y[best_pos];  //与(center_x,center_y)的差
  }
  return min_mcost;  //返回最小代价
}


#ifdef _FAST_FULL_ME_
/*!
 ***********************************************************************
 * \brief快速正像素搜索
 *    Fast Full pixel block motion search
 *    目标是得到(mv_x,mv_y)和min_mcost，(mv_x,mv_y)指示从哪里开始做分像素搜索，search center
 *    后者用来跟分像素搜索结果做比较
 ***********************************************************************
 */
int                                                   //  ==> minimum motion cost after search
FastFullPelBlockMotionSearch (pel_t**   orig_pic,     // <--  not used
                              int       ref,          // <--  reference frame (0... or -1 (backward))
                              int       list,
                              int       pic_pix_x,    // <--  absolute x-coordinate of regarded AxB block
                              int       pic_pix_y,    // <--  absolute y-coordinate of regarded AxB block
                              int       blocktype,    // <--  block type (1-16x16 ... 7-4x4)
                              int       pred_mv_x,    // <--  motion vector predictor (x) in sub-pel units
                              int       pred_mv_y,    // <--  motion vector predictor (y) in sub-pel units
                              int*      mv_x,         //  --> motion vector (x) - in pel units
                              int*      mv_y,         //  --> motion vector (y) - in pel units
                              int       search_range, // <--  1-d search range in pel units
                              int       min_mcost,    // <--  minimum motion cost (cost for center or huge value)
                              double    lambda)       // <--  lagrangian parameter for determining motion cost
{
  int   pos, offset_x, offset_y, cand_x, cand_y, mcost;

  int   max_pos       = (2*search_range+1)*(2*search_range+1);              // number of search positions
  int   lambda_factor = LAMBDA_FACTOR (lambda);                             // factor for determining lagragian motion cost
  int   best_pos      = 0;                                                  // position with minimum motion cost
  int   block_index;                                                        // block index for indexing SAD array
  int*  block_sad;                                                          // pointer to SAD array

  block_index   = (pic_pix_y-img->opix_y)+((pic_pix_x-img->opix_x)>>2); // block index for indexing SAD array
  block_sad     = BlockSAD[list][ref][blocktype][block_index];         // pointer to SAD array

  //===== set up fast full integer search if needed / set search center =====
  if (!search_setup_done[list][ref])//对一个参考帧只做一次
  {
	//计算搜索范围所有位置所有分块模式的SAD(整像素)
    SetupFastFullPelSearch (ref, list);
  }

  offset_x = search_center_x[list][ref] - img->opix_x; //搜索中心相对原宏块的偏移
  offset_y = search_center_y[list][ref] - img->opix_y;

  //===== cost for (0,0)-vector: it is done before, because MVCost can be negative =====
  if (!input->rdopt)
  {
    //把刚才计算的SAD 跟mv代价相加得到总代价
    mcost = block_sad[pos_00[list][ref]] + MV_COST (lambda_factor, 2, 0, 0, pred_mv_x, pred_mv_y);
    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos_00[list][ref];//每帧搜索中心的位置
    }
  }

  //===== loop over all search positions =====
  for (pos=0; pos<max_pos; pos++, block_sad++)
  {
    //--- check residual cost ---
    if (*block_sad < min_mcost)
    {
      //--- get motion vector cost ---
      //计算出搜索位置，按照螺旋形顺序spiral_search_xy
      cand_x = offset_x + spiral_search_x[pos];
      cand_y = offset_y + spiral_search_y[pos];
      mcost  = *block_sad;
      mcost += MV_COST (lambda_factor, 2, cand_x, cand_y, pred_mv_x, pred_mv_y);	//计算 MV 代价

      //--- check motion cost ---
      if (mcost < min_mcost)
      {
        min_mcost = mcost;
        best_pos  = pos;
      }
    }
  }

  //===== set best motion vector and return minimum motion cost =====
  *mv_x = offset_x + spiral_search_x[best_pos];//根据代价最小，计算出最佳MV
  *mv_y = offset_y + spiral_search_y[best_pos];
  return min_mcost;
}
#endif


/*!
 ***********************************************************************
 * \brief
 *    Calculate SA(T)D
 ***********************************************************************
 */
int SATD (int* diff, int use_hadamard)
{
  int k, satd = 0, m[16], dd, *d=diff;
  
  if (use_hadamard)	// 如果采用了Hadamard变换，则先对残差块进行Hadamard变换，然后将变换后的16个残差值取绝对值相加作为代价
  {
    /*===== hadamard transform =====*/
    m[ 0] = d[ 0] + d[12];
    m[ 4] = d[ 4] + d[ 8];
    m[ 8] = d[ 4] - d[ 8];
    m[12] = d[ 0] - d[12];
    m[ 1] = d[ 1] + d[13];
    m[ 5] = d[ 5] + d[ 9];
    m[ 9] = d[ 5] - d[ 9];
    m[13] = d[ 1] - d[13];
    m[ 2] = d[ 2] + d[14];
    m[ 6] = d[ 6] + d[10];
    m[10] = d[ 6] - d[10];
    m[14] = d[ 2] - d[14];
    m[ 3] = d[ 3] + d[15];
    m[ 7] = d[ 7] + d[11];
    m[11] = d[ 7] - d[11];
    m[15] = d[ 3] - d[15];
    
    d[ 0] = m[ 0] + m[ 4];
    d[ 8] = m[ 0] - m[ 4];
    d[ 4] = m[ 8] + m[12];
    d[12] = m[12] - m[ 8];
    d[ 1] = m[ 1] + m[ 5];
    d[ 9] = m[ 1] - m[ 5];
    d[ 5] = m[ 9] + m[13];
    d[13] = m[13] - m[ 9];
    d[ 2] = m[ 2] + m[ 6];
    d[10] = m[ 2] - m[ 6];
    d[ 6] = m[10] + m[14];
    d[14] = m[14] - m[10];
    d[ 3] = m[ 3] + m[ 7];
    d[11] = m[ 3] - m[ 7];
    d[ 7] = m[11] + m[15];
    d[15] = m[15] - m[11];
    
    m[ 0] = d[ 0] + d[ 3];
    m[ 1] = d[ 1] + d[ 2];
    m[ 2] = d[ 1] - d[ 2];
    m[ 3] = d[ 0] - d[ 3];
    m[ 4] = d[ 4] + d[ 7];
    m[ 5] = d[ 5] + d[ 6];
    m[ 6] = d[ 5] - d[ 6];
    m[ 7] = d[ 4] - d[ 7];
    m[ 8] = d[ 8] + d[11];
    m[ 9] = d[ 9] + d[10];
    m[10] = d[ 9] - d[10];
    m[11] = d[ 8] - d[11];
    m[12] = d[12] + d[15];
    m[13] = d[13] + d[14];
    m[14] = d[13] - d[14];
    m[15] = d[12] - d[15];
    
    d[ 0] = m[ 0] + m[ 1];
    d[ 1] = m[ 0] - m[ 1];
    d[ 2] = m[ 2] + m[ 3];
    d[ 3] = m[ 3] - m[ 2];
    d[ 4] = m[ 4] + m[ 5];
    d[ 5] = m[ 4] - m[ 5];
    d[ 6] = m[ 6] + m[ 7];
    d[ 7] = m[ 7] - m[ 6];
    d[ 8] = m[ 8] + m[ 9];
    d[ 9] = m[ 8] - m[ 9];
    d[10] = m[10] + m[11];
    d[11] = m[11] - m[10];
    d[12] = m[12] + m[13];
    d[13] = m[12] - m[13];
    d[14] = m[14] + m[15];
    d[15] = m[15] - m[14];
    
    /*===== sum up =====*/
    for (dd=diff[k=0]; k<16; dd=diff[++k])
    {
      satd += (dd < 0 ? -dd : dd);
    }
    satd >>= 1;
  }
  else	// 如果未采用Hadamard变换，则直接将16个残差值取绝对值相加作为代价
  {
    /*===== sum up =====*/
    for (k = 0; k < 16; k++)
    {
      satd += byte_abs [diff [k]];
    }
  }
  
  return satd;
}



/*!
 ***********************************************************************
 * \brief以确定的最佳像素点为中心，进行半像素估计，在以最佳半像素点为中心，进行
 1/4像素估计
 *    Sub pixel block motion search
 ***********************************************************************
 */
int                                               //  ==> minimum motion cost after search
SubPelBlockMotionSearch (pel_t**   orig_pic,      // <--  original pixel values for the AxB block
                         int       ref,           // <--  reference frame (0... or -1 (backward))
                         int       list,          // <--  reference picture list 
                         int       pic_pix_x,     // <--  absolute x-coordinate of regarded AxB block
                         int       pic_pix_y,     // <--  absolute y-coordinate of regarded AxB block
                         int       blocktype,     // <--  block type (1-16x16 ... 7-4x4)
                         int       pred_mv_x,     // <--  motion vector predictor (x) in sub-pel units
                         int       pred_mv_y,     // <--  motion vector predictor (y) in sub-pel units
                         int*      mv_x,          // <--> in: search center (x) / out: motion vector (x) - in pel units
                         int*      mv_y,          // <--> in: search center (y) / out: motion vector (y) - in pel units
                         int       search_pos2,   // <--  半像素搜索点的位置 (default: 9)
                         int       search_pos4,   // <--  search positions for quarter-pel search  (default: 9)
                         int       min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                         double    lambda         // <--  lagrangian parameter for determining motion cost
                         )
{
  int   diff[16], *d;
  int   pos, best_pos, mcost, abort_search;
  int   y0, x0, ry0, rx0, ry;
  int   cand_mv_x, cand_mv_y;
  int   max_pos_x4, max_pos_y4;
  pel_t *orig_line;
  pel_t **ref_pic;      
  StorablePicture *ref_picture;
  int   lambda_factor   = LAMBDA_FACTOR (lambda);
  int   mv_shift        = 0;
  int   check_position0 = (blocktype==1 && *mv_x==0 && *mv_y==0 && input->hadamard && !input->rdopt && img->type!=B_SLICE && ref==0);
  int   blocksize_x     = input->blc_size[blocktype][0];
  int   blocksize_y     = input->blc_size[blocktype][1];
  int   pic4_pix_x      = (pic_pix_x << 2);//换算成以1/4像素为单位的坐标
  int   pic4_pix_y      = (pic_pix_y << 2);
  int   min_pos2        = (input->hadamard ? 0 : 1);
  int   max_pos2        = (input->hadamard ? max(1,search_pos2) : search_pos2);
  int   list_offset     = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

  int  apply_weights = ( (active_pps->weighted_pred_flag && (img->type == P_SLICE || img->type == SP_SLICE)) ||
                         (active_pps->weighted_bipred_idc && (img->type == B_SLICE)));  //是否使用权重计算代价

  int   img_width, img_height;
  
  ref_picture     = listX[list+list_offset][ref];

  if (apply_weights)
  {
    ref_pic = listX[list+list_offset][ref]->imgY_ups_w;
  }
  else
    ref_pic = listX[list+list_offset][ref]->imgY_ups; //做过1/4像素插值的图像放在imgY_ups

  img_width  = ref_picture->size_x;
  img_height = ref_picture->size_y;//未做插值的图像的宽高

  max_pos_x4      = ((ref_picture->size_x - blocksize_x+1)<<2);//最大1/4像素搜索位置
  max_pos_y4      = ((ref_picture->size_y - blocksize_y+1)<<2);
  
  /*********************************
   *****                       *****
   *****  HALF-PEL REFINEMENT  *****
   *****                       *****
   *********************************/
  //===== convert search center to quarter-pel units ===== 先变成1/4像素的运动向量
  *mv_x <<= 2;
  *mv_y <<= 2;
  //===== set function for getting pixel values =====
  if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 2) &&
      (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 2)   )//判断半像素是否出界
  {
    PelY_14 = FastPelY_14;//没出界
  }
  else
  {
    PelY_14 = UMVPelY_14;
  }
  //===== loop over search positions =====
  for (best_pos = 0, pos = min_pos2; pos < max_pos2; pos++)	//半像素搜索，只搜索最近的9个点(max_pos2==9)
  {
    cand_mv_x = *mv_x + (spiral_search_x[pos] << 1);    // quarter-pel units
    cand_mv_y = *mv_y + (spiral_search_y[pos] << 1);    // quarter-pel units

    //----- set motion vector cost -----
    mcost = MV_COST (lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);
    if (check_position0 && pos==0)
    {
      mcost -= WEIGHTED_COST (lambda_factor, 16);
    }

    if (mcost >= min_mcost) continue;

    //----- add up SATD -----
    for (y0=0, abort_search=0; y0<blocksize_y && !abort_search; y0+=4)
    {
      ry0 = ((pic_pix_y+y0)<<2) + cand_mv_y;//搜索中心

      for (x0=0; x0<blocksize_x; x0+=4)//4*4块计算SATD
      {
        rx0 = ((pic_pix_x+x0)<<2) + cand_mv_x;
        d   = diff;//共享内存空间
//找到1/4像素的搜索中心，根据插值后的像素值与整数像素值同位置出相减
        orig_line = orig_pic [y0  ];    ry=ry0;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+1];    ry=ry0+4;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+2];    ry=ry0+8;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+3];    ry=ry0+12;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d        = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        if ((mcost += SATD (diff, input->hadamard)) > min_mcost)
        {
          abort_search = 1;
          break;
        }//比最小代价大，终止搜索
      }
    }

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
    }
  }
  if (best_pos)
  {
    *mv_x += (spiral_search_x [best_pos] << 1);
    *mv_y += (spiral_search_y [best_pos] << 1);
  }


  /************************************
   *****                          *****
   *****  QUARTER-PEL REFINEMENT  *****
   *****                          *****
   ************************************/
  //===== set function for getting pixel values =====
  if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 1) &&
      (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 1)   )
  {
    PelY_14 = FastPelY_14;
  }
  else
  {
    PelY_14 = UMVPelY_14;
  }
  //===== loop over search positions =====
  for (best_pos = 0, pos = 1; pos < search_pos4; pos++)//1/4像素搜索，只搜索最近的9个点(max_pos2==9)
  {
    cand_mv_x = *mv_x + spiral_search_x[pos];    // quarter-pel units
    cand_mv_y = *mv_y + spiral_search_y[pos];    // quarter-pel units

    //----- set motion vector cost -----
    mcost = MV_COST (lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);

    if (mcost >= min_mcost) continue;

    //----- add up SATD -----
    for (y0=0, abort_search=0; y0<blocksize_y && !abort_search; y0+=4)
    {
      ry0 = ((pic_pix_y+y0)<<2) + cand_mv_y;

      for (x0=0; x0<blocksize_x; x0+=4)
      {
        rx0 = ((pic_pix_x+x0)<<2) + cand_mv_x;
        d   = diff;

        orig_line = orig_pic [y0  ];    ry=ry0;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+1];    ry=ry0+4;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+2];    ry=ry0+8;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+3];    ry=ry0+12;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d        = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        if ((mcost += SATD (diff, input->hadamard)) > min_mcost)
        {
          abort_search = 1;
          break;
        }
      }
    }

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
    }
  }
  if (best_pos)
  {
    *mv_x += spiral_search_x [best_pos];
    *mv_y += spiral_search_y [best_pos];
  }

  //===== return minimum motion cost =====
  return min_mcost;
}



/*!
 ***********************************************************************
 * \brief
 *    Block motion search  进行一个分块的运动搜索并返回运动代价
 ***********************************************************************
 */
int                                         //!< minimum motion cost after search
BlockMotionSearch (int       ref,           //!< reference idx
                   int       list,          //!< reference pciture list
                   int       mb_x,          //!< x-coordinate inside macroblock 以像素为单位MB内坐标
                   int       mb_y,          //!< y-coordinate inside macroblock 以像素为单位
                   int       blocktype,     //!< block type (1-16x16 ... 7-4x4)
                   int       search_range,  //!< 1-d search range for integer-position search
                   double    lambda         //!< lagrangian parameter for determining motion cost
                   )
{
  static pel_t   orig_val [256];//存储一个MB的像素
  //用下面这个数组使2维数组操作速度上会快一些，因为减少了下标操作
  static pel_t  *orig_pic  [16] = {orig_val,     orig_val+ 16, orig_val+ 32, orig_val+ 48,
                                   orig_val+ 64, orig_val+ 80, orig_val+ 96, orig_val+112,
                                   orig_val+128, orig_val+144, orig_val+160, orig_val+176,
                                   orig_val+192, orig_val+208, orig_val+224, orig_val+240};

  int       pred_mv_x, pred_mv_y, mv_x, mv_y, i, j;

  int       max_value = (1<<20);//40
  int       min_mcost = max_value; //用来存最小代价,先存个最大值

  int       block_x   = (mb_x>>2); //再变回以4x4块为单位
  int       block_y   = (mb_y>>2); //同上
  
  int       bsx       = input->blc_size[blocktype][0]; //block size x
  int       bsy       = input->blc_size[blocktype][1]; //block size y

  int       pic_pix_x = img->opix_x + mb_x;   //img->opix_xy 宏块原点的像素坐标
  int       pic_pix_y = img->opix_y + mb_y;

  int*      pred_mv;

  int***    mv_array  = enc_picture->mv[list];

  int****** all_mv    = img->all_mv;

#ifdef WIN32
  struct _timeb tstruct1;
  struct _timeb tstruct2;
#else
  struct timeb tstruct1;
  struct timeb tstruct2;
#endif
  
  int me_tmp_time;

  int  N_Bframe=0, n_Bframe=0;
  if(input->FMEnable)//快速运动估计
  {
    N_Bframe = input->successive_Bframe;
    n_Bframe =(N_Bframe) ? ((Bframe_ctr%N_Bframe)+1) : 0 ;
  }

   pred_mv = img->pred_mv[block_x][block_y][list][ref][blocktype];

  //==================================
  //=====   GET ORIGINAL BLOCK   =====
  //==================================
  for (j = 0; j < bsy; j++)
  {
    for (i = 0; i < bsx; i++)
    {
      orig_pic[j][i] = imgY_org[pic_pix_y+j][pic_pix_x+i];	//得到待编码原始图像的亮度像素值
    }
  }

  //如果使用快速运动估计，默认是不使用的(baseline,main,extended一样)
  //要了解这段代码可能需要阅读相关论文，标准中是没有的，关于FME的代码暂时不看
  if(input->FMEnable)
  {
    
    if(blocktype>6)
    {//all_mv[block_x][block_y][list][ref][blocktype][MV_x/MV_y]
    //all_bwmincost[block_x][block_y][ref][blocktype][0:SAD 1:MV_x 2:MV_y]
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][5][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][5][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][5][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][5][0]);
      pred_SAD_uplayer   /= 2; 
      
    }
    else if(blocktype>4)
    {
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][4][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][4][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][4][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][4][0]);
      pred_SAD_uplayer   /= 2; 
      
    }
    else if(blocktype == 4)
    {
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][2][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][2][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][2][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][2][0]);
      pred_SAD_uplayer   /= 2; 
    }
    else if(blocktype > 1)
    {
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][1][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][1][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][1][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][1][0]);
      pred_SAD_uplayer   /= 2; 
    }
    
    if ((img->type==B_SLICE)&& (img->nal_reference_idc>0))
    {
      if(blocktype>6)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][5][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][5][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][5][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][5][0]);
        pred_SAD_uplayer   /= 2; 
      }
      else if(blocktype>4)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][4][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][4][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][4][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][4][0]);
        pred_SAD_uplayer   /= 2; 
      }
      else if(blocktype == 4)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][2][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][2][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][2][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][2][0]);
        pred_SAD_uplayer   /= 2; 
      }
      else if(blocktype > 1)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][1][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][1][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][1][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][1][0]);
        pred_SAD_uplayer   /= 2; 
      }
    }
    
    pred_SAD_uplayer = flag_intra_SAD ? 0 : pred_SAD_uplayer;// for irregular motion
    
    //Coordinate prediction
    if (img->number > ref+1)
    {
      pred_SAD_time = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][0];
      pred_MV_time[0] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][1];
      pred_MV_time[1] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][2];
    }
    
    if(list==1 && (Bframe_ctr%N_Bframe) > 1) 
    {
      pred_SAD_time = all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][0];
      pred_MV_time[0] = (int)(all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][1] * ((n_Bframe==1) ? (N_Bframe) : (N_Bframe-n_Bframe+1.0)/(N_Bframe-n_Bframe+2.0)) );//should add a factor
      pred_MV_time[1] = (int)(all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][2] *((n_Bframe==1) ? (N_Bframe) : (N_Bframe-n_Bframe+1.0)/(N_Bframe-n_Bframe+2.0)) );//should add a factor
    }
    
    if (input->PicInterlace == FIELD_CODING) 
    {
      if (img->type == P_SLICE && ref > 1)
      {
        pred_SAD_ref = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-2)][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-2)][blocktype][1];
        pred_MV_ref[0] = (int)(pred_MV_ref[0]*((ref>>1)+1)/(float)((ref>>1)));
        pred_MV_ref[1] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-2)][blocktype][2];
        pred_MV_ref[1] = (int)(pred_MV_ref[1]*((ref>>1)+1)/(float)((ref>>1)));
      }
      if (img->type == B_SLICE && list==0 && (ref==0 || ref==1) )
      {
        pred_SAD_ref = all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] =(int) (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][1]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); //should add a factor
        pred_MV_ref[1] =(int) ( all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][2]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); 
      }
    }
    else //frame case
    {
      if (ref > 0)
      {//field_mode top_field
        pred_SAD_ref = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-1)][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-1)][blocktype][1];
        pred_MV_ref[0] = (int)(pred_MV_ref[0]*(ref+1)/(float)(ref));
        pred_MV_ref[1] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-1)][blocktype][2];
        pred_MV_ref[1] = (int)(pred_MV_ref[1]*(ref+1)/(float)(ref));
      }
      if (img->type == B_SLICE && (list==0 && ref==0)) //B frame forward prediction, first ref
      {
        pred_SAD_ref = all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] =(int) (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][1]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); //should add a factor
        pred_MV_ref[1] =(int) ( all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][2]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); 
      }
    }
 }
 
  //===========================================
  //=====   GET MOTION VECTOR PREDICTOR   =====
  //===========================================

  if (input->FMEnable) 
    FME_blocktype=blocktype;

   //计算预测MV，放在pred_mv中，以四分之一像素为单位
  SetMotionVectorPredictor (pred_mv, enc_picture->ref_idx, enc_picture->mv, ref, list, block_x, block_y, bsx, bsy);
  
  pred_mv_x = pred_mv[0];
  pred_mv_y = pred_mv[1]; //下面的运动估计就是基于预测mv的
#ifdef WIN32
  _ftime( &tstruct1 );    // start time ms
#else
  ftime(&tstruct1);
#endif

  //==================================
  //=====   INTEGER-PEL SEARCH ===== 整数运动估计
  //==================================

  if(input->FMEnable)
  {
    mv_x = pred_mv_x / 4;   
    mv_y = pred_mv_y / 4;
    if (!input->rdopt)
    {
      //--- adjust search center so that the (0,0)-vector is inside ---
      mv_x = max (-search_range, min (search_range, mv_x));
      mv_y = max (-search_range, min (search_range, mv_y));
	  //使mv_x,mv_y在(-search_range,search_range)之间
    }
    
    min_mcost = FastIntegerPelBlockMotionSearch(orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                                                          pred_mv_x, pred_mv_y, &mv_x, &mv_y, 
                                                                          search_range,min_mcost, lambda);
    //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3
    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
        if(list == 0) 
          all_mincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][0] = min_mcost;
        else
          all_bwmincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][0] = min_mcost; 
      }
    }
  }
  else
  {
#ifndef _FAST_FULL_ME_

    //--- set search center ---
    mv_x = pred_mv_x / 4;    //换算成以整数像素为单位的MV
    mv_y = pred_mv_y / 4;
    if (!input->rdopt)
    {
      //--- adjust search center so that the (0,0)-vector is inside ---
      /*使mv_x,mv_y在(-search_range,search_range)之间，
      回忆下配置文件中的RestrictSearchRange选项，可以通过它的设置使不同参考帧
      以及不同大小的块的搜索范围不同，所以，预测MV有可能大于search_range
   */
      mv_x = max (-search_range, min (search_range, mv_x));
      mv_y = max (-search_range, min (search_range, mv_y));
    }
    
    //--- perform motion search ---
    min_mcost = FullPelBlockMotionSearch     (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                              pred_mv_x, pred_mv_y, &mv_x, &mv_y, search_range,
                                              min_mcost, lambda);

#else

    // comments:   - orig_pic is not used  -> be careful
    //             - search center is automatically determined
    min_mcost = FastFullPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                              pred_mv_x, pred_mv_y, &mv_x, &mv_y, search_range,
                                              min_mcost, lambda);

#endif
  }

#ifdef WIN32
      _ftime(&tstruct2);   // end time ms
#else
      ftime(&tstruct2);    // end time ms
#endif
      
      me_tmp_time=(tstruct2.time*1000+tstruct2.millitm) - (tstruct1.time*1000+tstruct1.millitm); 
      me_tot_time += me_tmp_time;
      me_time += me_tmp_time;

  //==============================
  //=====   SUB-PEL SEARCH   =====
  //==============================
  if (input->hadamard)
  {
    min_mcost = max_value;
  }

  if(input->FMEnable)
  {
    if(blocktype >3)
    {
      min_mcost =  FastSubPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                                pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                                                min_mcost, lambda, /*useABT*/0);
    }
    else
    {
      min_mcost =  SubPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                            pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                                            min_mcost, lambda);
    }


    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
        if(list == 0)
        {
          all_mincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][1] = mv_x;
          all_mincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][2] = mv_y;
        }
        else
        {
          all_bwmincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][1] = mv_x;
          all_bwmincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][2] = mv_y;
          
        }
      }
    }
  }
  else  //进行分像素搜索，从这个函数出来之后MV又变成了以1/4像素为单位
  {
    min_mcost =  SubPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                          pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                                          min_mcost, lambda);
  }


  if (!input->rdopt)
  {
    // Get the skip mode cost得到跳跃宏块的代价
    if (blocktype == 1 && (img->type == P_SLICE||img->type == SP_SLICE))
    {
      int cost;

      FindSkipModeMotionVector ();

      cost  = GetSkipCostMB (lambda);
      cost -= (int)floor(8*lambda+0.4999);

      if (cost < min_mcost)
      {
        min_mcost = cost;
        mv_x      = img->all_mv [0][0][0][0][0][0];
        mv_y      = img->all_mv [0][0][0][0][0][1];
      }
    }
  }

  //===============================================
  //=====   SET MV'S AND RETURN MOTION COST   =====
  //===============================================

  if(input->FMEnable)
  {
    int h4x4blkno = (img->pix_x>>2)+block_x;  
    int v4x4blkno = (img->pix_y>>2)+block_y;
    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
        all_mv[block_x+i][block_y+j][list][ref][blocktype][0] = mv_x;
        all_mv[block_x+i][block_y+j][list][ref][blocktype][1] = mv_y;
        mv_array[h4x4blkno+i][v4x4blkno+j][0] = mv_x;
        mv_array[h4x4blkno+i][v4x4blkno+j][1] = mv_y;
      }
    }
  }
  else
  {
    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
        all_mv[block_x+i][block_y+j][list][ref][blocktype][0] = mv_x;
        all_mv[block_x+i][block_y+j][list][ref][blocktype][1] = mv_y;
      }
    }
  }

  return min_mcost; //chj 返回最小代价
}


/*!
 ***********************************************************************
 * \brief
 *    Motion Cost for Bidirectional modes
 ***********************************************************************
 */
int BIDPartitionCost (int   blocktype,
                      int   block8x8,
                      int   fw_ref,
                      int   bw_ref,
                      int   lambda_factor)
{
  static int  bx0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,2,0,2}};
  static int  by0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,0,0,0}, {0,0,2,2}};

  int   diff[16];
  int   pic_pix_x, pic_pix_y, block_x, block_y;
  int   v, h, mcost, i, j, k;
  int   mvd_bits  = 0;
  int   parttype  = (blocktype<4?blocktype:4);
  int   step_h0   = (input->blc_size[ parttype][0]>>2);
  int   step_v0   = (input->blc_size[ parttype][1]>>2);
  int   step_h    = (input->blc_size[blocktype][0]>>2);
  int   step_v    = (input->blc_size[blocktype][1]>>2);
  int   bxx, byy;                               // indexing curr_blk

  int   ******all_mv = img->all_mv;
  int   ******  p_mv = img->pred_mv;

  //----- cost for motion vector bits -----
  for (v=by0[parttype][block8x8]; v<by0[parttype][block8x8]+step_v0; v+=step_v)
  for (h=bx0[parttype][block8x8]; h<bx0[parttype][block8x8]+step_h0; h+=step_h)
  {
    mvd_bits += mvbits[ all_mv [h][v][LIST_0][fw_ref][blocktype][0] - p_mv[h][v][LIST_0][fw_ref][blocktype][0] ];
    mvd_bits += mvbits[ all_mv [h][v][LIST_0][fw_ref][blocktype][1] - p_mv[h][v][LIST_0][fw_ref][blocktype][1] ];

    mvd_bits += mvbits[ all_mv [h][v][LIST_1][bw_ref][blocktype][0] - p_mv[h][v][LIST_1][bw_ref][blocktype][0] ];
    mvd_bits += mvbits[ all_mv [h][v][LIST_1][bw_ref][blocktype][1] - p_mv[h][v][LIST_1][bw_ref][blocktype][1] ];
  }

  mcost = WEIGHTED_COST (lambda_factor, mvd_bits);

  //----- cost of residual signal -----
  for (byy=0, v=by0[parttype][block8x8]; v<by0[parttype][block8x8]+step_v0; byy+=4, v++)
  {
    pic_pix_y = img->opix_y + (block_y = (v<<2));

    for (bxx=0, h=bx0[parttype][block8x8]; h<bx0[parttype][block8x8]+step_h0; bxx+=4, h++)
    {
      pic_pix_x = img->opix_x + (block_x = (h<<2));

      LumaPrediction4x4 (block_x, block_y, 2, blocktype, blocktype, fw_ref, bw_ref);

      for (k=j=0; j<4; j++)
      for (  i=0; i<4; i++, k++)
      {
        diff[k] = imgY_org[pic_pix_y+j][pic_pix_x+i] - img->mpr[i+block_x][j+block_y];
      }
      mcost += SATD (diff, input->hadamard);
    }
  }
  return mcost;
}

/*!
 ************************************************************************
 * \brief
 *    Get cost for skip mode for an macroblock
 ************************************************************************
 */
int GetSkipCostMB (double lambda)
{
  int block_y, block_x, pic_pix_y, pic_pix_x, i, j, k;
  int diff[16];
  int cost = 0;

  for (block_y=0; block_y<16; block_y+=4)
  {
    pic_pix_y = img->opix_y + block_y;

    for (block_x=0; block_x<16; block_x+=4)
    {
      pic_pix_x = img->opix_x + block_x;

      //===== prediction of 4x4 block =====
      LumaPrediction4x4 (block_x, block_y, 0, 0, 0, 0, 0);

      //===== get displaced frame difference ======                
      for (k=j=0; j<4; j++)
        for (i=0; i<4; i++, k++)
        {
          diff[k] = imgY_org[pic_pix_y+j][pic_pix_x+i] - img->mpr[i+block_x][j+block_y];
        }
      cost += SATD (diff, input->hadamard);
    }
  }

  return cost;
}

/*!
 ************************************************************************
 * \brief
 *    Find motion vector for the Skip mode
 ************************************************************************
 */
void FindSkipModeMotionVector ()
{
  int bx, by;
  int ******all_mv = img->all_mv;

  int pmv[2];

  int zeroMotionAbove;
  int zeroMotionLeft;
  PixelPos mb_a, mb_b;
  int      a_mv_y = 0;
  int      a_ref_idx = 0;
  int      b_mv_y = 0;
  int      b_ref_idx = 0;

  Macroblock *currMB = &img->mb_data[img->current_mb_nr];
  
  getLuma4x4Neighbour(img->current_mb_nr,0,0,-1, 0,&mb_a);
  getLuma4x4Neighbour(img->current_mb_nr,0,0, 0,-1,&mb_b);
  
  if (mb_a.available)
  {
    a_mv_y    = enc_picture->mv[LIST_0][mb_a.pos_x][mb_a.pos_y][1];
    a_ref_idx = enc_picture->ref_idx[LIST_0][mb_a.pos_x][mb_a.pos_y];
    //帧场问题 调整MV和ref_idx
    if (currMB->mb_field && !img->mb_data[mb_a.mb_addr].mb_field)
    {
      a_mv_y    /=2;
      a_ref_idx *=2;
    }
    if (!currMB->mb_field && img->mb_data[mb_a.mb_addr].mb_field)
    {
      a_mv_y    *=2;
      a_ref_idx >>=1;
    }
  }
  
  if (mb_b.available)
  {
    b_mv_y    = enc_picture->mv[LIST_0][mb_b.pos_x][mb_b.pos_y][1];
    b_ref_idx = enc_picture->ref_idx[LIST_0][mb_b.pos_x][mb_b.pos_y];
    
    if (currMB->mb_field && !img->mb_data[mb_b.mb_addr].mb_field)
    {
      b_mv_y    /=2;
      b_ref_idx *=2;
    }
    if (!currMB->mb_field && img->mb_data[mb_b.mb_addr].mb_field)
    {
      b_mv_y    *=2;
      b_ref_idx >>=1;
    }
  }
  //a不可用且mv_a为0 zeroMotionLeft=1
  zeroMotionLeft  = !mb_a.available ? 1 : a_ref_idx==0 && enc_picture->mv[LIST_0][mb_a.pos_x][mb_a.pos_y][0]==0 && a_mv_y==0 ? 1 : 0;
  zeroMotionAbove = !mb_b.available ? 1 : b_ref_idx==0 && enc_picture->mv[LIST_0][mb_b.pos_x][mb_b.pos_y][0]==0 && b_mv_y==0 ? 1 : 0;
  
  if (zeroMotionAbove || zeroMotionLeft)
  {
    for (by = 0;by < 4;by++)
      for (bx = 0;bx < 4;bx++)
      {
        all_mv [bx][by][0][0][0][0] = 0;
        all_mv [bx][by][0][0][0][1] = 0;
      }
  }
  else
  {
    SetMotionVectorPredictor (pmv, enc_picture->ref_idx, enc_picture->mv, 0, LIST_0, 0, 0, 16, 16);
    for (by = 0;by < 4;by++)
      for (bx = 0;bx < 4;bx++)
      {
        all_mv [bx][by][0][0][0][0] = pmv[0];
        all_mv [bx][by][0][0][0][1] = pmv[1];
      }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Get cost for direct mode for an 8x8 block  B帧 8x8 block块直接预测模式
 ************************************************************************
 */
int Get_Direct_Cost8x8 (int block, double lambda)
{
  int block_y, block_x, pic_pix_y, pic_pix_x, i, j, k;
  int diff[16];
  int cost  = 0;
  int mb_y  = (block/2)<<3; //chj 把宏块内块坐标变成宏块内像素相对坐标
  int mb_x  = (block%2)<<3;

  for (block_y=mb_y; block_y<mb_y+8; block_y+=4)//chj 对8x8块循环
  {
    pic_pix_y = img->opix_y + block_y;//chj 像素相对图像原点的绝对坐标

    for (block_x=mb_x; block_x<mb_x+8; block_x+=4)
    {
      pic_pix_x = img->opix_x + block_x;//chj 同上

      if (direct_pdir[pic_pix_x>>2][pic_pix_y>>2]<0) //chj if该块direct模式不可用
      {
        return (1<<30); //mode not allowed
      }

      //===== prediction of 4x4 block =====
      LumaPrediction4x4 (block_x, block_y, direct_pdir[pic_pix_x>>2][pic_pix_y>>2], 0, 0, 
                         direct_ref_idx[LIST_0][pic_pix_x>>2][pic_pix_y>>2], 
                         direct_ref_idx[LIST_1][pic_pix_x>>2][pic_pix_y>>2]);
      //chj a 这个函数里面操作全局变量img->mpr，下面要用
      //chj mpr里面存的是参考块的像素
      //===== get displaced frame difference ======                
      for (k=j=0; j<4; j++)
        for (i=0; i<4; i++, k++)
        {
          diff[k] = imgY_org[pic_pix_y+j][pic_pix_x+i] - img->mpr[i+block_x][j+block_y];
        }//chj a mpr->current best prediction mode
      cost += SATD (diff, input->hadamard);
	  //chj a 计算方法:把残差先进行 Hadamard 变换，然后取绝对值求和
    }
  }

  return cost;
}



/*!
 ************************************************************************
 * \brief
 *    Get cost for direct mode for an macroblock 得到B宏块直接模式代价
 ************************************************************************
 */
int Get_Direct_CostMB (double lambda)
{
  int i;
  int cost = 0;
  
  for (i=0; i<4; i++)
  {
    cost += Get_Direct_Cost8x8 (i, lambda);
    if (cost >= (1<<30)) return cost;
  }
  return cost;
}


/*!
 ************************************************************************
 * \brief对每个分块进行运动搜索并计算代价
 *    Motion search for a partition
 ************************************************************************
 */
void
PartitionMotionSearch (int    blocktype,
                                                                    int    block8x8,
                                                                    double lambda)
{
  /*bx0[parttype][block8x8],by0[parttype][block8x8]
   4个分块，所以block8x8=4，4种类型分割:mode 1 2 3 4(parttype!=0)，
   注意mode>4的时候，parttype为4，mode即blocktype
   parttype用4代表所有的亚宏块分割
   bx0[5][4] = {{0,0,0,0}(无用), 
                {0,0,0,0},(mode1-16x16，只用第一个元素,因为无分块) 
                {0,0,0,0},(mode2-16x8，只用前两个元素,因为只有两个分块) 
                {0,2,0,0},(mode3-8x16，只用前两个元素,因为只有两个分块)   
                {0,2,0,2}};(mode4-8x8,四个元素都用，4个分块)
   by0[5][4] = {{0,0,0,0}(无用), 
                {0,0,0,0},(mode1-16x16，只用第一个元素,因为无分块)   
                {0,2,0,0},(mode2-16x8，只用前两个元素,因为只有两个分块)   
                {0,0,0,0},(mode3-8x16，只用前两个元素,因为只有两个分块)    
                {0,0,2,2}};(mode4-8x8,四个元素都用，4个分块)
   bx0,by0是各个模式下分块的起始坐标，坐标的单位是4x4小块，只需要画出分块图，就一目了然
  */
  static int  bx0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,2,0,2}};
  static int  by0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,0,0,0}, {0,0,2,2}};

  int   **ref_array, ***mv_array;
  int   ref, v, h, mcost, search_range, i, j;
  int   pic_block_x, pic_block_y;
  int   bslice    = (img->type==B_SLICE);
  int   parttype  = (blocktype<4?blocktype:4); //chj 最大到4
  int   step_h0   = (input->blc_size[ parttype][0]>>2);/*****************************/
  int   step_v0   = (input->blc_size[ parttype][1]>>2); /*把坐标从像素为单位，变成以 */
  int   step_h    = (input->blc_size[blocktype][0]>>2);/*4x4小块为单位，用来遍历分块*/
  int   step_v    = (input->blc_size[blocktype][1]>>2);/*****************************/
  int   list;
  int   numlists;
  int   list_offset;
  
  if (img->mb_data[img->current_mb_nr].mb_field) //是否为帧场自适应编码 
  {
    if(img->current_mb_nr%2)
      list_offset = 4; // bottom field mb 
    else
      list_offset = 2; // top field mb
  }
  else
  {
    list_offset = 0;  // no mb aff or frame mb
  }

  numlists=bslice?2:1; //如果是bslice就有2个list

  //===== LOOP OVER REFERENCE FRAMES =====
  for (list=0; list<numlists;list++) 
  {
     /*listXsize是什么? listXsize是记录帧和场的参考图像的个数的
     参考图像放在list中，list[0，1]是当前宏块的frame参考序列
     list[2,3]对应当前宏块的list[0,1]的顶场(top field)参考序列
     list[4,5]对应当前宏块的list[0,1]的底场(bottom field)参考序列
  */
    for (ref=0; ref < listXsize[list+list_offset]; ref++)//要遍历所有的参考序列list[0,5]
    {
        //----- set search range ---
        /* 关于search range:
        配置文件中有一项:RestrictSearchRange，即这里的full_search，
        用来控制search range，说明如下:
        如果设置成2，则所有大小的块在任意参考帧上搜索的范围都为search range；
        如果设置成1，则所有大小的块在ref num为0的参考帧(newest)上的搜索范围为search range，
        其余参考帧上为一半；
        如果设置成0，则除了在老参考帧上搜索范围减半之外，在所有非16x16块的搜索上再减半        
    */
#ifdef _FULL_SEARCH_RANGE_
        if      (input->full_search == 2) search_range = input->search_range;
        else if (input->full_search == 1) search_range = input->search_range /  (min(ref,1)+1);
        else    search_range = input->search_range / ((min(ref,1)+1) * min(2,blocktype));
#else
        search_range = input->search_range / ((min(ref,1)+1) * min(2,blocktype));
#endif
        
        //----- set arrays -----
        ref_array = enc_picture->ref_idx[list];//指定参考序列列表
        mv_array  = enc_picture->mv[list];//指定存储MV的列表
        
        //----- init motion cost -----
        /*认真思考motion_cost的下标:1.blocktype 2.list 3.ref 4.block8x8
       每一个宏块对不同的blocktype，list，ref和block8x8的motion_cost全部存下来
    */
        motion_cost[blocktype][list][ref][block8x8] = 0;
        
        //===== LOOP OVER SUB MACRO BLOCK partitions划分   block8x8=4
        for (v=by0[parttype][block8x8]; v<by0[parttype][block8x8]+step_v0; v+=step_v)
        {
          pic_block_y = img->block_y + v;//以4x4块为单位的分块绝对纵坐标
          
          for (h=bx0[parttype][block8x8]; h<bx0[parttype][block8x8]+step_h0; h+=step_h)
          {   //chj parttype:分块类型,block8x8分块个数
            pic_block_x = img->block_x + h;//以4x4块为单位的分块绝对横坐标
            
            //--- motion search for block ---
			//循环执行每个宏块（或亚宏块）
			//分割在各个参考帧中的运动搜索，
			//并计算代价mcost（残差代价 + MV代价，
			//即：SAD + MVD编码所需的bit数）
            mcost = BlockMotionSearch     (ref, list, h<<2, v<<2, blocktype, 
                                           search_range, lambda);
			//motion_cost存储各分块方法在各参考帧的代价
            motion_cost[blocktype][list][ref][block8x8] += mcost;
            
            //--- set motion vectors and reference frame (for motion vector prediction) ---
            /*把all_mv、ref存到enc_picture结构体中，用来计算预测MV*/
            for (j=0; j<step_v; j++)
              for (i=0; i<step_h; i++)
              {
                mv_array  [pic_block_x+i][pic_block_y+j][0] = img->all_mv[h][v][list][ref][blocktype][0];
                mv_array  [pic_block_x+i][pic_block_y+j][1] = img->all_mv[h][v][list][ref][blocktype][1];
                ref_array [pic_block_x+i][pic_block_y+j]    = ref;
              }
          }
        }
    }
  }
}





extern int* last_P_no;
/*********************************************
 *****      B帧 计算直接模式的MV         *****
 *****  Calculate Direct Motion Vectors  *****
 *****   未阅读                          *****
 *********************************************/
void Get_Direct_Motion_Vectors ()
{

  int  block_x, block_y, pic_block_x, pic_block_y, opic_block_x, opic_block_y;
  int  ******all_mvs = img->all_mv;
  int  mv_scale;
  byte **    moving_block;
  int ****   co_located_mv;
  int ***    co_located_ref_idx;
  int64 ***    co_located_ref_id;
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  if ((img->MbaffFrameFlag)&&(currMB->mb_field))
  {
    if(img->current_mb_nr%2)
    {
      moving_block = Co_located->bottom_moving_block;
      co_located_mv = Co_located->bottom_mv;
      co_located_ref_idx = Co_located->bottom_ref_idx;
      co_located_ref_id = Co_located->bottom_ref_pic_id;
    }
    else
    {
      moving_block = Co_located->top_moving_block;
      co_located_mv = Co_located->top_mv;
      co_located_ref_idx = Co_located->top_ref_idx;
      co_located_ref_id = Co_located->top_ref_pic_id;
    }
  }
  else
  {
    moving_block = Co_located->moving_block;
    co_located_mv = Co_located->mv;
    co_located_ref_idx = Co_located->ref_idx;
    co_located_ref_id = Co_located->ref_pic_id;
  }

  if (img->direct_type)  //spatial direct mode copy from decoder
  {
    
    int fw_rFrameL, fw_rFrameU, fw_rFrameUL, fw_rFrameUR;
    int bw_rFrameL, bw_rFrameU, bw_rFrameUL, bw_rFrameUR; 
    int fw_rFrame,bw_rFrame;
    int pmvfw[2]={0,0},pmvbw[2]={0,0};

    PixelPos mb_left, mb_up, mb_upleft, mb_upright;              
    
    getLuma4x4Neighbour(img->current_mb_nr,0,0,-1, 0,&mb_left);
    getLuma4x4Neighbour(img->current_mb_nr,0,0, 0,-1,&mb_up);
    getLuma4x4Neighbour(img->current_mb_nr,0,0,16, -1,&mb_upright);
    getLuma4x4Neighbour(img->current_mb_nr,0,0, -1,-1,&mb_upleft);

    if (!img->MbaffFrameFlag)
    {
      fw_rFrameL = mb_left.available ? enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] : -1;
      fw_rFrameU = mb_up.available ? enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] : -1;
      fw_rFrameUL = mb_upleft.available ? enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] : -1;
      fw_rFrameUR = mb_upright.available ? enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] : fw_rFrameUL;      
      
      bw_rFrameL = mb_left.available ? enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] : -1;
      bw_rFrameU = mb_up.available ? enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] : -1;
      bw_rFrameUL = mb_upleft.available ? enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] : -1;
      bw_rFrameUR = mb_upright.available ? enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] : bw_rFrameUL;      
    }
    else
    {
      if (currMB->mb_field)
      {
        fw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field  || enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] < 0? 
          enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] : 
          enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] * 2: -1;

        fw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] < 0? 
          enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] : 
        enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] * 2: -1;

        fw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] < 0?         
          enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] : 
        enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] *2: -1;      

        fw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] < 0?
          enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] : 
        enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] * 2: fw_rFrameUL;      
        
        bw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] < 0? 
          enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] * 2: -1;

        bw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] < 0? 
          enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] * 2: -1;

        bw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] < 0?         
          enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] *2: -1;      

        bw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] < 0?         
          enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] * 2: bw_rFrameUL;              
      }
      else
      {
        fw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y]  < 0 ?
          enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] >> 1 : 
        enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y]: -1;
        
        fw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] >> 1 :  
        enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] : -1;
        
        fw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y]>> 1 : 
        enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] : -1;      
        
        fw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] < 0 ? 
          enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] >> 1 :  
        enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] : fw_rFrameUL;      
        
        bw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] >> 1 :  
        enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] : -1;
        
        bw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] >> 1 : 
        enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] : -1;
        
        bw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] >> 1 : 
        enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] : -1;      
        
        bw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] >> 1: 
        enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] : bw_rFrameUL;      
      }
    }
    
    fw_rFrame = (fw_rFrameL >= 0 && fw_rFrameU >= 0) ? min(fw_rFrameL,fw_rFrameU): max(fw_rFrameL,fw_rFrameU);
    fw_rFrame = (fw_rFrame >= 0 && fw_rFrameUR >= 0) ? min(fw_rFrame,fw_rFrameUR): max(fw_rFrame,fw_rFrameUR);
    
    bw_rFrame = (bw_rFrameL >= 0 && bw_rFrameU >= 0) ? min(bw_rFrameL,bw_rFrameU): max(bw_rFrameL,bw_rFrameU);
    bw_rFrame = (bw_rFrame >= 0 && bw_rFrameUR >= 0) ? min(bw_rFrame,bw_rFrameUR): max(bw_rFrame,bw_rFrameUR);        
    
    if (fw_rFrame >=0)
      SetMotionVectorPredictor (pmvfw, enc_picture->ref_idx, enc_picture->mv, fw_rFrame, LIST_0, 0, 0, 16, 16);
    
    if (bw_rFrame >=0)
      SetMotionVectorPredictor (pmvbw, enc_picture->ref_idx, enc_picture->mv, bw_rFrame, LIST_1, 0, 0, 16, 16);

    for (block_y=0; block_y<4; block_y++)
    {
      pic_block_y  = (img->pix_y>>2) + block_y;
      opic_block_y = (img->opix_y>>2) + block_y;
      
      for (block_x=0; block_x<4; block_x++)
      {
        pic_block_x  = (img->pix_x>>2) + block_x;
        opic_block_x = (img->opix_x>>2) + block_x;

        if (fw_rFrame >=0)
        {
          if (!fw_rFrame  && !moving_block[opic_block_x][opic_block_y])
          {
            all_mvs [block_x][block_y][LIST_0][0][0][0] = 0;
            all_mvs [block_x][block_y][LIST_0][0][0][1] = 0;            
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y]=0;       
          }
          else
          {
            all_mvs [block_x][block_y][LIST_0][fw_rFrame][0][0] = pmvfw[0];
            all_mvs [block_x][block_y][LIST_0][fw_rFrame][0][1] = pmvfw[1];
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y]=fw_rFrame;              
          }
        }
        else
        {
          all_mvs [block_x][block_y][LIST_0][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_0][0][0][1] = 0;
          direct_ref_idx[LIST_0][pic_block_x][pic_block_y]=-1;          
        }

        if (bw_rFrame >=0)
        {
          if(bw_rFrame==0 && !moving_block[opic_block_x][opic_block_y])
          {                  
            all_mvs [block_x][block_y][LIST_1][0][0][0] = 0;
            all_mvs [block_x][block_y][LIST_1][0][0][1] = 0;
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y]=bw_rFrame;     
          }
          else
          {
            all_mvs [block_x][block_y][LIST_1][bw_rFrame][0][0] = pmvbw[0];
            all_mvs [block_x][block_y][LIST_1][bw_rFrame][0][1] = pmvbw[1];
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y]=bw_rFrame;
          }               
        }
        else
        {      
          direct_ref_idx[LIST_1][pic_block_x][pic_block_y]=-1;

          all_mvs [block_x][block_y][LIST_1][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_1][0][0][1] = 0;
        }
        
        if (fw_rFrame < 0 && bw_rFrame < 0)
        {
          direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = 
          direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = 0;
        }

        if      (direct_ref_idx[LIST_1][pic_block_x][pic_block_y]==-1) direct_pdir[pic_block_x][pic_block_y] = 0;
        else if (direct_ref_idx[LIST_0][pic_block_x][pic_block_y]==-1) direct_pdir[pic_block_x][pic_block_y] = 1;
        else                                                           direct_pdir[pic_block_x][pic_block_y] = 2;
      }
    }
  }
  else
  {
    //temporal direct mode copy from decoder
    for (block_y=0; block_y<4; block_y++)
    {
      pic_block_y  = (img->pix_y>>2) + block_y;
      opic_block_y = (img->opix_y>>2) + block_y;
      
      for (block_x=0; block_x<4; block_x++)
      {
        int refList; 
        int ref_idx; 

        int list_offset = ((img->MbaffFrameFlag)&&(currMB->mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

        pic_block_x  = (img->pix_x>>2) + block_x;
        opic_block_x = (img->opix_x>>2) + block_x;
        
        refList = (co_located_ref_idx[LIST_0][opic_block_x][opic_block_y]== -1 ? LIST_1 : LIST_0);
        ref_idx = co_located_ref_idx[refList][opic_block_x][opic_block_y];
              
        // next P is intra mode
        if (ref_idx==-1)
        {
          all_mvs [block_x][block_y][LIST_0][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_0][0][0][1] = 0;
          all_mvs [block_x][block_y][LIST_1][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_1][0][0][1] = 0;
          direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = 0;
          direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = 0;
          direct_pdir[pic_block_x][pic_block_y] = 2;
        }
        // next P is skip or inter mode
        else 
        {
          int mapped_idx=INVALIDINDEX;
          int iref; 

          {
            for (iref=0;iref<min(img->num_ref_idx_l0_active,listXsize[LIST_0+list_offset]);iref++)
            {
              if (enc_picture->ref_pic_num[LIST_0 +list_offset][iref]==co_located_ref_id[refList ][opic_block_x][opic_block_y])
              {
                mapped_idx=iref;
                break;
              }
              else //! invalid index. Default to zero even though this case should not happen
              {                        
                mapped_idx=INVALIDINDEX;
              }
            }
          }

          if (mapped_idx !=INVALIDINDEX)
          {
            mv_scale = img->mvscale[LIST_0+list_offset][mapped_idx];

            if (mv_scale==9999)
            {
              // forward
              all_mvs [block_x][block_y][LIST_0][0][0][0] = co_located_mv[refList][opic_block_x][opic_block_y][0];
              all_mvs [block_x][block_y][LIST_0][0][0][1] = co_located_mv[refList][opic_block_x][opic_block_y][1];
              // backward
              all_mvs [block_x][block_y][LIST_1][       0][0][0] = 0;
              all_mvs [block_x][block_y][LIST_1][       0][0][1] = 0;
            }else
            {
              // forward
              all_mvs [block_x][block_y][LIST_0][mapped_idx][0][0] = (mv_scale * co_located_mv[refList][opic_block_x][opic_block_y][0] + 128) >> 8;
              all_mvs [block_x][block_y][LIST_0][mapped_idx][0][1] = (mv_scale * co_located_mv[refList][opic_block_x][opic_block_y][1] + 128) >> 8;
              // backward
              all_mvs [block_x][block_y][LIST_1][       0][0][0] = ((mv_scale - 256)* co_located_mv[refList][opic_block_x][opic_block_y][0] + 128) >> 8;
              all_mvs [block_x][block_y][LIST_1][       0][0][1] = ((mv_scale - 256)* co_located_mv[refList][opic_block_x][opic_block_y][1] + 128) >> 8;
            }
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = mapped_idx;
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = 0;
            direct_pdir[pic_block_x][pic_block_y] = 2;
          }
          else
          {
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = -1;
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = -1;
            direct_pdir[pic_block_x][pic_block_y] = -1;
          }
        }
      }
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    control the sign of a with b
 ************************************************************************
 */
int sign(int a,int b)
{
  int x;
  x=absm(a);
  if (b >= 0)
    return x;
  else
    return -x;
}

