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

/*! (CA)VLC 编码函数
 ***************************************************************************
 * \file vlc.c
 *
 * \brief
 *    (CA)VLC coding functions
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Inge Lille-Lang                <inge.lille-langoy@telenor.com>
 *    - Detlev Marpe                    <marpe@hhi.de>
 *    - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 ***************************************************************************
 */

#include "contributors.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "elements.h"
#include "vlc.h"

#if TRACE
#define SYMTRACESTRING(s) strncpy(sym.tracestring,s,TRACESTRING_SIZE)
#else
#define SYMTRACESTRING(s) // do nothing
#endif


/*! 
 *************************************************************************************
 * \brief
 *    ue_v, writes an ue(v) syntax element, returns the length in bits
 *
 * \param tracestring
 *    the string for the trace file
 * \param value
 *    the value to be coded
 *  \param part
 *    the Data Partition the value should be coded into
 *
 * \return
 *    Number of bits used by the coded syntax element
 *
 * \ note
 *    This function writes always the bit buffer for the progressive scan flag, and
 *    should not be used (or should be modified appropriately) for the interlace crap
 *    When used in the context of the Parameter Sets, this is obviously not a
 *    problem.
 *
 *************************************************************************************
 */
 /*无符号整数哥伦布编码*/
/*
*tracing : 用于调试跟踪用的字符串信息
*value: 待编码的数值
*part: 
*/
int ue_v (char *tracestring, int value, DataPartition *part)
{
  SyntaxElement symbol, *sym=&symbol;
  sym->type = SE_HEADER;
  sym->mapping = ue_linfo;               // Mapping rule: unsigned integer
  sym->value1 = value;
  sym->value2 = 0;
#if TRACE
  strncpy(sym->tracestring,tracestring,TRACESTRING_SIZE);
#endif
  assert (part->bitstream->streamBuffer != NULL);
  return writeSyntaxElement_UVLC (sym, part);
}


/*! 
 *************************************************************************************
 * \brief
 *    se_v, writes an se(v) syntax element, returns the length in bits
 *
 * \param tracestring
 *    the string for the trace file
 * \param value
 *    the value to be coded
 *  \param part
 *    the Data Partition the value should be coded into
 *
 * \return
 *    Number of bits used by the coded syntax element
 *
 * \ note
 *    This function writes always the bit buffer for the progressive scan flag, and
 *    should not be used (or should be modified appropriately) for the interlace crap
 *    When used in the context of the Parameter Sets, this is obviously not a
 *    problem.
 *
 *************************************************************************************
 */
 /*有符号整数哥伦布编码*/
int se_v (char *tracestring, int value, DataPartition *part)
{
  SyntaxElement symbol, *sym=&symbol;
  sym->type = SE_HEADER;
  sym->mapping = se_linfo;               // Mapping rule: signed integer
  sym->value1 = value;
  sym->value2 = 0;
#if TRACE
  strncpy(sym->tracestring,tracestring,TRACESTRING_SIZE);
#endif
  assert (part->bitstream->streamBuffer != NULL);
  return writeSyntaxElement_UVLC (sym, part);
} 


/*! 
 *************************************************************************************
 * \brief
 *    u_1, writes a flag (u(1) syntax element, returns the length in bits, 
 *    always 1
 *
 * \param tracestring
 *    the string for the trace file
 * \param value
 *    the value to be coded
 *  \param part
 *    the Data Partition the value should be coded into
 *
 * \return
 *    Number of bits used by the coded syntax element (always 1)
 *
 * \ note
 *    This function writes always the bit buffer for the progressive scan flag, and
 *    should not be used (or should be modified appropriately) for the interlace crap
 *    When used in the context of the Parameter Sets, this is obviously not a
 *    problem.
 *
 *************************************************************************************
 */
 /*1bit无符号字节流*/
int u_1 (char *tracestring, int value, DataPartition *part)
{
  SyntaxElement symbol, *sym=&symbol;

  sym->bitpattern = value;
  sym->len = 1;
  sym->type = SE_HEADER;
  sym->value1 = value;
  sym->value2 = 0;
#if TRACE
  strncpy(sym->tracestring,tracestring,TRACESTRING_SIZE);
#endif
  assert (part->bitstream->streamBuffer != NULL);
  return writeSyntaxElement_fixed(sym, part);
}


/*! 
 *************************************************************************************
 * \brief
 *    u_v, writes a a n bit fixed length syntax element, returns the length in bits, 
 *
 * \param n
 *    length in bits
 * \param tracestring
 *    the string for the trace file
 * \param value
 *    the value to be coded
 *  \param part
 *    the Data Partition the value should be coded into
 *
 * \return
 *    Number of bits used by the coded syntax element 
 *
 * \ note
 *    This function writes always the bit buffer for the progressive scan flag, and
 *    should not be used (or should be modified appropriately) for the interlace crap
 *    When used in the context of the Parameter Sets, this is obviously not a
 *    problem.
 *
 *************************************************************************************
 */
/*u(v),码流中写入n bit 的无符号比特流*/
int u_v (int n, char *tracestring, int value, DataPartition *part)
{
  SyntaxElement symbol, *sym=&symbol;

  sym->bitpattern = value;
  sym->len = n;
  sym->type = SE_HEADER;
  sym->value1 = value;
  sym->value2 = 0;
#if TRACE
  strncpy(sym->tracestring,tracestring,TRACESTRING_SIZE);
#endif
  assert (part->bitstream->streamBuffer != NULL);
  return writeSyntaxElement_fixed(sym, part);
}


/*!
 ************************************************************************
 * \brief  无符号指数哥伦布编码
 *    mapping for ue(v) syntax elements
 * \param ue
 *    value to be mapped
 * \param dummy
 *    dummy parameter
 * \param info
 *    returns mapped value
 * \param len
 *    returns mapped value length
 ************************************************************************
 *
 /*k阶指数哥伦布编码
 *每个编码后的码字结构: [M zeros][1][INFO]
*用来表示非负整数的k阶指数哥伦布码可用如下步骤生成：
*(1)将数字以二进制形式写出，去掉最低的k个比特位，之后加1
*(2)计算留下的比特数，将此数减一，即是需要增加的前导零个数
*(3)将第一步中去掉的最低k个比特位补回比特串尾部
* h.264使用的是0阶的指数哥伦布编码,码表如下:
*    code_num       codeword
*        0              1
*        1             010
*        2             011
*        3            00100
*        4            00101
*        5            00110
*        6            00111
*        7           0001000
*        8           0001001
*      ......	     ......
*
*     ue(v): Unsigned direct mapping: code_num = v.
*   
*/        
 /*将需要编码的无符号整数按照无符号指数哥伦布编码
*请参考标准附录。
*
*/
void ue_linfo(int ue, int dummy, int *len,int *info)
{
  int i,nn;

  nn=(ue+1)/2;

  for (i=0; i < 16 && nn != 0; i++)
  {
    nn /= 2;
  }
  *len= 2*i + 1;//哥伦布码码字的长度
  *info=ue+1-(int)pow(2,i);// 码字的内容
}


/*!
 ************************************************************************
 * \brief  有符号整数指数哥伦布编码
 *    mapping for se(v) syntax elements
 * \param se
 *    value to be mapped
 * \param dummy
 *    dummy argument
 * \param len
 *    returns mapped value length
 * \param info
 *    returns mapped value
 ************************************************************************
 */
 /*se(v): Signed mapping.在编码mvd(motion vector difference),delta QP等语法元素时会用到。
 * code_num = 2|v| (v<0)
 * code_num = 2|v|-1 (v>0)
 *
 *     v       code_num
 *     0           0
 *     1           1
 *     -1          2
 *     2           3
 *     -2          4
 *     3           5
 *     ...         ...
*/

void se_linfo(int se, int dummy, int *len,int *info)
{

  int i,n,sign,nn;

  sign=0;

  if (se <= 0)
  {
    sign=1;
  }
  n=abs(se) << 1;

  /*
  n+1 is the number in the code table.  Based on this we find length and info
  */

  nn=n/2;
  for (i=0; i < 16 && nn != 0; i++)
  {
    nn /= 2;
  }
  *len=i*2 + 1;
  *info=n - (int)pow(2,i) + sign;
}


/*!
 ************************************************************************
 * \par Input: 帧内
 *    Number in the code table
 * \par Output:
 *    length and info
 ************************************************************************
 */
 /* cpb: coded_block_pattern, 当前编码宏块不是使用16*16的帧内预测模式时会使用到该语法元素。
 * 该语法元素包含了当前宏块的亮度以及色度分量的预测方式。
 * 其中:CodedBlockPatternLuma = coded_block_pattern % 16
 *     CodedBlockPatternChroma = coded_block_pattern / 16
 * 由于该语法元素使用映射的指数哥伦布编码，所以需要一个映射表。
*/
 /*
 *编码intra预测模式的cbp信息
 *
*/
void cbp_linfo_intra(int cbp, int dummy, int *len,int *info)
{
  extern const int NCBP[48][2];//me(v)使用的映射表。
  ue_linfo(NCBP[cbp][0], dummy, len, info);
}


/*!
 ************************************************************************
 * \par Input: 帧间
 *    Number in the code table
 * \par Output:
 *    length and info
 ************************************************************************
 */
void cbp_linfo_inter(int cbp, int dummy, int *len,int *info)
{
  extern const int NCBP[48][2];
  ue_linfo(NCBP[cbp][1], dummy, len, info);
}


/*!
 ************************************************************************
 * \brief
 *    2x2 transform of chroma DC
 * \par Input:
 *    level and run for coefficients
 * \par Output:
 *    length and info
 * \note
 *    see ITU document for bit assignment
 ************************************************************************
 */
 /*用来编码2*2 色度变换块的DC分量的(level run)对。
 *根据(level run)查表获取code_number
 *在进行指数哥伦布编码?
 *
 *
*/
void levrun_linfo_c2x2(int level,int run,int *len,int *info)
{//早期使用的方案，现在已被CAVLC代替
  const int NTAB[2][2]=
  {
    {1,5},
    {3,0}
  };
  const int LEVRUN[4]=
  {
    2,1,0,0
  };

  int levabs,i,n,sign,nn;

  if (level == 0) //  check if the coefficient sign EOB (level=0)
  {
    *len=1;
    return;
  }
  sign=0;
  if (level <= 0)
  {
    sign=1;
  }
  //根据levabs 跟 run值，查表获取code_number.
  /*这里注意是码表的构造分成两种不同方式:
  *(1). 对于一定范围内的(level, run)根据统计信息，构造到code_number的映射关系.
  *(2). 对于超出范围的(level,run),由简单的计算公式来获取code_number.
  * 
  * 在这个码表中，范围由LEVRUN数组描述。
  * 映射关系由NTAB表描述。
  * 简单计算规则由公式(levabs-LEVRUN[run])*8 + run*2确定。
  * 注意映射关系跟简单公式的结果之间不相互重合。
  */
  levabs=abs(level);
  if (levabs <= LEVRUN[run])
  {
    n=NTAB[levabs-1][run]+1;
  }
  else
  {
    n=(levabs-LEVRUN[run])*8 + run*2;
  }

  // 这里将码字进行哥伦布编码。这里进行了符号映射。
  nn=n/2;

  for (i=0; i < 16 && nn != 0; i++)
  {
    nn /= 2;
  }
  *len= 2*i + 1;
  *info=n-(int)pow(2,i)+sign;
}


/*!
 ************************************************************************
 * \brief
 *    Single scan coefficients
 * \par Input:
 *    level and run for coefficiets
 * \par Output:
 *    lenght and info
 * \note
 *    see ITU document for bit assignment
 ************************************************************************
 */

 /*用来编码inter方式的(level run)对。
 *根据(level run)查表获取code_number
 *在进行指数哥伦布编码?
*/
void levrun_linfo_inter(int level,int run,int *len,int *info)
{//早期使用的方案，现在已被CAVLC代替
  const byte LEVRUN[16]=
  {
    4,2,2,1,1,1,1,1,1,1,0,0,0,0,0,0
  };
  const byte NTAB[4][10]=
  {
    { 1, 3, 5, 9,11,13,21,23,25,27},
    { 7,17,19, 0, 0, 0, 0, 0, 0, 0},
    {15, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {29, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  };

  int levabs,i,n,sign,nn;

  if (level == 0)           //  check for EOB
  {
    *len=1;
    return;
  }

  if (level <= 0)
    sign=1;
  else
    sign=0;

  levabs=abs(level);
  if (levabs <= LEVRUN[run])
  {
    n=NTAB[levabs-1][run]+1;
  }
  else
  {
    n=(levabs-LEVRUN[run])*32 + run*2;
  }

  nn=n/2;

  for (i=0; i < 16 && nn != 0; i++)
  {
    nn /= 2;
  }
  *len= 2*i + 1;
  *info=n-(int)pow(2,i)+sign;

}


/*!
 ************************************************************************
 * \brief
 *    Double scan coefficients
 * \par Input:
 *    level and run for coefficiets
 * \par Output:
 *    lenght and info
 * \note
 *    see ITU document for bit assignment
 ************************************************************************
 */
  /*用来编码intra方式的(level run)对。
 *根据(level run)查表获取code_number
 *再进行指数哥伦布编码
*///早期使用的方案，现在已被CAVLC代替
void levrun_linfo_intra(int level,int run,int *len,int *info)
{
  const byte LEVRUN[8]=
  {
    9,3,1,1,1,0,0,0
  };

  const byte NTAB[9][5] =
  {
    { 1, 3, 7,15,17},
    { 5,19, 0, 0, 0},
    { 9,21, 0, 0, 0},
    {11, 0, 0, 0, 0},
    {13, 0, 0, 0, 0},
    {23, 0, 0, 0, 0},
    {25, 0, 0, 0, 0},
    {27, 0, 0, 0, 0},
    {29, 0, 0, 0, 0},
  };

  int levabs,i,n,sign,nn;

  if (level == 0)     //  check for EOB
  {
    *len=1;
    return;
  }
  if (level <= 0)
    sign=1;
  else
    sign=0;

  levabs=abs(level);
  if (levabs <= LEVRUN[run])
  {
    n=NTAB[levabs-1][run]+1;
  }
  else
  {
    n=(levabs-LEVRUN[run])*16 + 16 + run*2;
  }

  nn=n/2;

  for (i=0; i < 16 && nn != 0; i++)
  {
    nn /= 2;
  }
  *len= 2*i + 1;
  *info=n-(int)pow(2,i)+sign;
}


/*!
 ************************************************************************
 * \brief
 *    Makes code word and passes it back
 *    A code word has the following format: 0 0 0 ... 1 Xn ...X2 X1 X0.
 *
 * \par Input:
 *    Info   : Xn..X2 X1 X0                                             \n
 *    Length : Total number of bits in the codeword
 ************************************************************************
 */
 // NOTE this function is called with sym->inf > (1<<(sym->len/2)).  The upper bits of inf are junk
 /*根据SyntaxElement中的信息构造出码字
   码字保存在sym->bitpattern中
   计算语法元素UVLC编码比特形式，并保存在成员变量bitmap中
*/
int symbol2uvlc(SyntaxElement *sym)
{
  int suffix_len=sym->len/2;  
  sym->bitpattern = (1<<suffix_len)|(sym->inf&((1<<suffix_len)-1));
  return 0;
}


/*!
 ************************************************************************
 * \brief
 *    generates UVLC code and passes the codeword to the buffer
 ************************************************************************
 */
int writeSyntaxElement_UVLC(SyntaxElement *se, DataPartition *this_dataPart)
{
  //根据se的mapping方式计算len，和inf
  se->mapping(se->value1,se->value2,&(se->len),&(se->inf));
  //计算bitpattern
  symbol2uvlc(se);
 //将编码的bitpattern写入码流中
  writeUVLC2buffer(se, this_dataPart->bitstream);

  if(se->type != SE_HEADER)
    this_dataPart->bitstream->write_flag = 1;

#if TRACE//给TRACE文件写入
  if(se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    passes the fixed codeword to the buffer
 ************************************************************************
 */
int writeSyntaxElement_fixed(SyntaxElement *se, DataPartition *this_dataPart)
{  
  writeUVLC2buffer(se, this_dataPart->bitstream);
  
  if(se->type != SE_HEADER)
    this_dataPart->bitstream->write_flag = 1;

#if TRACE
  if(se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    generates code and passes the codeword to the buffer
 ************************************************************************
 */
 /*将Intra4x4PredictionMode语法元素写入码流*/
int writeSyntaxElement_Intra4x4PredictionMode(SyntaxElement *se, DataPartition *this_dataPart)
{//对帧内4*4子宏块预测模式的预测的编码

 // 这里是同时编码两个语法元素:prev_intra4x4_pred_mode_flag 跟rem_intra4x4_pred_mode
 // se->value1 = -1 对应着prev_intra4x4_pred_mode_flag =1 的情况。
 // 其它情况下value1保存的是语法元素rem_intra4x4_pred_mode的值。
 // 两个语法元素的意义请参照标准。
  if (se->value1 == -1)
  {
    se->len = 1;
    se->inf = 1; 
  }
  else 
  {
    se->len = 4;  
    se->inf = se->value1;
  }

  se->bitpattern = se->inf;
  writeUVLC2buffer(se, this_dataPart->bitstream);

  if(se->type != SE_HEADER)
    this_dataPart->bitstream->write_flag = 1;

#if TRACE
  if(se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    generates UVLC code and passes the codeword to the buffer
 * \author
 *  Tian Dong
 ************************************************************************
 */
int writeSyntaxElement2Buf_UVLC(SyntaxElement *se, Bitstream* this_streamBuffer )
{
   //计算inf
  se->mapping(se->value1,se->value2,&(se->len),&(se->inf));
  //计算bitpattern
  symbol2uvlc(se);
  //这里将产生的uvlc码字写入buffer。
  writeUVLC2buffer(se, this_streamBuffer );

#if TRACE
  if(se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    writes UVLC code to the appropriate buffer
 ************************************************************************
 */
void  writeUVLC2buffer(SyntaxElement *se, Bitstream *currStream)
{

  int i;
  unsigned int mask = 1 << (se->len-1);

  // Add the new bits to the bitstream.
  // Write out a byte if it is full
  for (i=0; i<se->len; i++)
  {
    currStream->byte_buf <<= 1;
    if (se->bitpattern & mask)
      currStream->byte_buf |= 1;
    currStream->bits_to_go--;
    mask >>= 1;
    if (currStream->bits_to_go==0)
    {
      currStream->bits_to_go = 8;
      currStream->streamBuffer[currStream->byte_pos++]=currStream->byte_buf;
      currStream->byte_buf = 0;
    }
  }
}


/*!
 ************************************************************************
 * \brief
 *    generates UVLC code and passes the codeword to the buffer
 * \author
 *  Tian Dong
 ************************************************************************
 */
int writeSyntaxElement2Buf_Fixed(SyntaxElement *se, Bitstream* this_streamBuffer )
{
  //这里直接将语法元素的bitpattern写入码流buffer中
  writeUVLC2buffer(se, this_streamBuffer );

#if TRACE
  if(se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    Makes code word and passes it back
 *与uvlc的区别是不包含前缀suffix
 * \par Input:
 *    Info   : Xn..X2 X1 X0                                             \n
 *    Length : Total number of bits in the codeword  
 ************************************************************************
 */

int symbol2vlc(SyntaxElement *sym)
{
  int info_len = sym->len;

  // Convert info into a bitpattern int
  sym->bitpattern = 0;

  // vlc coding
  while(--info_len >= 0)
  {
    sym->bitpattern <<= 1;
    sym->bitpattern |= (0x01 & (sym->inf >> info_len));
  }
  return 0;
}


/*!
 ************************************************************************
 * \brief 不包含前缀
 *    generates VLC code and passes the codeword to the buffer
 ************************************************************************
 */
int writeSyntaxElement_VLC(SyntaxElement *se, DataPartition *this_dataPart)
{

  se->inf = se->value1;
  se->len = se->value2;
  //计算vlc编码
  symbol2vlc(se);
  //写入码流中
  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *     NumCoeff 和 TrailingOnes编码
 ************************************************************************
 */
int writeSyntaxElement_NumCoeffTrailingOnes(SyntaxElement *se, DataPartition *this_dataPart)
{
  int lentab[3][4][17] = //chj 标准中5个表:4变长1定长，其中luma使用3个表
  {//得到的编码长度  [N][TrailingOnes][TotalCoeff]
    {   // 0702
      { 1, 6, 8, 9,10,11,13,13,13,14,14,15,15,16,16,16,16},
      { 0, 2, 6, 8, 9,10,11,13,13,14,14,15,15,15,16,16,16},
      { 0, 0, 3, 7, 8, 9,10,11,13,13,14,14,15,15,16,16,16},
      { 0, 0, 0, 5, 6, 7, 8, 9,10,11,13,14,14,15,15,16,16},
    },                                                 
    {                                                  
      { 2, 6, 6, 7, 8, 8, 9,11,11,12,12,12,13,13,13,14,14},
      { 0, 2, 5, 6, 6, 7, 8, 9,11,11,12,12,13,13,14,14,14},
      { 0, 0, 3, 6, 6, 7, 8, 9,11,11,12,12,13,13,13,14,14},
      { 0, 0, 0, 4, 4, 5, 6, 6, 7, 9,11,11,12,13,13,13,14},
    },                                                 
    {                                                  
      { 4, 6, 6, 6, 7, 7, 7, 7, 8, 8, 9, 9, 9,10,10,10,10},
      { 0, 4, 5, 5, 5, 5, 6, 6, 7, 8, 8, 9, 9, 9,10,10,10},
      { 0, 0, 4, 5, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9,10,10,10},
      { 0, 0, 0, 4, 4, 4, 4, 4, 5, 6, 7, 8, 8, 9,10,10,10},
    },

  };

  int codtab[3][4][17] = //chj 真正查询的码流表
  {//码流的二进制表示
    {
      { 1, 5, 7, 7, 7, 7,15,11, 8,15,11,15,11,15,11, 7,4}, 
      { 0, 1, 4, 6, 6, 6, 6,14,10,14,10,14,10, 1,14,10,6}, 
      { 0, 0, 1, 5, 5, 5, 5, 5,13, 9,13, 9,13, 9,13, 9,5}, 
      { 0, 0, 0, 3, 3, 4, 4, 4, 4, 4,12,12, 8,12, 8,12,8},
    },
    {
      { 3,11, 7, 7, 7, 4, 7,15,11,15,11, 8,15,11, 7, 9,7}, 
      { 0, 2, 7,10, 6, 6, 6, 6,14,10,14,10,14,10,11, 8,6}, 
      { 0, 0, 3, 9, 5, 5, 5, 5,13, 9,13, 9,13, 9, 6,10,5}, 
      { 0, 0, 0, 5, 4, 6, 8, 4, 4, 4,12, 8,12,12, 8, 1,4},
    },
    {
      {15,15,11, 8,15,11, 9, 8,15,11,15,11, 8,13, 9, 5,1}, 
      { 0,14,15,12,10, 8,14,10,14,14,10,14,10, 7,12, 8,4},
      { 0, 0,13,14,11, 9,13, 9,13,10,13, 9,13, 9,11, 7,3},
      { 0, 0, 0,12,11,10, 9, 8,13,12,12,12, 8,12,10, 6,2},
    },
  };
  int vlcnum;  //chj NC值

 // 注意这里se->len用来传递vlcnum，len的特殊用法。
  vlcnum = se->len;

  // se->value1 : numcoeff
  // se->value2 : numtrailingones

  // 对于超出码表范围的数值，使用公式计算。
  if (vlcnum == 3)
  {
    se->len = 6;  // 4 + 2 bit FLC
    if (se->value1 > 0)
    {
      se->inf = ((se->value1-1) << 2) | se->value2;
    }
    else
    {
      se->inf = 3;
    }
  }
  else
  {//未超出范围 查表操作
    se->len = lentab[vlcnum][se->value2][se->value1];
    se->inf = codtab[vlcnum][se->value2][se->value1];
  }
  //se->inf = 0;

  if (se->len == 0)
  {
    printf("ERROR: (numcoeff,trailingones) not valid: vlc=%d (%d, %d)\n", 
      vlcnum, se->value1, se->value2);
    exit(-1);
  }

  symbol2vlc(se);

  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    色度DC分量非零残差系数个数和拖尾系数个数的编码
 ************************************************************************
 */
int writeSyntaxElement_NumCoeffTrailingOnesChromaDC(SyntaxElement *se, DataPartition *this_dataPart)
{
  int lentab[4][5] = 
  {
    { 2, 6, 6, 6, 6,},          
    { 0, 1, 6, 7, 8,}, 
    { 0, 0, 3, 7, 8,}, 
    { 0, 0, 0, 6, 7,},
  };

  int codtab[4][5] = 
  {
    {1,7,4,3,2},
    {0,1,6,3,3},
    {0,0,1,2,2},
    {0,0,0,5,0},
  };

  // se->value1 : numcoeff
  // se->value2 : numtrailingones
  se->len = lentab[se->value2][se->value1];
  se->inf = codtab[se->value2][se->value1];

  if (se->len == 0)
  {
    printf("ERROR: (numcoeff,trailingones) not valid: (%d, %d)\n", 
      se->value1, se->value2);
    exit(-1);
  }

  symbol2vlc(se);

  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    write VLC for TotalZeros  编码左后一个非零残差系数之前零系数的总数
 ************************************************************************
 */
int writeSyntaxElement_TotalZeros(SyntaxElement *se, DataPartition *this_dataPart)
{
  int lentab[TOTRUN_NUM][16] = 
  {
    { 1,3,3,4,4,5,5,6,6,7,7,8,8,9,9,9},  
    { 3,3,3,3,3,4,4,4,4,5,5,6,6,6,6},  
    { 4,3,3,3,4,4,3,3,4,5,5,6,5,6},  
    { 5,3,4,4,3,3,3,4,3,4,5,5,5},  
    { 4,4,4,3,3,3,3,3,4,5,4,5},  
    { 6,5,3,3,3,3,3,3,4,3,6},  
    { 6,5,3,3,3,2,3,4,3,6},  
    { 6,4,5,3,2,2,3,3,6},  
    { 6,6,4,2,2,3,2,5},  
    { 5,5,3,2,2,2,4},  
    { 4,4,3,3,1,3},  
    { 4,4,2,1,3},  
    { 3,3,1,2},  
    { 2,2,1},  
    { 1,1},  
  };

  int codtab[TOTRUN_NUM][16] = 
  {
    {1,3,2,3,2,3,2,3,2,3,2,3,2,3,2,1},
    {7,6,5,4,3,5,4,3,2,3,2,3,2,1,0},
    {5,7,6,5,4,3,4,3,2,3,2,1,1,0},
    {3,7,5,4,6,5,4,3,3,2,2,1,0},
    {5,4,3,7,6,5,4,3,2,1,1,0},
    {1,1,7,6,5,4,3,2,1,1,0},
    {1,1,5,4,3,3,2,1,1,0},
    {1,1,1,3,3,2,2,1,0},
    {1,0,1,3,2,1,1,1,},
    {1,0,1,3,2,1,1,},
    {0,1,1,2,1,3},
    {0,1,1,1,1},
    {0,1,1,1},
    {0,1,1},
    {0,1},  
  };
  int vlcnum;

  vlcnum = se->len;//TotalCoeff

  // se->value1 : TotalZeros
  se->len = lentab[vlcnum][se->value1];
  se->inf = codtab[vlcnum][se->value1];

  if (se->len == 0)
  {
    printf("ERROR: (TotalZeros) not valid: (%d)\n",se->value1);
    exit(-1);
  }

  symbol2vlc(se);

  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    write VLC for TotalZeros for Chroma DC
 ************************************************************************
 */
int writeSyntaxElement_TotalZerosChromaDC(SyntaxElement *se, DataPartition *this_dataPart)
{
  int lentab[3][4] = 
  {
    { 1, 2, 3, 3,},
    { 1, 2, 2, 0,},
    { 1, 1, 0, 0,}, 
  };

  int codtab[3][4] = 
  {
    { 1, 1, 1, 0,},
    { 1, 1, 0, 0,},
    { 1, 0, 0, 0,},
  };
  int vlcnum;//TotalCoeff

  vlcnum = se->len;

  // se->value1 : TotalZeros
  se->len = lentab[vlcnum][se->value1];
  se->inf = codtab[vlcnum][se->value1];

  if (se->len == 0)
  {
    printf("ERROR: (TotalZeros) not valid: (%d)\n",se->value1);
    exit(-1);
  }

  symbol2vlc(se);

  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief 编码zerosleft和runbefore
 *    write VLC for Run Before Next Coefficient, VLC0
 ************************************************************************
 */
int writeSyntaxElement_Run(SyntaxElement *se, DataPartition *this_dataPart)
{
  int lentab[TOTRUN_NUM][16] = 
  {
    {1,1},
    {1,2,2},
    {2,2,2,2},
    {2,2,2,3,3},
    {2,2,3,3,3,3},
    {2,3,3,3,3,3,3},
    {3,3,3,3,3,3,3,4,5,6,7,8,9,10,11},
  };

  int codtab[TOTRUN_NUM][16] = 
  {
    {1,0},
    {1,1,0},
    {3,2,1,0},
    {3,2,1,1,0},
    {3,2,3,2,1,0},
    {3,0,1,3,2,5,4},
    {7,6,5,4,3,2,1,1,1,1,1,1,1,1,1},
  };
  int vlcnum;//zerosleft

  vlcnum = se->len;

  // se->value1 : runbefore
  se->len = lentab[vlcnum][se->value1];
  se->inf = codtab[vlcnum][se->value1];

  if (se->len == 0)
  {
    printf("ERROR: (run) not valid: (%d)\n",se->value1);
    exit(-1);
  }

  symbol2vlc(se);

  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    write VLC for Coeff Level (VLC1)
 ************************************************************************
 */
int writeSyntaxElement_Level_VLC1(SyntaxElement *se, DataPartition *this_dataPart)
{
  int level, levabs, sign;

  level = se->value1;//level
  levabs = abs(level);
  sign = (level < 0 ? 1 : 0);
  // 这里，对level的编码分成了三级
  // 对不同范围内的码字采用不同的编码方法
  if (levabs < 8)
  {
    se->len = levabs * 2 + sign - 1;
    se->inf = 1;
  }
  else if (levabs < 8+8)
  {
    // escape code1
    se->len = 14 + 1 + 4;
    se->inf = (1 << 4) | ((levabs - 8) << 1) | sign;
  }
  else
  {
    // escape code2
    se->len = 14 + 2 + 12;
    se->inf = (0x1 << 12) | ((levabs - 16)<< 1) | sign;
  }


  symbol2vlc(se);

  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    write VLC for Coeff Level
 ************************************************************************
 */
int writeSyntaxElement_Level_VLCN(SyntaxElement *se, int vlc, DataPartition *this_dataPart)
{
  int iCodeword;
  int iLength;

  int level = se->value1;

  int levabs = abs(level);
  int sign = (level < 0 ? 1 : 0);  

  int shift = vlc-1;//suffixLength
  int escape = (15<<shift)+1;

  int numPrefix = (levabs-1)>>shift;

  int sufmask = ~((0xffffffff)<<shift);
  int suffix = (levabs-1)&sufmask;
  // 这里采用了2级编码方式
  if (levabs < escape)
  {
    iLength = numPrefix + vlc + 1;
    iCodeword = (1<<(shift+1))|(suffix<<1)|sign;
  }
  else
  {
    iLength = 28;
    iCodeword = (1<<12)|((levabs-escape)<<1)|sign;
  }
  se->len = iLength;
  se->inf = iCodeword;

  symbol2vlc(se);

  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    Write out a trace string on the trace file
 ************************************************************************
 */
#if TRACE
void trace2out(SyntaxElement *sym)
{
  static int bitcounter = 0;
  int i, chars;

  if (p_trace != NULL)
  {
    putc('@', p_trace);
    chars = fprintf(p_trace, "%i", bitcounter);
    while(chars++ < 6)
      putc(' ',p_trace);

    chars += fprintf(p_trace, "%s", sym->tracestring);
    while(chars++ < 55)
      putc(' ',p_trace);

  // Align bitpattern
    if(sym->len<15)
    {
      for(i=0 ; i<15-sym->len ; i++)
        fputc(' ', p_trace);
    }
    
    // Print bitpattern
    bitcounter += sym->len;
    for(i=1 ; i<=sym->len ; i++)
    {
      if((sym->bitpattern >> (sym->len-i)) & 0x1)
        fputc('1', p_trace);
      else
        fputc('0', p_trace);
    }
    fprintf(p_trace, " (%3d) \n",sym->value1);
  }
  fflush (p_trace);
}
#endif


/*!
 ************************************************************************
 * \brief
 *    puts the less than 8 bits in the byte buffer of the Bitstream into
 *    the streamBuffer.  
 *
 * \param
 *   currStream: the Bitstream the alignment should be established
 *
 ************************************************************************
 */
void writeVlcByteAlign(Bitstream* currStream)
{
  if (currStream->bits_to_go < 8)
  { // trailing bits to process
    currStream->byte_buf = (currStream->byte_buf <<currStream->bits_to_go) | (0xff >> (8 - currStream->bits_to_go));
    stat->bit_use_stuffingBits[img->type]+=currStream->bits_to_go;
    currStream->streamBuffer[currStream->byte_pos++]=currStream->byte_buf;
    currStream->bits_to_go = 8;
  }
}
