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
 * \file context_ini.c
 *
 * \brief
 *    CABAC context initializations
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Detlev Marpe                    <marpe@hhi.de>
 *    - Heiko Schwarz                   <hschwarz@hhi.de>
 **************************************************************************************
 */

#define CONTEXT_INI_C

#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <string.h>

#include "defines.h"
#include "global.h"


#include "ctx_tables.h"
#include "cabac.h"

#define DEFAULT_CTX_MODEL   0
#define RELIABLE_COUNT      32.0
#define FRAME_TYPES         4
#define FIXED               0


int                     num_mb_per_slice;
int                     number_of_slices;
int***                  initialized;
int***                  model_number;


double entropy    [128];
double probability[128] =
{
  0.000000, 0.000000, 0.000000, 0.000000,    0.000000, 0.000000, 0.000000, 0.000000,
  0.000000, 0.000000, 0.000000, 0.000000,    0.000000, 0.000000, 0.000000, 0.000000,
  0.000000, 0.000000, 0.000000, 0.000000,    0.000000, 0.000000, 0.000000, 0.000000,
  0.000000, 0.000000, 0.000000, 0.000000,    0.000000, 0.000000, 0.000000, 0.000000,
  0.000000, 0.000000, 0.000000, 0.000000,    0.000000, 0.000000, 0.000000, 0.000000,
  0.000000, 0.000000, 0.000000, 0.000000,    0.000000, 0.000000, 0.000000, 0.000000,
  0.000000, 0.000000, 0.000000, 0.000000,    0.000000, 0.000000, 0.000000, 0.000000,
  0.000000, 0.000000, 0.000000, 0.000000,    0.000000, 0.000000, 0.000000, 0.000000,
  //--------------------------------------------------------------------------------
  0.500000, 0.474609, 0.450507, 0.427629,    0.405912, 0.385299, 0.365732, 0.347159,
  0.329530, 0.312795, 0.296911, 0.281833,    0.267520, 0.253935, 0.241039, 0.228799,
  0.217180, 0.206151, 0.195682, 0.185744,    0.176312, 0.167358, 0.158859, 0.150792,
  0.143134, 0.135866, 0.128966, 0.122417,    0.116200, 0.110299, 0.104698, 0.099381,
  0.094334, 0.089543, 0.084996, 0.080680,    0.076583, 0.072694, 0.069002, 0.065498,
  0.062172, 0.059014, 0.056018, 0.053173,    0.050473, 0.047909, 0.045476, 0.043167,
  0.040975, 0.038894, 0.036919, 0.035044,    0.033264, 0.031575, 0.029972, 0.028450,
  0.027005, 0.025633, 0.024332, 0.023096,    0.021923, 0.020810, 0.019753, 0.018750
};



void create_context_memory ()
{
  int i, j, k;
  int num_mb    = img->FrameSizeInMbs; // number of macroblocks for frame
 
  num_mb_per_slice  = (input->slice_mode==1 ? input->slice_argument : num_mb);
  number_of_slices  = (num_mb + num_mb_per_slice - 1) / num_mb_per_slice;

  if ((initialized  = (int***) malloc (2 * sizeof(int**))) == NULL)
  {
    no_mem_exit ("create_context_memory: initialized");
  }
  if ((model_number = (int***) malloc (2 * sizeof(int**))) == NULL)
  {
    no_mem_exit ("create_context_memory: model_number");
  }

  for (k=0; k<2; k++)
  {
    if ((initialized[k] = (int**) malloc (FRAME_TYPES * sizeof(int*))) == NULL)
    {
      no_mem_exit ("create_context_memory: initialized");
    }
    if ((model_number[k]= (int**) malloc (FRAME_TYPES * sizeof(int*))) == NULL)
    {
      no_mem_exit ("create_context_memory: model_number");
    }

    for (i=0; i<FRAME_TYPES; i++)
    {
      if ((initialized[k][i] = (int*) malloc (number_of_slices * sizeof(int))) == NULL)
      {
        no_mem_exit ("create_context_memory: initialized");
      }
      if ((model_number[k][i]= (int*) malloc (number_of_slices * sizeof(int))) == NULL)
      {
        no_mem_exit ("create_context_memory: model_number");
      }
    }
  }

  //===== set all context sets as "uninitialized" =====
  for (k=0; k<2; k++)
  {
    for (i=0; i<FRAME_TYPES; i++)
    {
      for (j=0; j<number_of_slices; j++)
      {
        initialized[k][i][j] = 0;
      }
    }
  }

  //----- init tables -----
  for( k=0; k<64; k++ )
  {
    probability[k] = 1.0 - probability[127-k];
    entropy    [k] = log10(probability[    k])/log10(2.0);
    entropy[127-k] = log10(probability[127-k])/log10(2.0);
  }
}




void free_context_memory ()
{
  int i, k;

  for (k=0; k<2; k++)
  {
    for (i=0; i<FRAME_TYPES; i++)
    {
      free (initialized [k][i]);
      free (model_number[k][i]);
    }
    free (initialized [k]);
    free (model_number[k]);
  }
  free (initialized);
  free (model_number);
}






#define BIARI_CTX_INIT2(ii,jj,ctx,tab,num) \
{ \
  for (i=0; i<ii; i++) \
  for (j=0; j<jj; j++) \
  { \
    if      (img->type==I_SLICE)  biari_init_context (&(ctx[i][j]), &(tab ## _I[num][i][j][0])); \
    else                            biari_init_context (&(ctx[i][j]), &(tab ## _P[num][i][j][0])); \
  } \
}
#define BIARI_CTX_INIT1(jj,ctx,tab,num) \
{ \
  for (j=0; j<jj; j++) \
  { \
    if      (img->type==I_SLICE)  biari_init_context (&(ctx[j]), &(tab ## _I[num][0][j][0])); \
    else                            biari_init_context (&(ctx[j]), &(tab ## _P[num][0][j][0])); \
  } \
}



void SetCtxModelNumber ()
{
  int frame_field = img->field_picture;
  int img_type    = img->type;
  int ctx_number  = img->currentSlice->start_mb_nr / num_mb_per_slice;

  if(img->type==I_SLICE)
  {
    img->model_number=DEFAULT_CTX_MODEL;
    return;
  }
  if(input->context_init_method==FIXED)
  {
    img->model_number=input->model_number;
    return;
  }

  if (initialized [frame_field][img_type][ctx_number])
  {
    img->model_number = model_number[frame_field][img_type][ctx_number];
  }
  else if (ctx_number && initialized[frame_field][img_type][ctx_number-1])
  {
    img->model_number = model_number[frame_field][img_type][ctx_number-1];
  }
  else
  {
    img->model_number = DEFAULT_CTX_MODEL;
  }
}



void init_contexts ()
{
  MotionInfoContexts*  mc = img->currentSlice->mot_ctx;
  TextureInfoContexts* tc = img->currentSlice->tex_ctx;
  int i, j;

  //--- motion coding contexts ---
  BIARI_CTX_INIT2 (3, NUM_MB_TYPE_CTX,   mc->mb_type_contexts,     INIT_MB_TYPE,    img->model_number);
  BIARI_CTX_INIT2 (2, NUM_B8_TYPE_CTX,   mc->b8_type_contexts,     INIT_B8_TYPE,    img->model_number);
  BIARI_CTX_INIT2 (2, NUM_MV_RES_CTX,    mc->mv_res_contexts,      INIT_MV_RES,     img->model_number);
  BIARI_CTX_INIT2 (2, NUM_REF_NO_CTX,    mc->ref_no_contexts,      INIT_REF_NO,     img->model_number);
  BIARI_CTX_INIT1 (   NUM_DELTA_QP_CTX,  mc->delta_qp_contexts,    INIT_DELTA_QP,   img->model_number);
  BIARI_CTX_INIT1 (   NUM_MB_AFF_CTX,    mc->mb_aff_contexts,      INIT_MB_AFF,     img->model_number);

  //--- texture coding contexts ---
  BIARI_CTX_INIT1 (                 NUM_IPR_CTX,  tc->ipr_contexts,     INIT_IPR,       img->model_number);
  BIARI_CTX_INIT1 (                 NUM_CIPR_CTX, tc->cipr_contexts,    INIT_CIPR,      img->model_number);
  BIARI_CTX_INIT2 (3,               NUM_CBP_CTX,  tc->cbp_contexts,     INIT_CBP,       img->model_number);
  BIARI_CTX_INIT2 (NUM_BLOCK_TYPES, NUM_BCBP_CTX, tc->bcbp_contexts,    INIT_BCBP,      img->model_number);
  BIARI_CTX_INIT2 (NUM_BLOCK_TYPES, NUM_MAP_CTX,  tc->map_contexts,     INIT_MAP,       img->model_number);
  BIARI_CTX_INIT2 (NUM_BLOCK_TYPES, NUM_LAST_CTX, tc->last_contexts,    INIT_LAST,      img->model_number);
  BIARI_CTX_INIT2 (NUM_BLOCK_TYPES, NUM_ONE_CTX,  tc->one_contexts,     INIT_ONE,       img->model_number);
  BIARI_CTX_INIT2 (NUM_BLOCK_TYPES, NUM_ABS_CTX,  tc->abs_contexts,     INIT_ABS,       img->model_number);
  BIARI_CTX_INIT2 (NUM_BLOCK_TYPES, NUM_MAP_CTX,  tc->fld_map_contexts, INIT_FLD_MAP,   img->model_number);
  BIARI_CTX_INIT2 (NUM_BLOCK_TYPES, NUM_LAST_CTX, tc->fld_last_contexts,INIT_FLD_LAST,  img->model_number);
}





double XRate (BiContextTypePtr ctx, const int* model)
{
  int     ctx_state, mod_state;
  double  weight, xr = 0.0;
  int     qp = img->qp;

  weight    = min (1.0, (double)ctx->count/(double)RELIABLE_COUNT);

  mod_state = ((model[0]*qp)>>4)+model[1];
  mod_state = min (max (0, mod_state), 127);
  ctx_state = (ctx->MPS ? 64+ctx->state : 63-ctx->state);

  xr -= weight * probability[    ctx_state] * entropy[    mod_state];
  xr -= weight * probability[127-ctx_state] * entropy[127-mod_state];

  return xr;
}

#define ADD_XRATE2(ii,jj,ctx,tab,num) \
{ \
  for (i=0; i<ii; i++) \
  for (j=0; j<jj; j++) \
  { \
    if      (img->type==I_SLICE)  xr += XRate (&(ctx[i][j]), &(tab ## _I[num][i][j][0])); \
    else                            xr += XRate (&(ctx[i][j]), &(tab ## _P[num][i][j][0])); \
  } \
}
#define ADD_XRATE1(jj,ctx,tab,num) \
{ \
  for (j=0; j<jj; j++) \
  { \
    if      (img->type==I_SLICE)  xr += XRate (&(ctx[j]), &(tab ## _I[num][0][j][0])); \
    else                            xr += XRate (&(ctx[j]), &(tab ## _P[num][0][j][0])); \
  } \
}


void GetCtxModelNumber (int* mnumber, MotionInfoContexts* mc, TextureInfoContexts* tc)
{
  int     model, j, i;
  int     num_models = (img->type==I_SLICE ? NUM_CTX_MODELS_I : NUM_CTX_MODELS_P);
  double  xr, min_xr = 1e30;

  for (model=0; model<num_models; model++)
  {
    xr = 0.0;
    //--- motion coding contexts ---
    ADD_XRATE2 (3, NUM_MB_TYPE_CTX,   mc->mb_type_contexts,     INIT_MB_TYPE,   model);
    ADD_XRATE2 (2, NUM_B8_TYPE_CTX,   mc->b8_type_contexts,     INIT_B8_TYPE,   model);
    ADD_XRATE2 (2, NUM_MV_RES_CTX,    mc->mv_res_contexts,      INIT_MV_RES,    model);
    ADD_XRATE2 (2, NUM_REF_NO_CTX,    mc->ref_no_contexts,      INIT_REF_NO,    model);
    ADD_XRATE1 (   NUM_DELTA_QP_CTX,  mc->delta_qp_contexts,    INIT_DELTA_QP,  model);
    ADD_XRATE1 (   NUM_MB_AFF_CTX,    mc->mb_aff_contexts,      INIT_MB_AFF,    model);
    //--- texture coding contexts ---
    ADD_XRATE1 (                  NUM_IPR_CTX,  tc->ipr_contexts,       INIT_IPR,       model);
    ADD_XRATE1 (                  NUM_CIPR_CTX, tc->cipr_contexts,      INIT_CIPR,      model);
    ADD_XRATE2 (3,                NUM_CBP_CTX,  tc->cbp_contexts,       INIT_CBP,       model);
    ADD_XRATE2 (NUM_BLOCK_TYPES,  NUM_BCBP_CTX, tc->bcbp_contexts,      INIT_BCBP,      model);
    ADD_XRATE2 (NUM_BLOCK_TYPES,  NUM_MAP_CTX,  tc->map_contexts,       INIT_MAP,       model);
    ADD_XRATE2 (NUM_BLOCK_TYPES,  NUM_LAST_CTX, tc->last_contexts,      INIT_LAST,      model);
    ADD_XRATE2 (NUM_BLOCK_TYPES,  NUM_ONE_CTX,  tc->one_contexts,       INIT_ONE,       model);
    ADD_XRATE2 (NUM_BLOCK_TYPES,  NUM_ABS_CTX,  tc->abs_contexts,       INIT_ABS,       model);
    ADD_XRATE2 (NUM_BLOCK_TYPES,  NUM_MAP_CTX,  tc->fld_map_contexts,   INIT_FLD_MAP,   model);
    ADD_XRATE2 (NUM_BLOCK_TYPES,  NUM_LAST_CTX, tc->fld_last_contexts,  INIT_FLD_LAST,  model);

    if (xr<min_xr)
    {
      min_xr    = xr;
      *mnumber  = model;
    }
  }
}

#undef ADD_XRATE2
#undef ADD_XRATE1






void store_contexts ()
{
  int frame_field = img->field_picture;
  int img_type    = img->type;
  int ctx_number  = img->currentSlice->start_mb_nr / num_mb_per_slice;

  if( input->context_init_method )
  {
    initialized [frame_field][img_type][ctx_number] = 1;
    GetCtxModelNumber (model_number[frame_field][img_type]+ctx_number, img->currentSlice->mot_ctx, img->currentSlice->tex_ctx);
  }
  else
  {
    // do nothing
  }
}


void update_field_frame_contexts (int field)
{
  int i, j;

  if (field)
  {
    // set frame contexts
    for (j=0; j<FRAME_TYPES; j++)
    {
      for (i=0; i<number_of_slices; i++)
      {
        initialized [0][j][i] = initialized [1][j][i>>1];
        model_number[0][j][i] = model_number[1][j][i>>1];
      }
    }
  }
  else
  {
    // set field contexts
    for (j=0; j<FRAME_TYPES; j++)
    {
      for (i=0; i<((number_of_slices+1)>>1); i++)
      {
        initialized [1][j][i] = initialized [0][j][i<<1];
        model_number[1][j][i] = model_number[0][j][i<<1];
      }
    }
  }
}

