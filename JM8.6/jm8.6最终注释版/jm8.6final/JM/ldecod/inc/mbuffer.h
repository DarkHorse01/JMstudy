
/*!
 ***********************************************************************
 *  \file
 *      mbuffer.h
 *
 *  \brief
 *      Frame buffer functions
 *
 *  \author
 *      Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Karsten S�hring          <suehring@hhi.de>
 ***********************************************************************
 */
#ifndef _MBUFFER_H_
#define _MBUFFER_H_

#include "global.h"

#define MAX_LIST_SIZE 33

//! definition a picture (field or frame)
typedef struct storable_picture
{
  PictureStructure structure;

  int         poc;
  int         top_poc;
  int         bottom_poc;
  int         frame_poc;
  int         order_num;
  int64       ref_pic_num        [MAX_NUM_SLICES][6][MAX_LIST_SIZE];
  int64       frm_ref_pic_num    [MAX_NUM_SLICES][6][MAX_LIST_SIZE];
  int64       top_ref_pic_num    [MAX_NUM_SLICES][6][MAX_LIST_SIZE];
  int64       bottom_ref_pic_num [MAX_NUM_SLICES][6][MAX_LIST_SIZE];
  unsigned    frame_num;
  int         pic_num;
  int         long_term_pic_num;
  int         long_term_frame_idx;

  int         is_long_term;
  int         used_for_reference;
  int         is_output;
  int         non_existing;

  int         max_slice_id;

  int         size_x, size_y, size_x_cr, size_y_cr;
  int         chroma_vector_adjustment;
  int         coded_frame;
  int         MbaffFrameFlag;
  unsigned    PicWidthInMbs;
  unsigned    PicSizeInMbs;

  byte **     imgY;          //!< Y picture component
  byte ***    imgUV;         //!< U and V picture components

  byte *      mb_field;      //!< field macroblock indicator

  int  **     slice_id;      //!< reference picture   [mb_x][mb_y]

  int  ***    ref_idx;       //!< reference picture   [list][subblock_x][subblock_y]

  int64 ***    ref_pic_id;    //!< reference picture identifier [list][subblock_x][subblock_y]
                             //   (not  simply index) 

  int64 ***    ref_id;    //!< reference picture identifier [list][subblock_x][subblock_y]
                             //   (not  simply index) 

  int  ****   mv;            //!< motion vector       [list][subblock_x][subblock_y][component]
  
  byte **     moving_block;
  byte **     field_frame;         //!< indicates if co_located is field or frame.

  struct storable_picture *top_field;     // for mb aff, if frame for referencing the top field
  struct storable_picture *bottom_field;  // for mb aff, if frame for referencing the bottom field
  struct storable_picture *frame;         // for mb aff, if field for referencing the combined frame

  int         slice_type;
  int         idr_flag;
  int         no_output_of_prior_pics_flag;
  int         long_term_reference_flag;
  int         adaptive_ref_pic_buffering_flag;

  int         frame_mbs_only_flag;
  int         frame_cropping_flag;
  int         frame_cropping_rect_left_offset;
  int         frame_cropping_rect_right_offset;
  int         frame_cropping_rect_top_offset;
  int         frame_cropping_rect_bottom_offset;

  DecRefPicMarking_t *dec_ref_pic_marking_buffer;                    //!< stores the memory management control operations

} StorablePicture;


//! definition a picture (field or frame)
typedef struct colocated_params
{
//  byte *      mb_field;      //!< field macroblock indicator
  int         mb_adaptive_frame_field_flag;
  int         size_x, size_y;

  int64       ref_pic_num[6][MAX_LIST_SIZE];  

  int  ***    ref_idx;       //!< reference picture   [list][subblock_x][subblock_y]
  int64 ***    ref_pic_id;    //!< reference picture identifier [list][subblock_x][subblock_y]
  int  ****   mv;            //!< motion vector       [list][subblock_x][subblock_y][component]  
  byte **     moving_block;

  // Top field params
  int64       top_ref_pic_num[6][MAX_LIST_SIZE];  
  int  ***    top_ref_idx;       //!< reference picture   [list][subblock_x][subblock_y]
  int64 ***    top_ref_pic_id;    //!< reference picture identifier [list][subblock_x][subblock_y]
  int  ****   top_mv;            //!< motion vector       [list][subblock_x][subblock_y][component]  
  byte **     top_moving_block;

  // Bottom field params
  int64       bottom_ref_pic_num[6][MAX_LIST_SIZE];  
  int  ***    bottom_ref_idx;       //!< reference picture   [list][subblock_x][subblock_y]
  int64 ***    bottom_ref_pic_id;    //!< reference picture identifier [list][subblock_x][subblock_y]
  int  ****   bottom_mv;            //!< motion vector       [list][subblock_x][subblock_y][component] 
  byte **     bottom_moving_block;
  
  int         is_long_term;
  byte **     field_frame;         //!< indicates if co_located is field or frame.
  //struct colocated_params *top_field;     // for mb aff, if frame for referencing the top field
  //struct colocated_params *bottom_field;  // for mb aff, if frame for referencing the bottom field
  //struct colocated_params *frame;         // for mb aff, if field for referencing the combined frame

} ColocatedParams;

//! Frame Stores for Decoded Picture Buffer
typedef struct frame_store
{
  int       is_used;                //!< 0=empty; 1=top; 2=bottom; 3=both fields (or frame)
  int       is_reference;           //!< 0=not used for ref; 1=top used; 2=bottom used; 3=both fields (or frame) used
  int       is_long_term;           //!< 0=not used for ref; 1=top used; 2=bottom used; 3=both fields (or frame) used
  int       is_orig_reference;      //!< original marking by nal_ref_idc: 0=not used for ref; 1=top used; 2=bottom used; 3=both fields (or frame) used

  int       is_non_existent;

  unsigned  frame_num;
  int       frame_num_wrap;
  int       long_term_frame_idx;
  int       is_output;
  int       poc;

  StorablePicture *frame;
  StorablePicture *top_field;
  StorablePicture *bottom_field;

} FrameStore;


//! Decoded Picture Buffer
typedef struct decoded_picture_buffer
{
  FrameStore  **fs;                     //指向图像存储帧的指针数组。
  FrameStore  **fs_ref;                 //指向短期参考帧的指针数组。
  FrameStore  **fs_ltref;               //指向长期参考帧的指针数组。
  unsigned      size;                   //缓冲区的大小，即fs数组的容量。
  unsigned      used_size;              //缓冲区中已使用的大小，即fs数组中存储的帧的数量。
  unsigned      ref_frames_in_buffer;   //缓冲区中的参考帧数量，包括短期参考帧和长期参考帧。
  unsigned      ltref_frames_in_buffer; //缓冲区中的长期参考帧数量。
  int           last_output_poc;        //最近输出的图像帧的解码顺序号（POC）
  int           max_long_term_pic_idx;  //最大的长期参考帧索引号。

  int           init_done;              //缓冲区是否已初始化的标志。
  int           num_ref_frames;         //缓冲区中的参考帧数量，不包括长期参考帧。

  FrameStore   *last_picture;           //指向最近一帧的图像存储帧的指针。
} DecodedPictureBuffer;


extern DecodedPictureBuffer dpb;
extern StorablePicture **listX[6];
extern int listXsize[6];

void             init_dpb();
void             free_dpb();
FrameStore*      alloc_frame_store();
void             free_frame_store(FrameStore* f);
StorablePicture* alloc_storable_picture(PictureStructure type, int size_x, int size_y, int size_x_cr, int size_y_cr);
void             free_storable_picture(StorablePicture* p);
void             store_picture_in_dpb(StorablePicture* p);
void             flush_dpb();

void             dpb_split_field(FrameStore *fs);
void             dpb_combine_field(FrameStore *fs);

void             init_lists(int currSliceType, PictureStructure currPicStructure);
void             reorder_ref_pic_list(StorablePicture **list, int *list_size, 
                                      int num_ref_idx_lX_active_minus1, int *remapping_of_pic_nums_idc, 
                                      int *abs_diff_pic_num_minus1, int *long_term_pic_idx);

void             init_mbaff_lists();
void             alloc_ref_pic_list_reordering_buffer(Slice *currSlice);
void             free_ref_pic_list_reordering_buffer(Slice *currSlice);

void             fill_frame_num_gap(ImageParameters *img);

ColocatedParams* alloc_colocated(int size_x, int size_y,int mb_adaptive_frame_field_flag);
void free_collocated(ColocatedParams* p);
void compute_collocated(ColocatedParams* p, StorablePicture **listX[6]);

#endif

