/** \file algorithm.h ******************************************************
*
* Project: MAXREFDES117#
* Filename: algorithm.h
* Description: This module is the heart rate/SpO2 calculation algorithm header file
*
* Revision History:
*\n 1-18-2016 Rev 01.00 SK Initial release.
*\n
*
* --------------------------------------------------------------------
*
* This code follows the following naming conventions:
*
*\n char              ch_pmod_value
*\n char (array)      s_pmod_s_string[16]
*\n float             f_pmod_value
*\n s32           n_pmod_value
*\n s32 (array)   an_pmod_value[16]
*\n int16_t           w_pmod_value
*\n int16_t (array)   aw_pmod_value[16]
*\n u16          uw_pmod_value
*\n u16 (array)  auw_pmod_value[16]
*\n u8           uch_pmod_value
*\n u8 (array)   auch_pmod_buffer[16]
*\n u32          un_pmod_value
*\n s32 *         pn_pmod_value
*
* ------------------------------------------------------------------------- */
/*******************************************************************************
* Copyright (C) 2015 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#include "ql_type.h"



#define FS 100
#define BUFFER_SIZE  (FS* 5) 
#define HR_FIFO_SIZE 7
#define MA4_SIZE  4 // DO NOT CHANGE
#define HAMMING_SIZE  5// DO NOT CHANGE
#define min(x,y) ((x) < (y) ? (x) : (y))





void maxim_heart_rate_and_oxygen_saturation(u32 *pun_ir_buffer ,  s32 n_ir_buffer_length, u32 *pun_red_buffer ,   s32 *pn_spo2, s8 *pch_spo2_valid ,  s32 *pn_heart_rate , s8  *pch_hr_valid);
void maxim_find_peaks( s32 *pn_locs, s32 *pn_npks,  s32 *pn_x, s32 n_size, s32 n_min_height, s32 n_min_distance, s32 n_max_num );
void maxim_peaks_above_min_height( s32 *pn_locs, s32 *pn_npks,  s32 *pn_x, s32 n_size, s32 n_min_height );
void maxim_remove_close_peaks( s32 *pn_locs, s32 *pn_npks,   s32  *pn_x, s32 n_min_distance );
void maxim_sort_ascend( s32 *pn_x, s32 n_size );
void maxim_sort_indices_descend(  s32  *pn_x, s32 *pn_indx, s32 n_size);

#endif /* ALGORITHM_H_ */

