/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

#include "descriptor.h"
#include "filter.h"
#include <emmintrin.h>

using namespace std;

Descriptor::Descriptor(uint8_t* I,int32_t width,int32_t height,int32_t bpl,bool half_resolution) {
  I_desc        = (uint8_t*)_mm_malloc(16*width*height*sizeof(uint8_t),16);
  uint8_t* I_du = (uint8_t*)_mm_malloc(bpl*height*sizeof(uint8_t),16);
  uint8_t* I_dv = (uint8_t*)_mm_malloc(bpl*height*sizeof(uint8_t),16);
  filter::sobel3x3(I,I_du,I_dv,bpl,height);
  createDescriptor(I_du,I_dv,width,height,bpl,half_resolution);
  _mm_free(I_du);
  _mm_free(I_dv);
}

Descriptor::~Descriptor() {
  _mm_free(I_desc);
}

void Descriptor::createDescriptor (uint8_t* I_du,uint8_t* I_dv,int32_t width,int32_t height,int32_t bpl,bool half_resolution) {

  uint8_t *I_desc_curr;  
  uint32_t addr_v0,addr_v1,addr_v2,addr_v3,addr_v4;
  
  // do not compute every second line
  if (half_resolution) {
  
    // create filter strip
    for (int32_t v=4; v<height-3; v+=2) {

      addr_v2 = v*bpl;
      addr_v0 = addr_v2-2*bpl;
      addr_v1 = addr_v2-1*bpl;
      addr_v3 = addr_v2+1*bpl;
      addr_v4 = addr_v2+2*bpl;

      for (int32_t u=3; u<width-3; u++) {
        I_desc_curr = I_desc+(v*width+u)*16;
        *(I_desc_curr++) = *(I_du+addr_v0+u+0);
        *(I_desc_curr++) = *(I_du+addr_v1+u-2);
        *(I_desc_curr++) = *(I_du+addr_v1+u+0);
        *(I_desc_curr++) = *(I_du+addr_v1+u+2);
        *(I_desc_curr++) = *(I_du+addr_v2+u-1);
        *(I_desc_curr++) = *(I_du+addr_v2+u+0);
        *(I_desc_curr++) = *(I_du+addr_v2+u+0);
        *(I_desc_curr++) = *(I_du+addr_v2+u+1);
        *(I_desc_curr++) = *(I_du+addr_v3+u-2);
        *(I_desc_curr++) = *(I_du+addr_v3+u+0);
        *(I_desc_curr++) = *(I_du+addr_v3+u+2);
        *(I_desc_curr++) = *(I_du+addr_v4+u+0);
        *(I_desc_curr++) = *(I_dv+addr_v1+u+0);
        *(I_desc_curr++) = *(I_dv+addr_v2+u-1);
        *(I_desc_curr++) = *(I_dv+addr_v2+u+1);
        *(I_desc_curr++) = *(I_dv+addr_v3+u+0);
      }
    }
    
  // compute full descriptor images
  } else {
    
    // create filter strip
    for (int32_t v=3; v<height-3; v++) {

      addr_v2 = v*bpl;
      addr_v0 = addr_v2-2*bpl;
      addr_v1 = addr_v2-1*bpl;
      addr_v3 = addr_v2+1*bpl;
      addr_v4 = addr_v2+2*bpl;

      for (int32_t u=3; u<width-3; u++) {
        I_desc_curr = I_desc+(v*width+u)*16;
        *(I_desc_curr++) = *(I_du+addr_v0+u+0);
        *(I_desc_curr++) = *(I_du+addr_v1+u-2);
        *(I_desc_curr++) = *(I_du+addr_v1+u+0);
        *(I_desc_curr++) = *(I_du+addr_v1+u+2);
        *(I_desc_curr++) = *(I_du+addr_v2+u-1);
        *(I_desc_curr++) = *(I_du+addr_v2+u+0);
        *(I_desc_curr++) = *(I_du+addr_v2+u+0);
        *(I_desc_curr++) = *(I_du+addr_v2+u+1);
        *(I_desc_curr++) = *(I_du+addr_v3+u-2);
        *(I_desc_curr++) = *(I_du+addr_v3+u+0);
        *(I_desc_curr++) = *(I_du+addr_v3+u+2);
        *(I_desc_curr++) = *(I_du+addr_v4+u+0);
        *(I_desc_curr++) = *(I_dv+addr_v1+u+0);
        *(I_desc_curr++) = *(I_dv+addr_v2+u-1);
        *(I_desc_curr++) = *(I_dv+addr_v2+u+1);
        *(I_desc_curr++) = *(I_dv+addr_v3+u+0);
      }
    }
  }
  
}
