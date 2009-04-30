/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#ifndef VOXEL_GRID_VOXEL_GRID_
#define VOXEL_GRID_VOXEL_GRID_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>


/**
 * @class VoxelGrid
 * @brief A 3D grid sturcture that stores points as an integer array. X and Y index the array and Z selects which bit of the integer is used giving a limit of 16 vertical cells. 
 */
namespace voxel_grid {
  enum VoxelStatus {
    FREE = 0,
    UNKNOWN = 1,
    MARKED = 2,
  };

  class VoxelGrid{
    public:
      /**
       * @brief  Constructor for a voxel grid
       * @param size_x The x size of the grid 
       * @param size_y The y size of the grid 
       * @param size_z The z size of the grid, only sizes <= 16 are supported
       */
      VoxelGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z);

      ~VoxelGrid();

      inline void markVoxel(unsigned int x, unsigned int y, unsigned int z){
        if(x >= size_x || y >= size_y || z >= size_z){
          printf("Error, voxel out of bounds.\n");
          return;
        }
        uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
        data[y * size_x + x] |= full_mask; //clear unknown and mark cell
      }

      inline void clearVoxel(unsigned int x, unsigned int y, unsigned int z){
        if(x >= size_x || y >= size_y || z >= size_z){
          printf("Error, voxel out of bounds.\n");
          return;
        }
        uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
        data[y * size_x + x] &= ~(full_mask); //clear unknown and clear cell
      }

      inline void clearVoxelInMap(unsigned int x, unsigned int y, unsigned int z){
        if(x >= size_x || y >= size_y || z >= size_z){
          printf("Error, voxel out of bounds.\n");
          return;
        }
        int index = y * size_x + x;
        uint32_t* col = &data[index];
        uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
        *col &= ~(full_mask); //clear unknown and clear cell

        unsigned int unknown_bits = uint16_t(*col>>16) ^ uint16_t(*col);
        unsigned int marked_bits = *col>>16;

        //make sure the number of bits in each is below our thesholds
        if(bitsBelowThreshold(unknown_bits, 1) && bitsBelowThreshold(marked_bits, 1))
          cost_map[index] = 0;
      }

      inline bool bitsBelowThreshold(unsigned int n, unsigned int bit_threshold){
        unsigned int bit_count;
        for(bit_count = 0; n;){
          ++bit_count;
          if(bit_count > bit_threshold)
            return false;
          n &= n - 1; //clear the least significant bit set
        }
        return true;
      }

      inline unsigned int numBits(unsigned int n){
        unsigned int bit_count;
        for(bit_count = 0; n; ++bit_count){
          n &= n - 1; //clear the least significant bit set
        }
        return bit_count;
        
      }

      void markVoxelLine(unsigned int x0, unsigned y0, unsigned int z0, unsigned int x1, unsigned int y1, unsigned int z1);
      void clearVoxelLine(unsigned int x0, unsigned int y0, unsigned int z0, unsigned int x1, unsigned int y1, unsigned int z1);
      void clearVoxelLineInMap(unsigned int x0, unsigned int y0, unsigned int z0, unsigned int x1, unsigned int y1, unsigned int z1, unsigned char *map_2d, unsigned int unknown_threshold, unsigned int mark_threshold);

      VoxelStatus getVoxel(unsigned int x, unsigned int y, unsigned int z);
      VoxelStatus getVoxelColumn(unsigned int x, unsigned int y, 
          unsigned int unknown_threshold = 0, unsigned int marked_threshold = 0); //Are there any obstacles at that (x, y) location in the grid?

      void printVoxelGrid();
      void printColumnGrid();
      unsigned int sizeX();
      unsigned int sizeY();
      unsigned int sizeZ();

      template <class ActionType>
        inline void raytraceLine(ActionType at, unsigned int x0, unsigned int y0, unsigned int z0, 
            unsigned int x1, unsigned int y1, unsigned int z1){

          int dx = x1 - x0;
          int dy = y1 - y0;
          int dz = z1 - z0;

          unsigned int abs_dx = abs(dx);
          unsigned int abs_dy = abs(dy);
          unsigned int abs_dz = abs(dz);

          int offset_dx = sign(dx);
          int offset_dy = sign(dy) * size_x;
          int offset_dz = sign(dz);

          unsigned int z_mask = 1 << 16 << z0 | 1 << z0;
          unsigned int offset = y0 * size_x + x0;

          GridOffset grid_off(offset);
          ZOffset z_off(z_mask);

          //is x dominant
          if(abs_dx >= max(abs_dy, abs_dz)){
            int error_y = abs_dx / 2;
            int error_z = abs_dx / 2;

            bresenham3D(at, grid_off, grid_off, z_off, abs_dx, abs_dy, abs_dz, error_y, error_z, offset_dx, offset_dy, offset_dz, offset, z_mask);
            return;
          }

          //y is dominant
          if(abs_dy >= abs_dz){
            int error_x = abs_dy / 2;
            int error_z = abs_dy / 2;

            bresenham3D(at, grid_off, grid_off, z_off, abs_dy, abs_dx, abs_dz, error_x, error_z, offset_dy, offset_dx, offset_dz, offset, z_mask);
            return;
          }

          //otherwise, z is dominant
          int error_x = abs_dz / 2;
          int error_y = abs_dz / 2;

          bresenham3D(at, z_off, grid_off, grid_off, abs_dz, abs_dx, abs_dy, error_x, error_y, offset_dz, offset_dx, offset_dy, offset, z_mask);
        }

    private:

      //the real work is done here... 3D bresenham implementation
      template <class ActionType, class OffA, class OffB, class OffC>
        inline void bresenham3D(ActionType at, OffA off_a, OffB off_b, OffC off_c,
            unsigned int abs_da, unsigned int abs_db, unsigned int abs_dc,
            int error_b, int error_c, int offset_a, int offset_b, int offset_c, unsigned int &offset,
            unsigned int &z_mask){
          for(unsigned int i = 0; i < abs_da; ++i){
            at(offset, z_mask);
            off_a(offset_a);
            error_b += abs_db;
            error_c += abs_dc;
            if((unsigned int)error_b >= abs_da){
              off_b(offset_b);
              error_b -= abs_da;
            }
            if((unsigned int)error_c >= abs_da){
              off_c(offset_c);
              error_c -= abs_da;
            }
          }
          at(offset, z_mask);
        }

      inline int sign(int i){
        return i > 0 ? 1 : -1;
      }

      inline unsigned int max(unsigned int x, unsigned int y){
        return x > y ? x : y;
      }

      unsigned int size_x, size_y, size_z;
      uint32_t *data;
      unsigned char *cost_map;

      //Aren't functors so much fun... used to recreate the Bresenham macro Eric wrote in the original version, but in "proper" c++
      class MarkVoxel {
        public:
          MarkVoxel(uint32_t* data): data_(data){}
          inline void operator()(unsigned int offset, unsigned int z_mask){
            data_[offset] |= z_mask; //clear unknown and mark cell
          }
        private:
          uint32_t* data_;
      };

      class ClearVoxel {
        public:
          ClearVoxel(uint32_t* data): data_(data){}
          inline void operator()(unsigned int offset, unsigned int z_mask){
            data_[offset] &= ~(z_mask); //clear unknown and clear cell
          }
        private:
          uint32_t* data_;
      };

      class ClearVoxelInMap {
        public:
          ClearVoxelInMap(uint32_t* data, unsigned char *cost_map, 
              unsigned int unknown_clear_threshold, unsigned int marked_clear_threshold): data_(data), cost_map_(cost_map),
        unknown_clear_threshold_(unknown_clear_threshold), marked_clear_threshold_(marked_clear_threshold){}
          inline void operator()(unsigned int offset, unsigned int z_mask){
            uint32_t* col = &data_[offset];
            *col &= ~(z_mask); //clear unknown and clear cell

            unsigned int unknown_bits = uint16_t(*col>>16) ^ uint16_t(*col);
            unsigned int marked_bits = *col>>16;

            //make sure the number of bits in each is below our thesholds
            if(bitsBelowThreshold(unknown_bits, unknown_clear_threshold_) && bitsBelowThreshold(marked_bits, marked_clear_threshold_))
              cost_map_[offset] = 0;
          }
        private:
          inline bool bitsBelowThreshold(unsigned int n, unsigned int bit_threshold){
            unsigned int bit_count;
            for(bit_count = 0; n;){
              ++bit_count;
              if(bit_count > bit_threshold)
                return false;
              n &= n - 1; //clear the least significant bit set
            }
            return true;
          }
          uint32_t* data_;
          unsigned char *cost_map_;
          unsigned int unknown_clear_threshold_, marked_clear_threshold_;
      };

      class GridOffset {
        public:
          GridOffset(unsigned int &offset) : offset_(offset) {}
          inline void operator()(int offset_val){
            offset_ += offset_val;
          }
        private:
          unsigned int &offset_;
      };

      class ZOffset {
        public:
          ZOffset(unsigned int &z_mask) : z_mask_(z_mask) {}
          inline void operator()(int offset_val){
            offset_val > 0 ? z_mask_ <<= 1 : z_mask_ >>= 1;
          }
        private:
          unsigned int & z_mask_;
      };

  };
};

#endif
