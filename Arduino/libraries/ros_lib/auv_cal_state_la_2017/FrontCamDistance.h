#ifndef _ROS_auv_cal_state_la_2017_FrontCamDistance_h
#define _ROS_auv_cal_state_la_2017_FrontCamDistance_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace auv_cal_state_la_2017
{

  class FrontCamDistance : public ros::Msg
  {
    public:
      typedef float _frontCamForwardDistance_type;
      _frontCamForwardDistance_type frontCamForwardDistance;
      typedef float _frontCamHorizontalDistance_type;
      _frontCamHorizontalDistance_type frontCamHorizontalDistance;
      typedef float _frontCamVerticalDistance_type;
      _frontCamVerticalDistance_type frontCamVerticalDistance;

    FrontCamDistance():
      frontCamForwardDistance(0),
      frontCamHorizontalDistance(0),
      frontCamVerticalDistance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_frontCamForwardDistance;
      u_frontCamForwardDistance.real = this->frontCamForwardDistance;
      *(outbuffer + offset + 0) = (u_frontCamForwardDistance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_frontCamForwardDistance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_frontCamForwardDistance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_frontCamForwardDistance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frontCamForwardDistance);
      union {
        float real;
        uint32_t base;
      } u_frontCamHorizontalDistance;
      u_frontCamHorizontalDistance.real = this->frontCamHorizontalDistance;
      *(outbuffer + offset + 0) = (u_frontCamHorizontalDistance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_frontCamHorizontalDistance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_frontCamHorizontalDistance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_frontCamHorizontalDistance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frontCamHorizontalDistance);
      union {
        float real;
        uint32_t base;
      } u_frontCamVerticalDistance;
      u_frontCamVerticalDistance.real = this->frontCamVerticalDistance;
      *(outbuffer + offset + 0) = (u_frontCamVerticalDistance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_frontCamVerticalDistance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_frontCamVerticalDistance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_frontCamVerticalDistance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frontCamVerticalDistance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_frontCamForwardDistance;
      u_frontCamForwardDistance.base = 0;
      u_frontCamForwardDistance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_frontCamForwardDistance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_frontCamForwardDistance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_frontCamForwardDistance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->frontCamForwardDistance = u_frontCamForwardDistance.real;
      offset += sizeof(this->frontCamForwardDistance);
      union {
        float real;
        uint32_t base;
      } u_frontCamHorizontalDistance;
      u_frontCamHorizontalDistance.base = 0;
      u_frontCamHorizontalDistance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_frontCamHorizontalDistance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_frontCamHorizontalDistance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_frontCamHorizontalDistance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->frontCamHorizontalDistance = u_frontCamHorizontalDistance.real;
      offset += sizeof(this->frontCamHorizontalDistance);
      union {
        float real;
        uint32_t base;
      } u_frontCamVerticalDistance;
      u_frontCamVerticalDistance.base = 0;
      u_frontCamVerticalDistance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_frontCamVerticalDistance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_frontCamVerticalDistance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_frontCamVerticalDistance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->frontCamVerticalDistance = u_frontCamVerticalDistance.real;
      offset += sizeof(this->frontCamVerticalDistance);
     return offset;
    }

    const char * getType(){ return "auv_cal_state_la_2017/FrontCamDistance"; };
    const char * getMD5(){ return "65f5794fdf87a86db880b10f7ba78110"; };

  };

}
#endif