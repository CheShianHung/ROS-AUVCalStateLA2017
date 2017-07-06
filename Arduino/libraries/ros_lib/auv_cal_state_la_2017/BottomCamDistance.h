#ifndef _ROS_auv_cal_state_la_2017_BottomCamDistance_h
#define _ROS_auv_cal_state_la_2017_BottomCamDistance_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace auv_cal_state_la_2017
{

  class BottomCamDistance : public ros::Msg
  {
    public:
      typedef float _bottomCamForwardDistance_type;
      _bottomCamForwardDistance_type bottomCamForwardDistance;
      typedef float _bottomCamHorizontalDistance_type;
      _bottomCamHorizontalDistance_type bottomCamHorizontalDistance;
      typedef float _bottomCamVerticalDistance_type;
      _bottomCamVerticalDistance_type bottomCamVerticalDistance;

    BottomCamDistance():
      bottomCamForwardDistance(0),
      bottomCamHorizontalDistance(0),
      bottomCamVerticalDistance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_bottomCamForwardDistance;
      u_bottomCamForwardDistance.real = this->bottomCamForwardDistance;
      *(outbuffer + offset + 0) = (u_bottomCamForwardDistance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bottomCamForwardDistance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bottomCamForwardDistance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bottomCamForwardDistance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bottomCamForwardDistance);
      union {
        float real;
        uint32_t base;
      } u_bottomCamHorizontalDistance;
      u_bottomCamHorizontalDistance.real = this->bottomCamHorizontalDistance;
      *(outbuffer + offset + 0) = (u_bottomCamHorizontalDistance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bottomCamHorizontalDistance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bottomCamHorizontalDistance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bottomCamHorizontalDistance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bottomCamHorizontalDistance);
      union {
        float real;
        uint32_t base;
      } u_bottomCamVerticalDistance;
      u_bottomCamVerticalDistance.real = this->bottomCamVerticalDistance;
      *(outbuffer + offset + 0) = (u_bottomCamVerticalDistance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bottomCamVerticalDistance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bottomCamVerticalDistance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bottomCamVerticalDistance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bottomCamVerticalDistance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_bottomCamForwardDistance;
      u_bottomCamForwardDistance.base = 0;
      u_bottomCamForwardDistance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bottomCamForwardDistance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bottomCamForwardDistance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bottomCamForwardDistance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bottomCamForwardDistance = u_bottomCamForwardDistance.real;
      offset += sizeof(this->bottomCamForwardDistance);
      union {
        float real;
        uint32_t base;
      } u_bottomCamHorizontalDistance;
      u_bottomCamHorizontalDistance.base = 0;
      u_bottomCamHorizontalDistance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bottomCamHorizontalDistance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bottomCamHorizontalDistance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bottomCamHorizontalDistance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bottomCamHorizontalDistance = u_bottomCamHorizontalDistance.real;
      offset += sizeof(this->bottomCamHorizontalDistance);
      union {
        float real;
        uint32_t base;
      } u_bottomCamVerticalDistance;
      u_bottomCamVerticalDistance.base = 0;
      u_bottomCamVerticalDistance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bottomCamVerticalDistance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bottomCamVerticalDistance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bottomCamVerticalDistance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bottomCamVerticalDistance = u_bottomCamVerticalDistance.real;
      offset += sizeof(this->bottomCamVerticalDistance);
     return offset;
    }

    const char * getType(){ return "auv_cal_state_la_2017/BottomCamDistance"; };
    const char * getMD5(){ return "a4a15ad6335100902962ddb0eae27a67"; };

  };

}
#endif