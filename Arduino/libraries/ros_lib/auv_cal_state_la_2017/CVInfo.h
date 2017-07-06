#ifndef _ROS_auv_cal_state_la_2017_CVInfo_h
#define _ROS_auv_cal_state_la_2017_CVInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace auv_cal_state_la_2017
{

  class CVInfo : public ros::Msg
  {
    public:
      typedef int32_t _cameraNumber_type;
      _cameraNumber_type cameraNumber;
      typedef int32_t _taskNumber_type;
      _taskNumber_type taskNumber;
      typedef int32_t _givenColor_type;
      _givenColor_type givenColor;
      typedef int32_t _givenShape_type;
      _givenShape_type givenShape;
      typedef float _givenLength_type;
      _givenLength_type givenLength;
      typedef float _givenDistance_type;
      _givenDistance_type givenDistance;

    CVInfo():
      cameraNumber(0),
      taskNumber(0),
      givenColor(0),
      givenShape(0),
      givenLength(0),
      givenDistance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_cameraNumber;
      u_cameraNumber.real = this->cameraNumber;
      *(outbuffer + offset + 0) = (u_cameraNumber.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cameraNumber.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cameraNumber.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cameraNumber.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cameraNumber);
      union {
        int32_t real;
        uint32_t base;
      } u_taskNumber;
      u_taskNumber.real = this->taskNumber;
      *(outbuffer + offset + 0) = (u_taskNumber.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_taskNumber.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_taskNumber.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_taskNumber.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->taskNumber);
      union {
        int32_t real;
        uint32_t base;
      } u_givenColor;
      u_givenColor.real = this->givenColor;
      *(outbuffer + offset + 0) = (u_givenColor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_givenColor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_givenColor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_givenColor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->givenColor);
      union {
        int32_t real;
        uint32_t base;
      } u_givenShape;
      u_givenShape.real = this->givenShape;
      *(outbuffer + offset + 0) = (u_givenShape.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_givenShape.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_givenShape.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_givenShape.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->givenShape);
      union {
        float real;
        uint32_t base;
      } u_givenLength;
      u_givenLength.real = this->givenLength;
      *(outbuffer + offset + 0) = (u_givenLength.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_givenLength.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_givenLength.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_givenLength.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->givenLength);
      union {
        float real;
        uint32_t base;
      } u_givenDistance;
      u_givenDistance.real = this->givenDistance;
      *(outbuffer + offset + 0) = (u_givenDistance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_givenDistance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_givenDistance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_givenDistance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->givenDistance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_cameraNumber;
      u_cameraNumber.base = 0;
      u_cameraNumber.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cameraNumber.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cameraNumber.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cameraNumber.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cameraNumber = u_cameraNumber.real;
      offset += sizeof(this->cameraNumber);
      union {
        int32_t real;
        uint32_t base;
      } u_taskNumber;
      u_taskNumber.base = 0;
      u_taskNumber.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_taskNumber.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_taskNumber.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_taskNumber.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->taskNumber = u_taskNumber.real;
      offset += sizeof(this->taskNumber);
      union {
        int32_t real;
        uint32_t base;
      } u_givenColor;
      u_givenColor.base = 0;
      u_givenColor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_givenColor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_givenColor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_givenColor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->givenColor = u_givenColor.real;
      offset += sizeof(this->givenColor);
      union {
        int32_t real;
        uint32_t base;
      } u_givenShape;
      u_givenShape.base = 0;
      u_givenShape.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_givenShape.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_givenShape.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_givenShape.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->givenShape = u_givenShape.real;
      offset += sizeof(this->givenShape);
      union {
        float real;
        uint32_t base;
      } u_givenLength;
      u_givenLength.base = 0;
      u_givenLength.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_givenLength.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_givenLength.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_givenLength.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->givenLength = u_givenLength.real;
      offset += sizeof(this->givenLength);
      union {
        float real;
        uint32_t base;
      } u_givenDistance;
      u_givenDistance.base = 0;
      u_givenDistance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_givenDistance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_givenDistance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_givenDistance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->givenDistance = u_givenDistance.real;
      offset += sizeof(this->givenDistance);
     return offset;
    }

    const char * getType(){ return "auv_cal_state_la_2017/CVInfo"; };
    const char * getMD5(){ return "8723a30b7f69c63ec944a866a7c37339"; };

  };

}
#endif