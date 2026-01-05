
.. _program_listing_file_include_gmapping_sensor_sensor_odometry_odometryreading.h:

Program Listing for File odometryreading.h
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file_include_gmapping_sensor_sensor_odometry_odometryreading.h>` (``include/gmapping/sensor/sensor_odometry/odometryreading.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef ODOMETRYREADING_H
   #define ODOMETRYREADING_H
   
   #include <string.h>
   #include <gmapping/sensor/sensor_base/sensorreading.h>
   #include <gmapping/utils/point.h>
   #include "odometrysensor.h"
   
   namespace GMapping{
   
   class OdometryReading: public SensorReading{
       public:
           OdometryReading(const OdometrySensor* odo, double time=0);
           inline const OrientedPoint& getPose() const {return m_pose;}
           inline const OrientedPoint& getSpeed() const {return m_speed;}
           inline const OrientedPoint& getAcceleration() const {return m_acceleration;}
           inline void setPose(const OrientedPoint& pose) {m_pose=pose;}
           inline void setSpeed(const OrientedPoint& speed) {m_speed=speed;}
           inline void setAcceleration(const OrientedPoint& acceleration) {m_acceleration=acceleration;}
           
       protected:
           OrientedPoint m_pose;
           OrientedPoint m_speed;
           OrientedPoint m_acceleration;
   };
   
   };
   #endif
   
