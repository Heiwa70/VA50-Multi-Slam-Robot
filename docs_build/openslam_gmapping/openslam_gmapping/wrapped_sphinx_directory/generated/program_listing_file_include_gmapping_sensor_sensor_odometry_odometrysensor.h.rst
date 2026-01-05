
.. _program_listing_file_include_gmapping_sensor_sensor_odometry_odometrysensor.h:

Program Listing for File odometrysensor.h
=========================================

|exhale_lsh| :ref:`Return to documentation for file <file_include_gmapping_sensor_sensor_odometry_odometrysensor.h>` (``include/gmapping/sensor/sensor_odometry/odometrysensor.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef ODOMETRYSENSOR_H
   #define ODOMETRYSENSOR_H
   
   #include <string>
   #include <gmapping/sensor/sensor_base/sensor.h>
   
   namespace GMapping{
   
   class OdometrySensor: public Sensor{
       public:
           OdometrySensor(const std::string& name, bool ideal=false);
           inline bool isIdeal() const { return m_ideal; }
       protected:
           bool m_ideal;   
   };
   
   };
   
   #endif
   
