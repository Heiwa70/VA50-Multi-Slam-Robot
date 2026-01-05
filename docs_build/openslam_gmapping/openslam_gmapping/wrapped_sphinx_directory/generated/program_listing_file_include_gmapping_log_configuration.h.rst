
.. _program_listing_file_include_gmapping_log_configuration.h:

Program Listing for File configuration.h
========================================

|exhale_lsh| :ref:`Return to documentation for file <file_include_gmapping_log_configuration.h>` (``include/gmapping/log/configuration.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef CONFIGURATION_H
   #define CONFIGURATION_H
   
   #include <istream>
   #include <gmapping/sensor/sensor_base/sensor.h>
   
   namespace GMapping {
   
   class Configuration{
       public:
           virtual ~Configuration();
           virtual SensorMap computeSensorMap() const=0;
   };
   
   };
   #endif
   
