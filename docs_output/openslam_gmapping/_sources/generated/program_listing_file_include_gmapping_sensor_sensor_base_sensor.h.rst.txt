
.. _program_listing_file_include_gmapping_sensor_sensor_base_sensor.h:

Program Listing for File sensor.h
=================================

|exhale_lsh| :ref:`Return to documentation for file <file_include_gmapping_sensor_sensor_base_sensor.h>` (``include/gmapping/sensor/sensor_base/sensor.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef SENSOR_H
   #define SENSOR_H
   
   #include <string>
   #include <map>
   
   namespace GMapping{
   
   class Sensor{
       public:
           Sensor(const std::string& name="");
           virtual ~Sensor();
           inline std::string getName() const {return m_name;}
           inline void setName(const std::string& name) {m_name=name;}
       protected:
           std::string m_name;
   };
   
   typedef std::map<std::string, Sensor*> SensorMap;
   
   }; //end namespace
   
   #endif
   
