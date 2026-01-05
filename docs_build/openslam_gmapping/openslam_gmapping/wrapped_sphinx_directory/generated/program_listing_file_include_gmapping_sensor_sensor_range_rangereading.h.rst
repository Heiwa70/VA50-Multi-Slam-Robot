
.. _program_listing_file_include_gmapping_sensor_sensor_range_rangereading.h:

Program Listing for File rangereading.h
=======================================

|exhale_lsh| :ref:`Return to documentation for file <file_include_gmapping_sensor_sensor_range_rangereading.h>` (``include/gmapping/sensor/sensor_range/rangereading.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef RANGEREADING_H
   #define RANGEREADING_H
   
   #include <vector>
   #include <gmapping/sensor/sensor_base/sensorreading.h>
   #include <gmapping/sensor/sensor_range/rangesensor.h>
   
   namespace GMapping{
   
   class RangeReading: public SensorReading, public std::vector<double>{
       public:
           RangeReading(const RangeSensor* rs, double time=0);
           RangeReading(unsigned int n_beams, const double* d, const RangeSensor* rs, double time=0);
           virtual ~RangeReading();
           inline const OrientedPoint& getPose() const {return m_pose;}
           inline void setPose(const OrientedPoint& pose) {m_pose=pose;}
           unsigned int rawView(double* v, double density=0.) const;
           std::vector<Point> cartesianForm(double maxRange=1e6) const;
           unsigned int activeBeams(double density=0.) const;
       protected:
           OrientedPoint m_pose;
   };
   
   };
   
   #endif
