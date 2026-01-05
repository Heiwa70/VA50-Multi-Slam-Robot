
.. _program_listing_file_include_gmapping_gridfastslam_motionmodel.h:

Program Listing for File motionmodel.h
======================================

|exhale_lsh| :ref:`Return to documentation for file <file_include_gmapping_gridfastslam_motionmodel.h>` (``include/gmapping/gridfastslam/motionmodel.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef MOTIONMODEL_H
   #define MOTIONMODEL_H
   
   #include <gmapping/utils/point.h>
   #include <gmapping/utils/stat.h>
   #include <gmapping/utils/macro_params.h>
   
   namespace  GMapping { 
   
   struct MotionModel{
       OrientedPoint drawFromMotion(const OrientedPoint& p, double linearMove, double angularMove) const;
       OrientedPoint drawFromMotion(const OrientedPoint& p, const OrientedPoint& pnew, const OrientedPoint& pold) const;
       Covariance3 gaussianApproximation(const OrientedPoint& pnew, const OrientedPoint& pold) const;
       double srr, str, srt, stt;
   };
   
   };
   
   #endif
