
.. _program_listing_file_include_gmapping_grid_accessstate.h:

Program Listing for File accessstate.h
======================================

|exhale_lsh| :ref:`Return to documentation for file <file_include_gmapping_grid_accessstate.h>` (``include/gmapping/grid/accessstate.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef ACCESSTATE_H
   #define ACCESSTATE_H
   
   namespace GMapping {
   enum AccessibilityState{Outside=0x0, Inside=0x1, Allocated=0x2};
   };
   
   #endif
   
