<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.15.0">
  <compound kind="class">
    <name>GMapping::Array2D</name>
    <filename>class_g_mapping_1_1_array2_d.html</filename>
    <templarg>class Cell</templarg>
    <templarg>const bool debug</templarg>
  </compound>
  <compound kind="class">
    <name>GMapping::autoptr</name>
    <filename>class_g_mapping_1_1autoptr.html</filename>
    <templarg>class X</templarg>
    <class kind="struct">GMapping::autoptr::reference</class>
  </compound>
  <compound kind="struct">
    <name>auxiliary_evolver</name>
    <filename>structauxiliary__evolver.html</filename>
    <templarg>class Particle</templarg>
    <templarg>class Numeric</templarg>
    <templarg>class QualificationModel</templarg>
    <templarg>class EvolutionModel</templarg>
    <templarg>class LikelyhoodModel</templarg>
  </compound>
  <compound kind="struct">
    <name>GMapping::RangeSensor::Beam</name>
    <filename>struct_g_mapping_1_1_range_sensor_1_1_beam.html</filename>
  </compound>
  <compound kind="class">
    <name>GMapping::Configuration</name>
    <filename>class_g_mapping_1_1_configuration.html</filename>
  </compound>
  <compound kind="struct">
    <name>GMapping::Covariance3</name>
    <filename>struct_g_mapping_1_1_covariance3.html</filename>
  </compound>
  <compound kind="struct">
    <name>GMapping::EigenCovariance3</name>
    <filename>struct_g_mapping_1_1_eigen_covariance3.html</filename>
  </compound>
  <compound kind="struct">
    <name>evolver</name>
    <filename>structevolver.html</filename>
    <templarg>class Particle</templarg>
    <templarg>class EvolutionModel</templarg>
  </compound>
  <compound kind="struct">
    <name>GMapping::Gaussian3</name>
    <filename>struct_g_mapping_1_1_gaussian3.html</filename>
  </compound>
  <compound kind="class">
    <name>GMapping::GridSlamProcessor</name>
    <filename>class_g_mapping_1_1_grid_slam_processor.html</filename>
    <class kind="struct">GMapping::GridSlamProcessor::TNode</class>
    <class kind="struct">GMapping::GridSlamProcessor::Particle</class>
    <member kind="function">
      <type></type>
      <name>GridSlamProcessor</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a736f647e15e7b3e5f853920beb7b2b80</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>GridSlamProcessor</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>acccb98ea4c54b844acf14d7ec979e8a8</anchor>
      <arglist>(std::ostream &amp;infoStr)</arglist>
    </member>
    <member kind="function">
      <type>GridSlamProcessor *</type>
      <name>clone</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a2e5270f6e87161189f19e5880db792ca</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~GridSlamProcessor</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>ad0acfbe24e22ca2450052d67013c06e3</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>TNodeVector</type>
      <name>getTrajectories</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a9a4a65da5c87f57b28623957eae640a4</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::ofstream &amp;</type>
      <name>outputStream</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a4cd45d059a6cc1b82cb7688addccfd29</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::ostream &amp;</type>
      <name>infoStream</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>ae7be39c0e2a9cea422e1a76c96a407b6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const ParticleVector &amp;</type>
      <name>getParticles</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a2e844f171384ea4b85e82a9c5c95916f</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a59fd57518b8010bdf60972b239067f19</anchor>
      <arglist>(m_matcher, double, laserMaxRange, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a76a51cf2adbc8b4ff976d94b1c7d6e91</anchor>
      <arglist>(m_matcher, double, usableRange, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a1a4b79434cb2b998ebeecdf84bec85c5</anchor>
      <arglist>(m_matcher, double, gaussianSigma, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a0ca266215482753f991e702f222e5f4b</anchor>
      <arglist>(m_matcher, double, likelihoodSigma, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>abe193f91a1ddcdf68cf65284217c64d8</anchor>
      <arglist>(m_matcher, int, kernelSize, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a2d7ed8bd432b4e8fd2f415536f48231d</anchor>
      <arglist>(m_matcher, double, optAngularDelta, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>ab1d5ded4e425ece05d38a98fbe0a8b2c</anchor>
      <arglist>(m_matcher, double, optLinearDelta, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a52575a11c089158b1a37e74ead526da9</anchor>
      <arglist>(m_matcher, unsigned int, optRecursiveIterations, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a600cd2d77f3662e3c4c6c241be4aaaac</anchor>
      <arglist>(m_matcher, unsigned int, likelihoodSkip, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a61ebeb294126d9081b8cb099fc60c6b0</anchor>
      <arglist>(m_matcher, double, llsamplerange, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>ab6234158b112bfd2aba9ce5a063eff78</anchor>
      <arglist>(m_matcher, double, lasamplerange, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a7a491940692ed37326fc746e27c687bc</anchor>
      <arglist>(m_matcher, double, llsamplestep, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>ae3fb37cdec3e49f4a74d7e5adbeaed75</anchor>
      <arglist>(m_matcher, double, lasamplestep, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a375b3327975b1d8de93d5b8c4c494463</anchor>
      <arglist>(m_matcher, bool, generateMap, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>ad7e50913d07e1003d2fb97b40b3768aa</anchor>
      <arglist>(m_matcher, bool, enlargeStep, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MEMBER_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a02a3a4462d3e5f35e746a789f0d9674c</anchor>
      <arglist>(m_matcher, OrientedPoint, laserPose, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>STRUCT_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a5dca255f4b499a703a002715fd70828e</anchor>
      <arglist>(m_motionModel, double, srr, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>STRUCT_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a10f9bff87947ec28aedae11a3ec97f3d</anchor>
      <arglist>(m_motionModel, double, srt, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>STRUCT_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a8cb558f4aa4f6f57f2173a0ac8f63bc1</anchor>
      <arglist>(m_motionModel, double, str, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>STRUCT_PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a272f53828c2033879c364de540d1e0c0</anchor>
      <arglist>(m_motionModel, double, stt, protected, public, public)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>aef1af48fa8542ceff98b81e1d0f6e039</anchor>
      <arglist>(double, minimumScore, protected, public, public)</arglist>
    </member>
    <member kind="variable">
      <type>ScanMatcher</type>
      <name>m_matcher</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>ac50952d259590c4aeba22eee4e6437f5</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>GridSlamProcessor</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a7800b02d50b6b5d7c8b226e3c361ca2c</anchor>
      <arglist>(const GridSlamProcessor &amp;gsp)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>PARAM_SET_GET</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a9669d7233fda1a872ee2e7bfd58422f6</anchor>
      <arglist>(double, resampleThreshold, protected, public, public)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>unsigned int</type>
      <name>m_beams</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>aec30b4ee3a767c9f50bb95236851a1be</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>ParticleVector</type>
      <name>m_particles</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a67c2ade1a066269de12fe382999f2905</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; unsigned int &gt;</type>
      <name>m_indexes</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a79cee7069bb938f3b9d6d35b42b48ec6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; double &gt;</type>
      <name>m_weights</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>a39e71ee439dde6ca9ccd2929cf12551f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>MotionModel</type>
      <name>m_motionModel</name>
      <anchorfile>class_g_mapping_1_1_grid_slam_processor.html</anchorfile>
      <anchor>ae1fa20e9679abdbc23784e2dc17194b5</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>GMapping::HierarchicalArray2D</name>
    <filename>class_g_mapping_1_1_hierarchical_array2_d.html</filename>
    <templarg>class Cell</templarg>
    <base>GMapping::Array2D&lt; autoptr&lt; Array2D&lt; Cell &gt; &gt; &gt;</base>
  </compound>
  <compound kind="class">
    <name>GMapping::Map</name>
    <filename>class_g_mapping_1_1_map.html</filename>
    <templarg>class Cell</templarg>
    <templarg>class Storage</templarg>
    <templarg>const bool isClass</templarg>
  </compound>
  <compound kind="struct">
    <name>GMapping::MotionModel</name>
    <filename>struct_g_mapping_1_1_motion_model.html</filename>
  </compound>
  <compound kind="class">
    <name>GMapping::OdometryReading</name>
    <filename>class_g_mapping_1_1_odometry_reading.html</filename>
    <base>GMapping::SensorReading</base>
  </compound>
  <compound kind="class">
    <name>GMapping::OdometrySensor</name>
    <filename>class_g_mapping_1_1_odometry_sensor.html</filename>
    <base>GMapping::Sensor</base>
  </compound>
  <compound kind="struct">
    <name>GMapping::orientedpoint</name>
    <filename>struct_g_mapping_1_1orientedpoint.html</filename>
    <templarg>class T</templarg>
    <templarg>class A</templarg>
    <base>GMapping::point&lt; T &gt;</base>
  </compound>
  <compound kind="struct">
    <name>GMapping::GridSlamProcessor::Particle</name>
    <filename>struct_g_mapping_1_1_grid_slam_processor_1_1_particle.html</filename>
    <member kind="function">
      <type></type>
      <name>Particle</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_particle.html</anchorfile>
      <anchor>a51dc73563ad5e3035f8cb3b51965854a</anchor>
      <arglist>(const ScanMatcherMap &amp;map)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator double</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_particle.html</anchorfile>
      <anchor>a0b78d91dc4d41c565c388fe7320603b3</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator OrientedPoint</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_particle.html</anchorfile>
      <anchor>a4538a813d749fadc1ba922858cdd8d7f</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setWeight</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_particle.html</anchorfile>
      <anchor>a07888ebc307fab0bfee912bbfda45832</anchor>
      <arglist>(double w)</arglist>
    </member>
    <member kind="variable">
      <type>ScanMatcherMap</type>
      <name>map</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_particle.html</anchorfile>
      <anchor>a33f1690ccf3a95622e0246e75f5af145</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>OrientedPoint</type>
      <name>pose</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_particle.html</anchorfile>
      <anchor>a82d332a479fe5441be6f6f7be32ba39d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>OrientedPoint</type>
      <name>previousPose</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_particle.html</anchorfile>
      <anchor>aa0bfef7c1113864bc0dd5c3e38cfbf38</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>weight</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_particle.html</anchorfile>
      <anchor>a29fee66d34dd87e2bd641eeebfe6fc00</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>weightSum</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_particle.html</anchorfile>
      <anchor>a9e1d6ebe3b70943834d2987e826313ef</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>previousIndex</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_particle.html</anchorfile>
      <anchor>ab80f34a85a79ad4584d25b162bb72e5b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>TNode *</type>
      <name>node</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_particle.html</anchorfile>
      <anchor>a9c91ccdf7009b50785d939e1791ae4be</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>GMapping::point</name>
    <filename>struct_g_mapping_1_1point.html</filename>
    <templarg>class T</templarg>
  </compound>
  <compound kind="struct">
    <name>GMapping::PointAccumulator</name>
    <filename>struct_g_mapping_1_1_point_accumulator.html</filename>
  </compound>
  <compound kind="struct">
    <name>GMapping::pointcomparator</name>
    <filename>struct_g_mapping_1_1pointcomparator.html</filename>
    <templarg>class T</templarg>
  </compound>
  <compound kind="struct">
    <name>GMapping::pointradialcomparator</name>
    <filename>struct_g_mapping_1_1pointradialcomparator.html</filename>
    <templarg>class T</templarg>
  </compound>
  <compound kind="class">
    <name>GMapping::RangeReading</name>
    <filename>class_g_mapping_1_1_range_reading.html</filename>
    <base>GMapping::SensorReading</base>
  </compound>
  <compound kind="class">
    <name>GMapping::RangeSensor</name>
    <filename>class_g_mapping_1_1_range_sensor.html</filename>
    <base>GMapping::Sensor</base>
    <class kind="struct">GMapping::RangeSensor::Beam</class>
  </compound>
  <compound kind="struct">
    <name>GMapping::autoptr::reference</name>
    <filename>struct_g_mapping_1_1autoptr_1_1reference.html</filename>
  </compound>
  <compound kind="class">
    <name>GMapping::ScanMatcher</name>
    <filename>class_g_mapping_1_1_scan_matcher.html</filename>
    <member kind="variable" protection="protected">
      <type>unsigned int</type>
      <name>m_laserBeams</name>
      <anchorfile>class_g_mapping_1_1_scan_matcher.html</anchorfile>
      <anchor>ade447080711cbb616eb19cdd55d6a266</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>IntPoint *</type>
      <name>m_linePoints</name>
      <anchorfile>class_g_mapping_1_1_scan_matcher.html</anchorfile>
      <anchor>a73a0f19d405bb52d342e73e6152bf76a</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>GMapping::Sensor</name>
    <filename>class_g_mapping_1_1_sensor.html</filename>
  </compound>
  <compound kind="class">
    <name>GMapping::SensorLog</name>
    <filename>class_g_mapping_1_1_sensor_log.html</filename>
  </compound>
  <compound kind="class">
    <name>GMapping::SensorReading</name>
    <filename>class_g_mapping_1_1_sensor_reading.html</filename>
  </compound>
  <compound kind="struct">
    <name>GMapping::GridSlamProcessor::TNode</name>
    <filename>struct_g_mapping_1_1_grid_slam_processor_1_1_t_node.html</filename>
    <member kind="function">
      <type></type>
      <name>TNode</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_t_node.html</anchorfile>
      <anchor>a7742a4335c17d489467af9bbd21334ef</anchor>
      <arglist>(const OrientedPoint &amp;pose, double weight, TNode *parent=0, unsigned int childs=0)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~TNode</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_t_node.html</anchorfile>
      <anchor>a22b5e02c6b9dc2139f63652d8b0d57c5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable">
      <type>OrientedPoint</type>
      <name>pose</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_t_node.html</anchorfile>
      <anchor>a00ffad9f49ae467746aa211fbfaf4523</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>weight</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_t_node.html</anchorfile>
      <anchor>ab30d02945ab4cba987bbc428f87b3028</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>accWeight</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_t_node.html</anchorfile>
      <anchor>ab0c21a7a7fb16bd061f3b61cae8f55bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>TNode *</type>
      <name>parent</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_t_node.html</anchorfile>
      <anchor>aa96443473844405f417fb8170b3d7243</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const RangeReading *</type>
      <name>reading</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_t_node.html</anchorfile>
      <anchor>a574dd0cc9f28e7feb3d85a1ddb544d06</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>unsigned int</type>
      <name>childs</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_t_node.html</anchorfile>
      <anchor>a4c15378fddd812b0376ceb81c43be3e0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>unsigned int</type>
      <name>visitCounter</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_t_node.html</anchorfile>
      <anchor>aa5aa2b0e4bea309de90270cd56a2bd78</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>flag</name>
      <anchorfile>struct_g_mapping_1_1_grid_slam_processor_1_1_t_node.html</anchorfile>
      <anchor>a0d10763653ac0630632259c8c1e48e42</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>uniform_resampler</name>
    <filename>structuniform__resampler.html</filename>
    <templarg>class Particle</templarg>
    <templarg>class Numeric</templarg>
  </compound>
</tagfile>
