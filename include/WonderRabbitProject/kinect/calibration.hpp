#pragma once

#include <cstddef>
#include <array>
#include <vector>
#include <algorithm>

namespace WonderRabbitProject { namespace kinect {
  
  class calibrator_conf1_default final
  {
    calibrator_conf1_default();
  public:
#define WRP_TMP(a,b) static constexpr double a () { return b ; }
    WRP_TMP(fx_rgb,  5.2921508098293293e+02)
    WRP_TMP(fy_rgb,  5.2556393630057437e+02)
    WRP_TMP(cx_rgb,  3.2894272028759258e+02)
    WRP_TMP(cy_rgb,  2.6748068171871557e+02)
    WRP_TMP(k1_rgb,  2.6451622333009589e-01)
    WRP_TMP(k2_rgb, -8.3990749424620825e-01)
    WRP_TMP(k3_rgb, -1.9922302173693159e-03)
    WRP_TMP(p1_rgb,  1.4371995932897616e-03)
    WRP_TMP(p2_rgb,  9.1192465078713847e-01)
    WRP_TMP(fx_d  ,  5.9421434211923247e+02)
    WRP_TMP(fy_d  ,  5.9104053696870778e+02)
    WRP_TMP(cx_d  ,  3.3930780975300314e+02)
    WRP_TMP(cy_d  ,  2.4273913761751615e+02)
    WRP_TMP(k1_d  , -2.6386489753128833e-01)
    WRP_TMP(k2_d  ,  9.9966832163729757e-01)
    WRP_TMP(k3_d  , -7.6275862143610667e-04)
    WRP_TMP(p1_d  ,  5.0350940090814270e-03)
    WRP_TMP(p2_d  , -1.3053628089976321e+00)
#undef WRP_TMP
    static constexpr std::array<std::array<double, 3>, 3>
    R()
    { return 
      {{
        {{  9.9984628826577793e-01
        ,   1.2635359098409581e-03
        ,  -1.7487233004436643e-02
        }},
        {{ -1.4779096108364480e-03
        ,   9.9992385683542895e-01
        ,  -1.2251380107679535e-02
        }},
        {{  1.7470421412464927e-02
        ,   1.2275341476520762e-02
        ,   9.9977202419716948e-01
        }}
      }};
    }
    static constexpr std::array<double, 3>
    T()
    { return 
      {{ 1.9985242312092553e-02
      , -7.4423738761617583e-04
      ,-1.0916736334336222e-02
      }};
    }
  };

  class calibrator_conf2_default final
  {
    calibrator_conf2_default();
  public:
    static constexpr double depth_m_from_raw_a() { return -0.0030711016; }
    static constexpr double depth_m_from_raw_b() { return -3.3309495161; }
  };

  template
  < class TCONF1  = calibrator_conf1_default
  , class TCONF2  = calibrator_conf2_default
  , class TDEPTH_RAW  = uint16_t
  , class TDEPTH_M    = double
  , class TDEPTH_M_VERTEX_ELEMENT = float
  , size_t TDEPTH_M_VERTEX_SIZE   = 3
  , class TVIDEO_PIXEL_UNORM_COORDINATE_ELEMENT = float
  >
  class calibrator final
  {
    calibrator();
  public:
    using conf1_type = TCONF1;
    using conf2_type = TCONF2;
    using depth_raw_type = TDEPTH_RAW;
    using depth_m_type   = TDEPTH_M;
    using depth_m_vertex_element_type = TDEPTH_M_VERTEX_ELEMENT;
    static constexpr size_t depth_m_vertex_size = TDEPTH_M_VERTEX_SIZE;
    using video_pixel_unorm_coordinate_element_type
      = TVIDEO_PIXEL_UNORM_COORDINATE_ELEMENT;
    
    using depth_raw_value  = uint16_t;
    using depth_raw_values = std::vector<depth_raw_value>;
    using depth_m_value    = depth_m_type;
    using depth_m_values   = std::vector<depth_m_value>;
    
    using depth_m_vertex   = std::array
      < depth_m_vertex_element_type
      , depth_m_vertex_size
      >;
    using depth_m_vertices = std::vector<depth_m_vertex>;

    using video_pixel_unorm_coordinate = std::array
      < video_pixel_unorm_coordinate_element_type
      , 2
      >;
    
    using video_pixel_index_coordinate = std::array
      < size_t
      , 2
      >;
    
    // depth[m] <-- depth[raw(11bit)]
    static
    depth_m_value depth_m_from_raw(const depth_raw_value v)
    { return 1.0 / ( v
                     * TCONF2::depth_m_from_raw_a()
                     + TCONF2::depth_m_from_raw_b()
                   );
    }

    // { depth[m] } <-- { depth[raw(11bit] }
    static
    depth_m_values depth_m_from_raw(const depth_raw_values& vs)
    {
      depth_m_values r(vs.size());
      std::transform
      ( std::begin(vs), std::end(vs)
      , std::begin(r)
      , [](const depth_raw_values::value_type v)
        { return depth_m_from_raw(v); }
      );
      return r;
    }

    // { depth[(m,m,m)] } <-- { depth[raw(11bit)] }
    // ToDo: impl overload Boost.GIL view version
    // ToDo: impl overload run-time width param version
    template<size_t TTWIDTH = 640>
    static
    depth_m_vertices depth_m_vertices_from_raw(const depth_raw_values& vs)
    {
      depth_m_vertices r(vs.size());
      
      const size_t height = vs.size() / TTWIDTH;
      
      for(size_t y = 0; y < height; ++y)
      {
        const size_t yi = y * TTWIDTH;
        for(size_t x = 0; x < TTWIDTH; ++x)
        {
          const size_t i = x + yi;
          using t = depth_m_vertex_element_type;
          const t z_ = t(depth_m_from_raw(vs[i]));
          r[i] =
            {{ ( t(x) - t(conf1_type::cx_d()) ) * z_ / t(conf1_type::fx_d)
            ,  ( t(y) - t(conf1_type::cy_d()) ) * z_ / t(conf1_type::fy_d)
            ,  z_
            }};
        }
      }
      
      return r;
    }
    
    // video[(x-unorm,y-unorm)] <-- depth[(m,m,m)]
    template<size_t TTWIDTH = 640, size_t TTHEIGHT = 480>
    static
    video_pixel_unorm_coordinate
    video_pixel_unorm_coordinate_from_depth_m_vertex
    (const depth_m_vertex& v)
    {
      using t = depth_m_vertex_element_type;
      const t x_ = conf1_type::R()[0][0] * v[0]
                 + conf1_type::R()[0][1] * v[1]
                 + conf1_type::R()[0][2] * v[2]
                 + conf1_type::R()[0]
                 ;
      const t y_ = conf1_type::R()[1][0] * v[0]
                 + conf1_type::R()[1][1] * v[1]
                 + conf1_type::R()[1][2] * v[2]
                 + conf1_type::R()[1]
                 ;
      const t z_ = conf1_type::R()[2][0] * v[0]
                 + conf1_type::R()[2][1] * v[1]
                 + conf1_type::R()[2][2] * v[2]
                 + conf1_type::R()[2]
                 ;
      const t x__
        = (x_ * t(conf1_type::fx_rgb()) / z_ + t(conf1_type::cx_rgb)) * TTWIDTH;
      const t y__
        = (y_ * t(conf1_type::fy_rgb()) / z_ + t(conf1_type::cy_rgb)) * TTHEIGHT;
      return
      //{{ std::min(std::max(x__,0), 1 )
      //,  std::min(std::max(y__,0), 1 )
      //}}
      {{ x__, y__ }}
      ;
    }

    // video[(x-unorm,y-unorm)] <-- depth[(m,m,m)]
    template<size_t TTWIDTH = 640, size_t TTHEIGHT = 480>
    static
    video_pixel_index_coordinate
    video_pixel_index_coordinate_from_depth_m_vertex
    (const depth_m_vertex& v)
    {
      using t = depth_m_vertex_element_type;
      const t x_ = conf1_type::R()[0][0] * v[0]
                 + conf1_type::R()[0][1] * v[1]
                 + conf1_type::R()[0][2] * v[2]
                 + conf1_type::R()[0]
                 ;
      const t y_ = conf1_type::R()[1][0] * v[0]
                 + conf1_type::R()[1][1] * v[1]
                 + conf1_type::R()[1][2] * v[2]
                 + conf1_type::R()[1]
                 ;
      const t z_ = conf1_type::R()[2][0] * v[0]
                 + conf1_type::R()[2][1] * v[1]
                 + conf1_type::R()[2][2] * v[2]
                 + conf1_type::R()[2]
                 ;
      const auto x__
        = size_t(x_ * t(conf1_type::fx_rgb()) / z_ + t(conf1_type::cx_rgb));
      const auto y__
        = size_t(y_ * t(conf1_type::fy_rgb()) / z_ + t(conf1_type::cy_rgb));
      return
      //{{ std::min(std::max(x__,0), 640 )
      //,  std::min(std::max(y__,0), 480 )
      //}}
      {{ x__, y__ }}
      ;
    }

  };
  
} }

