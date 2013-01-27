#pragma once

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>
#include <mutex>
#include <string>
#include <array>

#ifdef _LOGGING_H_
  #define WRP_GLOG_ENABLED
#endif

namespace WonderRabbitProject { namespace kinect {
  
  namespace C {
    #include <libfreenect/libfreenect.h>
  }

  enum class SUBDEVICE
  { MOTOR  = C::FREENECT_DEVICE_MOTOR
  , CAMERA = C::FREENECT_DEVICE_CAMERA
  , AUDIO  = C::FREENECT_DEVICE_AUDIO
    
  , ALL = MOTOR | CAMERA | AUDIO
  };
  
  enum class RESOLUTION : uint32_t
  { QVGA = C::FREENECT_RESOLUTION_LOW
  , VGA  = C::FREENECT_RESOLUTION_MEDIUM
  , SXGA = C::FREENECT_RESOLUTION_HIGH
  };

  std::string to_string(RESOLUTION r)
  { switch(r){
#define WRP_TMP(a) case RESOLUTION :: a : return std::string( #a );
WRP_TMP(SXGA)
WRP_TMP(VGA)
WRP_TMP(QVGA)
#undef WRP_TMP
    default: return std::string();
    }
  }
  
  constexpr uint_fast16_t resolution_width(RESOLUTION r)
  { return
      r == RESOLUTION::SXGA ? 1280 :
      r == RESOLUTION::VGA  ?  640 :
      r == RESOLUTION::QVGA ?  320 :
                                 0 ;
  }
  
  enum class VIDEO : uint32_t
  { RGB             = C::FREENECT_VIDEO_RGB
  , BAYER           = C::FREENECT_VIDEO_BAYER
  , IR_8BIT         = C::FREENECT_VIDEO_IR_8BIT
  , IR_10BIT        = C::FREENECT_VIDEO_IR_10BIT
  , IR_10BIT_PACKED = C::FREENECT_VIDEO_IR_10BIT_PACKED
  , YUV_RGB         = C::FREENECT_VIDEO_YUV_RGB
  , YUV_RAW         = C::FREENECT_VIDEO_YUV_RAW
  };

  std::string to_string(VIDEO v)
  {
    switch(v){
#define WRP_TMP(a) case VIDEO :: a : return std::string( #a );
WRP_TMP(RGB)
WRP_TMP(BAYER)
WRP_TMP(IR_8BIT)
WRP_TMP(IR_10BIT)
WRP_TMP(IR_10BIT_PACKED)
WRP_TMP(YUV_RGB)
WRP_TMP(YUV_RAW)
#undef WRP_TMP
    default             : return std::string();
    }
  }

  enum class DEPTH : uint32_t
  { _11BIT        = C::FREENECT_DEPTH_11BIT
  , _10BIT        = C::FREENECT_DEPTH_10BIT
  , _11BIT_PACKED = C::FREENECT_DEPTH_11BIT_PACKED
  , _10BIT_PACKED = C::FREENECT_DEPTH_10BIT_PACKED
  , REGISTERED   = C::FREENECT_DEPTH_REGISTERED
  , MM           = C::FREENECT_DEPTH_MM
  };

  std::string to_string(DEPTH d)
  {
    switch(d){
#define WRP_TMP(a) case DEPTH :: a : return std::string( #a );
WRP_TMP(_11BIT)
WRP_TMP(_10BIT)
WRP_TMP(_11BIT_PACKED)
WRP_TMP(_10BIT_PACKED)
WRP_TMP(REGISTERED)
WRP_TMP(MM)
#undef WRP_TMP
    default             : return std::string();
    }
  }

  enum class LED
  { OFF              = C::LED_OFF
  , GREEN            = C::LED_GREEN
  , RED              = C::LED_RED
  , YELLOW           = C::LED_YELLOW
  , BLINK_GREEN      = C::LED_BLINK_GREEN
  , BLINK_RED_YELLOW = C::LED_BLINK_RED_YELLOW
  };

  std::string to_string(LED l)
  {
    switch(l){
#define WRP_TMP(a) case LED :: a : return std::string( #a );
WRP_TMP(OFF)
WRP_TMP(GREEN)
WRP_TMP(RED)
WRP_TMP(YELLOW)
WRP_TMP(BLINK_GREEN)
WRP_TMP(BLINK_RED_YELLOW)
#undef WRP_TMP
    default             : return std::string();
    }
  }
  enum class TILT
  { STOPPED = C::TILT_STATUS_STOPPED
  , LIMIT   = C::TILT_STATUS_LIMIT
  , MOVING  = C::TILT_STATUS_MOVING
  };

  std::string to_string(TILT t)
  {
    switch(t){
#define WRP_TMP(a) case TILT :: a : return std::string( #a );
WRP_TMP(STOPPED)
WRP_TMP(LIMIT)
WRP_TMP(MOVING)
#undef WRP_TMP
    default             : return std::string();
    }
  }

  enum class LOG_LEVEL
  { FATAL   = C::FREENECT_LOG_FATAL
  , ERROR   = C::FREENECT_LOG_ERROR
  , WARNING = C::FREENECT_LOG_WARNING
  , NOTICE  = C::FREENECT_LOG_NOTICE
  , INFO    = C::FREENECT_LOG_INFO
  , DEBUG   = C::FREENECT_LOG_DEBUG
  , SPEW    = C::FREENECT_LOG_SPEW
  , FLOOD   = C::FREENECT_LOG_FLOOD
  };

  std::string to_string(LOG_LEVEL t)
  {
    switch(t){
#define WRP_TMP(a) case LOG_LEVEL :: a : return std::string( #a );
WRP_TMP(FATAL)
WRP_TMP(ERROR)
WRP_TMP(WARNING)
WRP_TMP(NOTICE)
WRP_TMP(INFO)
WRP_TMP(DEBUG)
WRP_TMP(SPEW)
WRP_TMP(FLOOD)
#undef WRP_TMP
    default             : return std::string();
    }
  }
  
  struct frame_mode : protected C::freenect_frame_mode
  {
    frame_mode(C::freenect_frame_mode&& b)
      : C::freenect_frame_mode(std::move(b))
    {}

    inline RESOLUTION resolution_() const
    { return RESOLUTION(resolution); }
    
    inline decltype(bytes) bytes_() const
    { return bytes; }
    
    inline decltype(height) height_() const
    { return height; }
    
    inline decltype(width) width_() const
    { return width; }
    
    inline decltype(data_bits_per_pixel) data_bits_per_pixel_() const
    { return data_bits_per_pixel; }
 
    inline decltype(padding_bits_per_pixel) padding_bits_per_pixel_() const
    { return padding_bits_per_pixel; }

    inline decltype(framerate) framerate_() const
    { return framerate; }
  };

  struct video_frame_mode final : public frame_mode
  {
    video_frame_mode(C::freenect_frame_mode&& b)
      : frame_mode(std::move(b))
    {}
    VIDEO format() const { return VIDEO(video_format); }
  };

  struct depth_frame_mode final : public frame_mode
  {
    depth_frame_mode(C::freenect_frame_mode&& b)
      : frame_mode(std::move(b))
    {}
    DEPTH format() const { return DEPTH(depth_format); }
  };
  
  struct deleter final
  {
    using f_type = std::function<void()>;
    deleter(deleter&& t):f_(std::move(t.f_)){t.cancel();}
    deleter(f_type&& f):f_(std::move(f)){}
    deleter& operator=(const deleter&) = delete;
    deleter& operator=(deleter&&)      = delete;
    ~deleter(){ f_(); }
    inline void cancel(){ f_ = [](){}; }
    inline void invoke(){ f_(); cancel();}
  private:
    f_type f_;
  };

  struct context;
  template<unsigned, class>
  struct device;

  struct context final
  {
    using this_type = context;
    
    template<unsigned, class> friend class device;

    context(const this_type&) = delete;
    context(this_type&&)      = delete;
    this_type& operator=(const this_type&) = delete;
    this_type& operator=(this_type&&)      = delete;

    ~context()
    {
      #ifdef WRP_GLOG_ENABLED
        LOG(INFO) << "--> WRP::kinect::context::dtor";
        LOG(INFO) << "context address is " << std::hex << c;
      #endif
      if ( c )
      {
        C::freenect_shutdown(c);
        #ifdef WRP_GLOG_ENABLED
          LOG(INFO) << "freenect_shutdown done";
        #endif
      }
      #ifdef WRP_GLOG_ENABLED
        LOG(INFO) << "<-- WRP::kinect::context::dtor";
      #endif
    }

    inline void log_level(const LOG_LEVEL f) const
    {
      std::lock_guard<std::mutex> l(m);
      C::freenect_set_log_level(c, static_cast<C::freenect_loglevel>(f));
    }
    
    inline void select_sub_device(const SUBDEVICE f) const
    {
      std::lock_guard<std::mutex> l(m);
      C::freenect_select_subdevices(c, static_cast<C::freenect_device_flags>(f));
    }

    inline void process_events() const
    {
      /*
      #ifdef WRP_GLOG_ENABLED
        LOG(INFO) << "--> WRP::kinect::context::process_events";
      #endif
      */
      auto r = C::freenect_process_events(c);
      if( r != 0 )
      {
        #ifdef WRP_GLOG_ENABLED
          LOG(FATAL) << "freenect_process_events fail. code = " << r;
        #endif
        throw std::runtime_error(
          std::string("freenect_process_events fail. code = ")
          + std::to_string(r)
        );
      }
      /*
      #ifdef WRP_GLOG_ENABLED
        LOG(INFO) << "<-- WRP::kinect::context::process_events";
      #endif
      */
    }

    inline deleter thread_process_events() const
    {
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "--> WRP::kinect::context::thread_process_events";
      #endif
      auto is_running = new bool(true);
      auto t = new std::thread([is_running, this](){
        while(*is_running)
          this->process_events();
      });
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "<-- WRP::kinect::context::thread_process_events";
      #endif
      return deleter( [is_running, t](){
        *is_running = false;
        t->join();
        delete is_running;
        delete t;
      } );
    }
    
    static this_type& instance()
    {
      #ifdef WRP_GLOG_ENABLED
        LOG(INFO) << "--> WRP::kinect::context::instance";
      #endif
      if ( ! i )
      {
        std::lock_guard<decltype(m)> g(m);
        #ifdef WRP_GLOG_ENABLED
          LOG(INFO) << "lock_guard with mutex address is " << std::hex << &m;
        #endif
        if ( ! i )
        {
          i.reset(new this_type());
          #ifdef WRP_GLOG_ENABLED
          LOG(INFO)
            << "new instance; address of instance is "
            << std::hex << i.get();
          #endif
        }
      }
      #ifdef WRP_GLOG_ENABLED
        LOG(INFO) << "returning instance address is " << std::hex << i.get();
        LOG(INFO) << "<-- WRP::kinect::context::instance";
      #endif
      return *i;
    }
    
  private:
    context()
    {
      #ifdef WRP_GLOG_ENABLED
        LOG(INFO) << "--> WRP::kinect::context::ctor";
      #endif
      auto r = C::freenect_init(&c, nullptr);
      if ( r != 0 ) {
        #ifdef WRP_GLOG_ENABLED
          LOG(ERROR) << "freenect_init fail. code = " << r;
        #endif
        throw std::runtime_error(
          std::string("freenect_init fail. code = ")
          + std::to_string(r)
        );
      }
      #ifdef WRP_GLOG_ENABLED
        LOG(INFO) << "context address is " << std::hex << c;
        LOG(INFO) << "<-- WRP::kinect::context::ctor";
      #endif
    }

    inline C::freenect_context* internal_context()
    { return c; }

    C::freenect_context* c;
    
    static std::mutex m;
    static std::unique_ptr<this_type> i;
  };

  std::mutex context::m;
  std::unique_ptr<context> context::i;
  
  template
  < uint32_t DEFAULT_VIDEO_RESOLUTION = uint32_t(RESOLUTION::SXGA)
  , uint32_t DEFAULT_DEPTH_RESOLUTION = uint32_t(RESOLUTION::VGA)
  , uint32_t DEFAULT_VIDEO = uint32_t(VIDEO::RGB)
  , uint32_t DEFAULT_DEPTH = uint32_t(DEPTH::_11BIT)
  , unsigned DEFAULT_LED = unsigned(LED::OFF)
  >
  struct configurator final
  {
    static constexpr RESOLUTION
      default_video_resolution = RESOLUTION(DEFAULT_VIDEO_RESOLUTION),
      default_depth_resolution = RESOLUTION(DEFAULT_DEPTH_RESOLUTION);
    static constexpr VIDEO default_video = VIDEO(DEFAULT_VIDEO);
    static constexpr DEPTH default_depth = DEPTH(DEFAULT_DEPTH);
    static constexpr LED default_led = LED(DEFAULT_LED);
    RESOLUTION
      video_resolution = default_video_resolution,
      depth_resolution = default_depth_resolution;
    VIDEO video = default_video;
    DEPTH depth = default_depth;
    LED led = default_led;
  };

  template
  < unsigned DEVICE_ID = 0
  , class TCONF = configurator<>
  >
  struct device final
  {
    friend class context;
    using this_type = device<DEVICE_ID, TCONF>;
    
    static constexpr unsigned device_id = DEVICE_ID;
    using configurator_type = TCONF;
    
    device(const this_type&) = delete;
    device(this_type&&)      = delete;
    this_type& operator=(const this_type&) = delete;
    this_type& operator=(this_type&&)      = delete;
    
    ~device()
    {
      #ifdef WRP_GLOG_ENABLED
        LOG(INFO) << "--> WRP::kinect::device::dtor";
        LOG(INFO) << "device address is " << std::hex << d;
      #endif
      if ( d )
      {
        C::freenect_close_device(d);
        #ifdef WRP_GLOG_ENABLED
          LOG(INFO) << "freenect_shutdown done";
        #endif
      }
      #ifdef WRP_GLOG_ENABLED
        LOG(INFO) << "<-- WRP::kinect::device::dtor";
      #endif
    }

    inline video_frame_mode video_mode() const
    {
      return C::freenect_get_current_video_mode(d);
    }

    inline depth_frame_mode depth_mode() const
    {
      return C::freenect_get_current_depth_mode(d);
    }

    inline void video_mode(RESOLUTION r, VIDEO f)
    {
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "--> WRP::kinect::video_mode";
      #endif
      auto m = C::freenect_find_video_mode(
        C::freenect_resolution(r),
        C::freenect_video_format(f)
      );
      
      {
        auto result = C::freenect_set_video_mode(d, m);
        if( result != 0 )
        {
          throw std::runtime_error(
            std::string("freenect_set_video_mode fail. code = ")
            + std::to_string(result)
          );
        }
      }
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "freenect_set_video_mode succeed.";
      #endif
      conf.video_resolution = r;
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "update conf.video_resolution = " << to_string(r);
      #endif
      conf.video = f;
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "update conf.video = " << to_string(f);
      #endif
      for (auto& b : video_buffers){
        b.resize(m.bytes);
        std::fill(b.begin(), b.end(), 0);
      }
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "video_buffers prepared";
      #endif
      {
        auto result = C::freenect_set_video_buffer(
          d,
          reinterpret_cast<void*>( video_buffers[1].data() )
        );
        if( result != 0 )
        {
          throw std::runtime_error(
            std::string("freenect_set_video_buffer fail. code = ")
            + std::to_string(result)
          );
        }
      }
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "freenect_set_video_buffer succeed.";
      #endif
      C::freenect_set_video_callback(
        d,
        [](C::freenect_device*,void*,uint32_t){
          auto t = std::chrono::steady_clock::now();
          std::lock_guard<std::mutex> l(video_buffer_mutex);
          swap(video_buffers[1], video_buffers[0]);
          C::freenect_set_video_buffer(
            d,
            reinterpret_cast<void*>( video_buffers[1].data() )
          );
          //#ifdef WRP_GLOG_ENABLED
          //LOG(INFO) << "video_buffers swaped";
          //#endif
          video_time_ = t;
          //#ifdef WRP_GLOG_ENABLED
          //LOG(INFO) << "video time update to " << t.time_since_epoch().count();
          //#endif
        }
      );
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "<-- WRP::kinect::video_mode";
      #endif
    }

    inline void depth_mode(RESOLUTION r, DEPTH f)
    {
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "--> WRP::kinect::depth_mode";
      #endif
      auto m = C::freenect_find_depth_mode(
        C::freenect_resolution(r),
        C::freenect_depth_format(f)
      );
      
      {
        auto result = C::freenect_set_depth_mode(d, m);
        if( result != 0 )
        {
          throw std::runtime_error(
            std::string("freenect_set_depth_mode fail. code = ")
            + std::to_string(result)
          );
        }
      }
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "freenect_set_depth_mode succeed.";
      #endif
      conf.depth_resolution = r;
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "update conf.depth_resolution = " << to_string(r);
      #endif
      conf.depth = f;
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "update conf.depth = " << to_string(f);
      #endif
      for (auto& b : depth_buffers){
        b.resize(m.bytes);
        std::fill(b.begin(), b.end(), 80);
      }
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "depth_buffers prepared";
      #endif
      {
        auto result = C::freenect_set_depth_buffer(
          d,
          reinterpret_cast<void*>( depth_buffers[1].data() )
        );
        if( result != 0 )
        {
          throw std::runtime_error(
            std::string("freenect_set_depth_buffer fail. code = ")
            + std::to_string(result)
          );
        }
      }
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "freenect_set_depth_buffer succeed.";
      #endif
      C::freenect_set_depth_callback(
        d,
        [](C::freenect_device*,void*,uint32_t){
          auto t = std::chrono::steady_clock::now();
          std::lock_guard<std::mutex> l(video_buffer_mutex);
          swap(depth_buffers[0], depth_buffers[1]);
          C::freenect_set_depth_buffer(
            d,
            reinterpret_cast<void*>( depth_buffers[1].data() )
          );
          //#ifdef WRP_GLOG_ENABLED
          //LOG(INFO) << "depth_buffers swaped";
          //#endif
          depth_time_ = t;
          //#ifdef WRP_GLOG_ENABLED
          //LOG(INFO) << "depth time update to " << t.time_since_epoch().count();
          //#endif
        }
      );
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "<-- WRP::kinect::depth_mode";
      #endif
    }

    inline void led(LED l)
    {
      C::freenect_set_led(d, C::freenect_led_options(l));
      conf.led = l;
    }

    inline double tilt() const
    {
      C::freenect_update_tilt_state(d);
      return C::freenect_get_tilt_degs(
        C::freenect_get_tilt_state(d)
      );
    }

    inline void tilt(double angle_in_degrees)
    {
      C::freenect_set_tilt_degs(d, angle_in_degrees);
    }

    inline TILT tilt_status() const
    {
      C::freenect_update_tilt_state(d);
      return TILT(
        C::freenect_get_tilt_status(
          C::freenect_get_tilt_state(d)
        )
      );
    }

    inline deleter start_video()
    {
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "--> WRP::kinect::start_video";
      #endif
      if(video_is_started){
        #ifdef WRP_GLOG_ENABLED
        LOG(INFO) << "aleady started";
        #endif
        return { [](){} };
      }
      video_time_ = decltype(video_time_)();
      auto r = C::freenect_start_video(d);
      if ( r != 0 )
      {
        throw std::runtime_error(
          std::string("freenect_start_video fail. code = ")
          + std::to_string(r)
        );
      }
      video_is_started = true;
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "<-- WRP::kinect::start_video";
      #endif
      return { [this](){ this->stop_video(); } };
    }

    inline deleter start_depth()
    {
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "--> WRP::kinect::start_depth";
      #endif
      if(depth_is_started){
        #ifdef WRP_GLOG_ENABLED
        LOG(INFO) << "aleady started";
        #endif
        return { [](){} };
      }
      depth_time_ = decltype(depth_time_)();
      auto r = C::freenect_start_depth(d);
      if ( r != 0 )
      {
        throw std::runtime_error(
          std::string("freenect_start_depth fail. code = ")
          + std::to_string(r)
        );
      }
      depth_is_started = true;
      #ifdef WRP_GLOG_ENABLED
      LOG(INFO) << "<-- WRP::kinect::start_depth";
      #endif
      return { [this](){ this->stop_video(); } };
    }

    inline void stop_video()
    {
      if(!video_is_started)
        return;
      auto r = C::freenect_stop_video(d);
      if ( r != 0 )
      {
        throw std::runtime_error(
          std::string("freenect_stop_video fail. code = ")
          + std::to_string(r)
        );
      }
      video_is_started = false;
    }

    inline void stop_depth()
    {
      if(!depth_is_started)
        return;
      auto r = C::freenect_stop_depth(d);
      if ( r != 0 )
      {
        throw std::runtime_error(
          std::string("freenect_stop_depth fail. code = ")
          + std::to_string(r)
        );
      }
      depth_is_started = false;
    }

    inline deleter start()
    {
      start_video().cancel();
      start_depth().cancel();
      return { [this](){ this->stop(); } };
    }

    inline void stop()
    {
      stop_video();
      stop_depth();
    }

    inline void wait_first_data() const
    {
      while(
           ( video_is_started && video_time_.time_since_epoch().count() == 0 )
        || ( depth_is_started && depth_time_.time_since_epoch().count() == 0 )
      )
        process_events();
    }

    inline void wait_next_data() const
    {
      auto vt0 = video_time_;
      auto dt0 = depth_time_;
      while(
           ( video_is_started && video_time_ > vt0 )
        || ( depth_is_started && depth_time_ > dt0 )
      )
        process_events();
    }

    inline std::chrono::time_point<std::chrono::steady_clock>
    video_time() const
    { return video_time_; }

    inline std::chrono::time_point<std::chrono::steady_clock>
    depth_time() const
    { return depth_time_; }

    inline void reset(configurator_type&& new_conf)
    {
      video_mode(
        new_conf.video_resolution,
        new_conf.video
      );
      depth_mode(
        new_conf.depth_resolution,
        new_conf.depth
      );
      led( new_conf.led );
    }
    
    inline void reset()
    {
      reset(configurator_type());
    }

    inline const configurator_type& ref_configurator()
    {
      #ifdef WRP_GLOG_ENABLED
        LOG(INFO)
          << "--> WRP::kinect::device::ref_configurator; address of configurator is "
          << std::hex << &conf;
      #endif
      return conf;
    }

    inline context& ref_context() const
    {
      #ifdef WRP_GLOG_ENABLED
        LOG(INFO)
          << "--> WRP::kinect::device::ref_context; address of context is "
          << std::hex << &c;
      #endif
      return c;
    }

    inline void process_events() const
    { c.process_events(); }

    inline deleter thread_process_events() const
    { return std::move(c.thread_process_events()); }

    template<class T = uint8_t>
    std::vector<T> video_buffer()
    {
      std::lock_guard<std::mutex> l(video_buffer_mutex);
      auto b = reinterpret_cast<T*>(video_buffers[0].data());
      auto e = b + ( video_buffers[0].size() / sizeof(T) );
      return std::vector<T>(b, e);
    }

    template<class T = uint16_t>
    std::vector<T> depth_buffer()
    {
      std::lock_guard<std::mutex> l(depth_buffer_mutex);
      auto b = reinterpret_cast<T*>(depth_buffers[0].data());
      auto e = b + ( depth_buffers[0].size() / sizeof(T) );
      return std::vector<T>(b, e);
    }

    static this_type& instance()
    {
      #ifdef WRP_GLOG_ENABLED
        LOG(INFO) << "--> WRP::kinect::device::instance";
      #endif
      if ( ! i )
      {
        std::lock_guard<decltype(m)> g(m);
        #ifdef WRP_GLOG_ENABLED
          LOG(INFO) << "lock_guard with mutex address is " << std::hex << &m;
        #endif
        if ( ! i )
        {
          i.reset(new this_type());
          #ifdef WRP_GLOG_ENABLED
          LOG(INFO)
            << "new instance; address of instance is "
            << std::hex << i.get();
          #endif
        }
      }
      #ifdef WRP_GLOG_ENABLED
        LOG(INFO) << "returning instance address is " << std::hex << i.get();
        LOG(INFO) << "<-- WRP::kinect::device::instance";
      #endif
      return *i;
    }
    
  private:
    configurator_type conf;
    context& c;

    bool
      video_is_started = false,
      depth_is_started = false;

    device()
      : c(context::instance())
    {
      initialize_device();
    }

    inline void initialize_device()
    {
      auto r = C::freenect_open_device(
        c.internal_context(),
        &d,
        static_cast<int>(DEVICE_ID)
      );
      if ( r != 0 )
      {
        throw std::runtime_error(
          std::string("freenect_open_device fail. code = ")
          + std::to_string(r)
        );
      }
      reset();
    }
    
    static C::freenect_device* d;
    
    static std::mutex m;
    static std::unique_ptr<this_type> i;
    
    static std::array<std::vector<uint8_t>, 2>
      video_buffers,
      depth_buffers;

    static std::chrono::time_point<std::chrono::steady_clock>
      video_time_,
      depth_time_;
    
    static std::mutex
      video_buffer_mutex,
      depth_buffer_mutex;
  };

  template<unsigned DEVICE_ID, class TCONF>
  C::freenect_device* device<DEVICE_ID, TCONF>::d = nullptr;
  
  template<unsigned DEVICE_ID, class TCONF>
  std::mutex device<DEVICE_ID, TCONF>::m;

  template<unsigned DEVICE_ID, class TCONF>
  std::unique_ptr<device<DEVICE_ID, TCONF>> device<DEVICE_ID, TCONF>::i;

  template<unsigned DEVICE_ID, class TCONF>
  std::array<std::vector<uint8_t>, 2> device<DEVICE_ID, TCONF>::video_buffers;
  
  template<unsigned DEVICE_ID, class TCONF>
  std::array<std::vector<uint8_t>, 2> device<DEVICE_ID, TCONF>::depth_buffers;
  
  template<unsigned DEVICE_ID, class TCONF>
  std::chrono::time_point<std::chrono::steady_clock>
  device<DEVICE_ID, TCONF>::video_time_;
  
  template<unsigned DEVICE_ID, class TCONF>
  std::chrono::time_point<std::chrono::steady_clock>
  device<DEVICE_ID, TCONF>::depth_time_;
  
  template<unsigned DEVICE_ID, class TCONF>
  std::mutex device<DEVICE_ID, TCONF>::video_buffer_mutex;

  template<unsigned DEVICE_ID, class TCONF>
  std::mutex device<DEVICE_ID, TCONF>::depth_buffer_mutex;

} }

#ifdef WRP_GLOG_ENABLED
  #undef WRP_GLOG_ENABLED
#endif

