#include <iostream>
#include <glog/logging.h>

#include <WonderRabbitProject/kinect.hpp>

#include <boost/gil/extension/io/png_io.hpp>

namespace
{
  using namespace std;
  using namespace WonderRabbitProject;

  constexpr auto name = "00_kinect-test";
  
  enum class GLOG_OUT
  {
    DEFAULT,
    STDERR
  };
  
  void glog_initialize(const GLOG_OUT out)
  {
    google::InitGoogleLogging(name);
    switch(out)
    {
    case GLOG_OUT::STDERR:
      google::LogToStderr();
    default:;
    }
    LOG(INFO) << "glog initialized";
  }
}

int main() try
{
  glog_initialize(GLOG_OUT::STDERR);
  
  using device_type = kinect::device<>;
  auto& d = device_type::instance();

  d.video_mode(kinect::RESOLUTION::SXGA, kinect::VIDEO::RGB);
  d.depth_mode(kinect::RESOLUTION::VGA, kinect::DEPTH::_11BIT);

  auto device_stopper = d.start();
  auto thread_stopper = d.thread_process_events();
  
  d.wait_first_data();

  {
    using namespace boost::gil;
    
    LOG(INFO) << "save video";
    
    auto vm = d.video_mode();
    auto vb = d.video_buffer<rgb8_pixel_t>();
    rgb8_view_t vv = interleaved_view(
      vm.width_(), vm.height_(),
      vb.data()  , vm.width_() * sizeof(rgb8_pixel_t)
    );
    png_write_view("video.png", vv);
  }
  {
    using namespace boost::gil;
    
    LOG(INFO) << "save depth";
    
    auto dm = d.depth_mode();
    auto db = d.depth_buffer<gray16_pixel_t>();
    gray16_view_t dv = interleaved_view(
      dm.width_(), dm.height_(),
      db.data()  , dm.width_() * sizeof(gray16_pixel_t)
    );
    transform_pixels(
      dv, dv,
      []( const gray16_pixel_t& v ){ return v << 5; }
    );
    png_write_view("depth.png", dv);
  }
}
catch (const std::exception& e)
{ std::cerr << "exception: " << e.what(); }
catch (...)
{ std::cerr << "unknown exception"; }

