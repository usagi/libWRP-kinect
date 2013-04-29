#include <iostream>

#include <WonderRabbitProject/glog.hpp>
#include <WonderRabbitProject/kinect.hpp>

#include <boost/gil/extension/io/png_io.hpp>

namespace
{
  using namespace std;
  using namespace WonderRabbitProject;

  constexpr auto name = "00_kinect-test";
}

int main() try
{
  glog::initialize(glog::OUT::STDERR, name);
  
  using device_type = kinect::device<>;
  auto& d = device_type::instance();

  d.video_mode(kinect::RESOLUTION::SXGA, kinect::VIDEO::RGB);
  d.depth_mode(kinect::RESOLUTION::VGA, kinect::DEPTH::_11BIT);

  auto device_stopper = d.start();
  auto thread_stopper = d.thread_process_events();
  
  d.wait_first_data();

  {
    using namespace boost::gil;
    
    L(INFO, "save video");
    
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
    
    L(INFO, "save depth");
    
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
{ L(FATAL, "exception: " << e.what()); return -1; }
catch (...)
{ L(FATAL, "unknown exception"); return -2; }

