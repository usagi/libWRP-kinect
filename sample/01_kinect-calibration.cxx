#include <iostream>
#include <array>
#include <fstream>
#include <glog/logging.h>
#include <WonderRabbitProject/kinect.hpp>
//#include <boost/gil/extension/io/png_io.hpp>
#include <openctmpp.h>

namespace
{
  using namespace std;
  using namespace WonderRabbitProject;

  constexpr auto name = "01_kinect-calibration";
  
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

  using rgb8_t = std::array<uint8_t, 3>;

  LOG(INFO) << "get video";
  
  //auto vm = d.video_mode();
  auto vb = d.video_buffer<rgb8_t>();
  
  LOG(INFO) << "get depth";
  
  //auto dm = d.depth_mode();
  auto db = d.depth_buffer<uint16_t>();

  //LOG(INFO) << "{depth[(m,m,m)]} <-- {depth[raw(11bit)]}";
  //auto dp = kinect::calibrator<>::depth_m_vertices_from_raw(db);
  
  LOG(INFO) << "{ {{depth[(m,m,m)]}, color[unorm(r,g,b)]} } <-- {depth[raw(11bit)]}";
  auto dcp = kinect::calibrator<>::depth_m_colored_vertices_from_raw(db, vb);

  LOG(INFO) << "save model";
  ofstream o("model.vertices.tsv");
//  o <<
//    "x\ty\tz\n"
/*
    "ply\n"
    "format ascii 1.0\n"
    "element vertex " << dp.size() << "\n"
    <<
    "property float x\n"
    "property float y\n"
    "property float z\n"
    "end_header\n"
*/
//    ;
  
  for(const auto& p : dcp)
    o << p.vertex[0] << "\t"
      << p.vertex[1] << "\t"
      << p.vertex[2] << "\t"
      << p.color[0] << "\t"
      << p.color[1] << "\t"
      << p.color[2] << "\n"
      ;
  //ToDo: FacesとIndicesつける。とりあえずダミーでも
  
  /*
  LOG(INFO) << "save model.ctm";
  CTMexporter ctm;
  ctm.DefineMesh
  ( dp.data()->data()
  , CTMuint(dp.size())
  , nullptr
  , 0
  , nullptr
  );
  ctm.Save("model.ctm");
  */
}
catch (const std::exception& e)
{ std::cerr << "exception: " << e.what(); }
catch (...)
{ std::cerr << "unknown exception"; }

