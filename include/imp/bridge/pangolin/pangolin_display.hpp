#ifndef PANGOLINDISPLAY_HPP
#define PANGOLINDISPLAY_HPP

#include <imp/core/size.hpp>
#include <imp/core/image.hpp>
#include <pangolin/display/display.h>

// #include <imp/cu_core/cu_image_gpu.cuh>

namespace imp
{

////------------------------------------------------------------------------------
//inline pangolin::View& setupPangolinCudaView(
//    const imp::Size2u& sz,
//    const std::string& title = "-")
//{
//  pangolin::View& container = pangolin::setupPangolinView(sz, title);
//  .SetBounds(0, 1.0f, 0, 1.0f);

//  // TODO

//  return container;
//}

//------------------------------------------------------------------------------
inline pangolin::View& setupPangolinView(
    const imp::Size2u& sz,
    const std::string& title = "-")
{
  // Create OpenGL window in single line
  pangolin::CreateWindowAndBind(title, sz.width(), sz.height());

  if (glewInit() != GLEW_OK )
  {
    LOG(ERROR) << "Unable to initialize GLEW." << std::endl;
  }

  glEnable(GL_DEPTH_TEST); // 3D Mouse handler requires depth testing to be enabled
  //! @todo (MWE) set OpenGL parameters... which glHint / glEnable options do we need?

  pangolin::View& container = pangolin::CreateDisplay()
      .SetBounds(0, 1.0f, 0, 1.0f);

  return container;
}

//------------------------------------------------------------------------------
inline void setupPangolinViewLayout(
    pangolin::View& container,
    int num_tiles=1, std::vector<float> aspect={})
{
  container.SetLayout(pangolin::LayoutEqual);
  for (int i=0; i<num_tiles; ++i)
  {
    pangolin::View& v = pangolin::CreateDisplay();
    if(static_cast<int>(aspect.size())>i)
    {
      v.SetAspect(aspect.at(i));
    }
    container.AddDisplay(v);
  }

  //! @todo (MWE) register keypresses, etc.
}

//------------------------------------------------------------------------------
inline void imshow(const imp::Image8uC1& im, const std::string& title="-")
{
  pangolin::View& container = imp::setupPangolinView(im.size(), title);
  imp::setupPangolinViewLayout(container, 1, {(float)im.width()/im.height()});
  pangolin::GlTexture tex8(im.width(), im.height(), GL_LUMINANCE8);

  while(!pangolin::ShouldQuit())
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f(1,1,1);

    if (container[0].IsShown())
    {
      container[0].Activate();
      tex8.Upload(im.data(), GL_LUMINANCE, GL_UNSIGNED_BYTE);
      tex8.RenderToViewportFlipY();
    }
    pangolin::FinishFrame();
  }
}

////------------------------------------------------------------------------------
//inline void imshow(const imp::cu::ImageGpu8uC1& im, const std::string& title="-")
//{
//  pangolin::View& container = imp::setupPangolinView(im.size(), title);
//  imp::setupPangolinViewLayout(container, 1, {(float)im.width()/im.height()});

//  container[0].SetDrawFunction


//}


////==============================================================================
//class PangolinDisplay
//{
//public:
//  PangolinDisplay();
//  ~PangolinDisplay();
//};

} // namespace imp

#endif // PANGOLINDISPLAY_HPP
