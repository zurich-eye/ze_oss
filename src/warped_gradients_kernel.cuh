#ifndef IMP_CU_K_WARPED_GRADIENTS_CUH
#define IMP_CU_K_WARPED_GRADIENTS_CUH

#include <cuda_runtime_api.h>
#include <imp/core/types.hpp>
#include <imp/cuda_toolkit/helper_math.h>
#include <imp/cu_core/cu_pinhole_camera.cuh>
#include <imp/cu_core/cu_matrix.cuh>
#include <imp/cu_core/cu_se3.cuh>


namespace imp {
namespace cu {

//------------------------------------------------------------------------------
template<typename Pixel>
__global__ void k_warpedGradients(Pixel* ix, Pixel* it, size_t stride,
                                  std::uint32_t width, std::uint32_t height,
                                  //std::uint32_t roi_x, std::uint32_t roi_y,
                                  Texture2D i1_tex, Texture2D i2_tex, Texture2D u0_tex)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x /*+ roi_x*/;
  const int y = blockIdx.y*blockDim.y + threadIdx.y /*+ roi_y*/;
  const int c = y*stride+x;

  if (x<width && y<height)
  {
    float u0 = tex2DFetch<float>(u0_tex, x,y);
    float wx = x+u0;

    float bd = .5f;
    if ((wx < bd) || (x < bd) || (wx > width-bd-1) || (x > width-bd-1) ||
        (y<bd) || (y>height-bd-1))
    {
      ix[c] =  0.0f;
      it[c] =  0.0f;
    }
    else
    {
      Pixel i1_c, i2_w_c, i2_w_m, i2_w_p;

      tex2DFetch(i1_c, i1_tex, x, y);

      tex2DFetch(i2_w_c, i2_tex, wx, y);
      tex2DFetch(i2_w_m, i2_tex, wx-.5f, y);
      tex2DFetch(i2_w_p, i2_tex, wx+.5f, y);

      // spatial gradient on warped image
      ix[c] = i2_w_p - i2_w_m;
      // temporal gradient between the warped moving image and the fixed image
      it[c] = i2_w_c - i1_c;
    }

  }
}

//------------------------------------------------------------------------------
template<typename Pixel>
__global__ void k_warpedGradientsEpipolarConstraint(
    Pixel* iw, Pixel* ix, Pixel* it, size_t stride,
    std::uint32_t width, std::uint32_t height,
    cu::PinholeCamera cam1, cu::PinholeCamera cam2,
    const cu::Matrix3f F_mov_fix, const cu::SE3<float> T_mov_fix,
    Texture2D i1_tex, Texture2D i2_tex, Texture2D u0_tex,
    Texture2D depth_proposal_tex)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x /*+ roi_x*/;
  const int y = blockIdx.y*blockDim.y + threadIdx.y /*+ roi_y*/;
  const int c = y*stride+x;

  if (x<width && y<height)
  {
    // compute epipolar geometry
    float mu = tex2DFetch<float>(depth_proposal_tex, x, y);

    //    float sigma = sqrtf(depth_proposal_sigma2_tex.fetch<float>(x,y));
    float2 px_ref = make_float2((float)x, (float)y);
    float2 px_mean = px_ref;

    float3 px_ref_w = cam1.cam2world(px_ref);
    float3 f_ref = ::normalize(px_ref_w);
    if (std::fabs(mu) > 1e-3f)
      px_mean = cam2.world2cam(T_mov_fix * (f_ref*mu));

    //Vec32fC2 px_p3s = cam2.world2cam(T_mov_fix * (f_ref*(mu + 3.f*sigma)));
    //float3 px_mean_h = make_float3(px_mean.x, px_mean.y, 1.0f);
    float3 px_ref_h = make_float3(px_ref.x, px_ref.y, 1.f);

    //    float3 epi_line = F_ref_cur*px_ref_h;
    //    float3 epi_line = F_ref_cur*px_mean_h;
    //float2 epi_line_slope = make_float2(epi_line.y, -epi_line.x);
    //float2 epi_vec = ::normalize(epi_line_slope);

    float3 epi_line = F_mov_fix*px_ref_h;
    // from line equation: ax+by+c=0 -> y=(-c-ax)/b -> k=-a/b
    float2 epi_line_slope = make_float2(1.0f, -epi_line.x/epi_line.y);
    float2 epi_vec = ::normalize(epi_line_slope);

#if 0
    if(x==20 && y==20)
    {
      printf("mu: %f\n", mu);
      printf("cam: %f, %f; %f, %f\n", cam1.fx(), cam1.fy(), cam1.cx(), cam1.cy());
      printf("F: %e, %e, %e; %e, %e, %e; %e, %e, %e\n",
             F_mov_fix(0,0), F_mov_fix(0,1), F_mov_fix(0,2),
             F_mov_fix(1,0), F_mov_fix(1,1), F_mov_fix(1,2),
             F_mov_fix(2,0), F_mov_fix(2,1), F_mov_fix(2,2));

      printf("px_ref_w: %f, %f, %f\n", px_ref_w.x, px_ref_w.y, px_ref_w.z);

      printf("f_ref: %f, %f %f\n", f_ref.x, f_ref.y, f_ref.z);
      float3 bla = T_mov_fix * (f_ref*mu);
      printf("bla: %f, %f %f\n", bla.x, bla.y, bla.z);

      printf("px_ref: %f, %f\n", px_ref.x, px_ref.y);
      printf("px_mean: %f, %f\n", px_mean.x, px_mean.y);

      printf("epi_line: %f, %f %f\n", epi_line.x, epi_line.y, epi_line.z);
      printf("epi_line_slope: %f, %f (length: %f)\n", epi_line_slope.x, epi_line_slope.y, ::length(epi_line_slope));
      printf("epi_vec: %f, %f (length: %f)\n\n", epi_vec.x, epi_vec.y, ::length(epi_vec));
    }
#endif

    float u0 = tex2DFetch<float>(u0_tex, x,y);
    float2 px_u0 = px_mean + epi_vec*u0; // assuming that epi_vec is the unit vec
    float2 px_u0_p = px_u0 + 0.5f*epi_vec;
    float2 px_u0_m = px_u0 - 0.5f*epi_vec;

    float bd = .5f;
    // check if current mean projects in image /*and mark if not*/
    // and if warped point is within a certain image area
    if (
        (px_mean.x > width-bd-1) || (px_mean.y > height-bd-1) || (px_mean.x < bd) || (px_mean.y < bd) ||
        (px_u0.x < bd) || (x < bd) || (px_u0.y > width-bd-1) || (x > width-bd-1) ||
        (px_u0.y < bd) || (px_u0.y > height-bd-1) || (y < bd) || (y > height-bd-1))
    {
      ix[c] = 0.0f;
      it[c] = 0.0f;
      iw[c] = 0.0f;
    }
    else
    {
      Pixel i1_c, i2_w_c, i2_w_m, i2_w_p;


      tex2DFetch(i1_c, i1_tex, x, y);

      tex2DFetch(i2_w_c, i2_tex, px_u0.x, px_u0.y-.5f*epi_vec.y);
      tex2DFetch(i2_w_m, i2_tex, px_u0.x-.5f*epi_vec.x, px_u0.y-.5f*epi_vec.y);
      tex2DFetch(i2_w_p, i2_tex, px_u0.x+.5f*epi_vec.x, px_u0.y+.5f*epi_vec.y);

#if 0
      i2_tex.fetch(i2_w_c, px_ref.x+epi_vec.x*u0, px_ref.y);
      i2_tex.fetch(i2_w_m, px_ref.x+epi_vec.x*u0-epi_vec.x*0.5f, px_ref.y-0.5f*epi_vec.y);
      i2_tex.fetch(i2_w_p, px_ref.x+epi_vec.x*u0+epi_vec.x*0.5f, px_ref.y+0.5f*epi_vec.y);
#endif
#if 0
      float wx = x+u0;
      i2_tex.fetch(i2_w_c, wx, y);
      i2_tex.fetch(i2_w_m, wx-epi_vec.x*0.5f, y-epi_vec.y*0.5f);
      i2_tex.fetch(i2_w_p, wx+epi_vec.x*0.5f, y+epi_vec.y*0.5f);
#endif
#if 0
      float wx = x+u0;
      i2_tex.fetch(i2_w_c, wx, y);
      i2_tex.fetch(i2_w_m, wx-0.5f, y);
      i2_tex.fetch(i2_w_p, wx+0.5f, y);
#endif


      // spatial gradient on warped image
      ix[c] = i2_w_p - i2_w_m;
      // temporal gradient between the warped moving image and the fixed image
      it[c] = i2_w_c - i1_c;
      // warped image
      iw[c] = i2_w_c;
    }
  }
}


} // namespace cu
} // namespace imp



#endif // IMP_CU_K_WARPED_GRADIENTS_CUH

