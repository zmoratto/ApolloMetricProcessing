#ifndef __VW_IMAGE_KRIGING_H__
#define __VW_IMAGE_KRIGING_H__

#include <list>
#include <utility>
#include <vw/Image/ImageView.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/BBox.h>
#include <vw/Math/LinearAlgebra.h>

namespace vw {

  // Variogram Model
  //
  // There could be more of these in the future, however I don't know about them.
  template <class PixelT>
  class PowVariogram {
    typedef Vector<double,CompoundNumChannels<PixelT>::value> result_type;
    result_type m_alpha;
    double m_beta, m_nugsq;

  public:
    PowVariogram( std::list<std::pair<Vector2, PixelT> > const& samples,
                  double beta = 1.5, double nug = 0) :
      m_beta(beta), m_nugsq( nug*nug), m_alpha(CompoundNumChannels<PixelT>::value) {
      size_t m_ndim = CompoundNumChannels<PixelT>::value;
      typedef std::pair<Vector2, PixelT> pair_list;
      typedef std::list<pair_list > sample_type;
      result_type num(m_ndim), denum(m_ndim);
      for ( typename sample_type::const_iterator i = samples.begin();
            i != samples.end(); i++ ) {
        typename sample_type::const_iterator j = i; j++;
        for ( ; j != samples.end(); j++ ) {
          double rb = norm_2_sqr(i->first-j->first);
          rb = pow(rb,0.5*m_beta);
          for ( size_t k = 0; k < m_ndim; k++ ) {
            num[k] += rb*(0.5*pow(i->second[k]-j->second[k],2) - m_nugsq);
            denum[k] += pow(rb,2);
          }
        }
      }
      for ( size_t k = 0; k < m_ndim; k++ )
        m_alpha[k] = num[k]/denum[k];
    }

    result_type operator()(const double r) const {
      return elem_sum(m_alpha*pow(r,m_beta),m_nugsq);
    }
  };


  /// Kriging View
  ///
  /// This view renders scatter point data into a single image.
  /// Scatter data is provided with std::list<std::pair<Vector2,T> >.
  /// Where T is the output pixel type.
  ///
  /// Largely inspired by p146 of Numerical Recipes: Third Edition
  template <class PixelT>
  class KrigingView : public ImageViewBase<KrigingView<PixelT> > {
    std::list<std::pair<Vector2, PixelT> > m_samples;
    size_t m_npt, m_ndim;
    BBox2i m_region;
    PowVariogram<PixelT> m_variogram;
    std::vector<Vector<double> > m_inv_v_y;

  public:
    typedef PixelT pixel_type;
    typedef PixelT result_type;
    typedef ProceduralPixelAccessor<KrigingView<PixelT> > pixel_accessor;

    KrigingView( std::list<std::pair<Vector2, PixelT> > const& samples,
                 BBox2i const& region ) :
      m_samples(samples), m_npt( samples.size() ),
      m_ndim( CompoundNumChannels<PixelT>::value ),
      m_region(region), m_variogram(samples), m_inv_v_y( m_ndim ) {

      // Allocating
      std::vector<Matrix<double> > v(m_ndim);
      std::vector<Vector<double> > y(m_ndim);
      for ( size_t k = 0; k < m_ndim; k++ ) {
        v[k].set_size(m_npt+1,m_npt+1);
        y[k].set_size(m_npt+1);
      }

      // Building both y and v
      typedef std::list<std::pair<Vector2, PixelT> > sample_type;
      size_t i_index = 0, j_index;
      for ( typename sample_type::const_iterator i = m_samples.begin();
            i != m_samples.end(); i++ ) {
        for ( size_t k = 0; k < m_ndim; k++ )
          y[k][i_index] = i->second[k];

        j_index = i_index + 1;
        typename sample_type::const_iterator j = i; j++;
        for ( ; j != m_samples.end(); j++ ) {
          Vector<double> variogram = m_variogram(norm_2(i->first - j->first));
          for ( size_t k = 0; k < m_ndim; k++ )
            v[k](i_index,j_index) = v[k](j_index,i_index) =
              variogram[k];
          j_index++;
        }
        for ( size_t k = 0; k < m_ndim; k++ )
          v[k](i_index,m_npt) = v[k](m_npt,i_index) = 1.0;
        i_index++;
      }
      for ( size_t k = 0; k < m_ndim; k++ ) {
        v[k](m_npt,m_npt) = 0.0;
        y[k][m_npt] = 0.0;
      }

      // Solving for inv_v_y for each output dimension
      for ( size_t k = 0; k < m_ndim; k++ )
        m_inv_v_y[k] = solve( v[k], y[k] );
    }

    inline int32 cols() const { return m_region.width(); }
    inline int32 rows() const { return m_region.height(); }
    inline int32 planes() const { return 1; }

    inline pixel_accessor origin() const { return pixel_accessor(*this, 0, 0); }

    inline result_type operator() (double i, double j, int32 p = 0) const {
      Vector2 xstar(i,j);
      xstar -= m_region.min();
      Matrix<double> vstar(m_ndim,m_npt+1);
      typedef std::list<std::pair<Vector2, PixelT> > sample_type;
      size_t i_index = 0;
      for ( typename sample_type::const_iterator i = m_samples.begin();
            i != m_samples.end(); i++ ) {
        select_col(vstar,i_index) = m_variogram(norm_2(xstar-i->first));
        i_index++;
      }
      for ( size_t k = 0; k < m_ndim; k++ )
        vstar(k,m_npt) = 1;
      result_type result;
      for ( size_t k = 0; k < m_ndim; k++ )
        result[k] = dot_prod(select_row(vstar,k),m_inv_v_y[k]);
      return result;
    }

    typedef KrigingView prerasterize_type;
    inline prerasterize_type prerasterize( BBox2i const& /*bbox*/ ) const {
      return *this;
    }
  };

  template <class PixelT>
  struct IsFloatingPointIndexable<KrigingView<PixelT> > : public true_type {};

  template <class PixelT>
  struct IsMultiplyAccessible<KrigingView<PixelT> > : public true_type {};

}

#endif//__VW_IMAGE_KRIGING_H__
