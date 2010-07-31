/// This is a lazy mod on RANSAC that disallows matrices that couldn't
/// happen on apollo.

#ifndef __VW_MATH_RANSAC_MOD_H__
#define __VW_MATH_RANSAC_MOD_H__

#include <vw/Math/Vector.h>
#include <vw/Core/Log.h>

namespace vw {
  namespace math {

    /// RANSAC Driver class
    template <class FittingFuncT, class ErrorFuncT>
      class RandomSampleConsensusMod {
      const FittingFuncT& m_fitting_func;
      const ErrorFuncT& m_error_func;
      double m_inlier_threshold;

      // Returns the number of inliers for a given threshold.
      template <class ContainerT1, class ContainerT2>
        unsigned num_inliers(typename FittingFuncT::result_type const& H,
                             std::vector<ContainerT1> const& p1,
                             std::vector<ContainerT2> const& p2) const {
        unsigned result = 0;
        for (unsigned i=0; i<p1.size(); i++) {
          if (m_error_func(H,p1[i],p2[i]) < m_inlier_threshold)
            ++result;
        }
        return result;
      }

      /// \cond INTERNAL
      // Utility Function: Pick N UNIQUE, random integers in the range [0, size]
      inline void _vw_get_n_unique_integers(unsigned int size, unsigned n, int* samples) const {
        VW_ASSERT(size >= n, ArgumentErr() << "Not enough samples (" << n << " / " << size << ")\n");

        for (unsigned i=0; i<n; ++i) {
          bool done = false;
          while (!done) {
            samples[i] = (int)(((double)random() / (double)RAND_MAX) * size);
            done = true;
            for (unsigned j = 0; j < i; j++)
              if (samples[i] == samples[j])
                done = false;
          }
        }
      }
      /// \endcond

    public:

      // Returns the list of inlier indices.
      template <class ContainerT1, class ContainerT2>
        void inliers(typename FittingFuncT::result_type const& H,
                     std::vector<ContainerT1> const& p1, std::vector<ContainerT2> const& p2,
                     std::vector<ContainerT1> &inliers1, std::vector<ContainerT2> &inliers2) const {

        inliers1.clear();
        inliers2.clear();

        for (unsigned int i=0; i<p1.size(); i++) {
          if (m_error_func(H,p1[i],p2[i]) < m_inlier_threshold) {
            inliers1.push_back(p1[i]);
            inliers2.push_back(p2[i]);
          }
        }
      }

      // Returns the list of inlier indices.
      template <class ContainerT1, class ContainerT2>
        std::vector<int> inlier_indices(typename FittingFuncT::result_type const& H,
                                        std::vector<ContainerT1> const& p1,std::vector<ContainerT2> const& p2) const {
        std::vector<int> result;
        for (unsigned int i=0; i<p1.size(); i++)
          if (m_error_func(H,p1[i],p2[i]) < m_inlier_threshold)
            result.push_back(i);
        return result;
      }

    RandomSampleConsensusMod(FittingFuncT const& fitting_func, ErrorFuncT const& error_func, double inlier_threshold)
      : m_fitting_func(fitting_func), m_error_func(error_func), m_inlier_threshold(inlier_threshold) {}

      template <class ContainerT1, class ContainerT2>
        typename FittingFuncT::result_type operator()(std::vector<ContainerT1> const& p1,
                                                      std::vector<ContainerT2> const& p2,
                                                      int ransac_iterations = 0) const {
        // check consistency
        VW_ASSERT( p1.size() == p2.size(),
                   RANSACErr() << "RANSAC Error.  data vectors are not the same size." );
        VW_ASSERT( p1.size() != 0,
                   RANSACErr() << "RANSAC Error.  Insufficient data.\n");
        VW_ASSERT( p1.size() >= m_fitting_func.min_elements_needed_for_fit(p1[0]),
                   RANSACErr() << "RANSAC Error.  Not enough potential matches for this fitting funtor. ("<<p1.size() << "/" << m_fitting_func.min_elements_needed_for_fit(p1[0]) << ")\n");

        unsigned inliers_max = 0;
        typename FittingFuncT::result_type H;
        typename FittingFuncT::result_type H_max;

        /////////////////////////////////////////
        // First part:
        //   1. choose N points at random
        //   2. find a fit for those N points
        //   3. check for consensus
        //   4. keep fit with best consensus so far
        /////////////////////////////////////////

        // Seed random number generator
        srandom((unsigned int) clock());

        // This is a rough value, but it seems to produce reasonably good results.
        if (ransac_iterations == 0)
          ransac_iterations = p1.size() * 2;

        int n = m_fitting_func.min_elements_needed_for_fit(p1[0]);
        std::vector<ContainerT1> try1(n);
        std::vector<ContainerT2> try2(n);
        boost::scoped_array<int> random_indices(new int[n]);

        for (int iteration=0; iteration < ransac_iterations; ++iteration) {
          // Get four points at random, taking care not
          // to select the same point twice.
          _vw_get_n_unique_integers(p1.size(), n, random_indices.get());

          for (int i=0; i < n; ++i) {
            try1[i] = p1[random_indices[i]];
            try2[i] = p2[random_indices[i]];
          }

          // Compute the fit using these samples
          H = m_fitting_func(try1, try2);

          // The MODIFICATION!
          Vector2 trans(H(0,2),H(1,2)); // (There should be translation)
          if ( norm_2(trans) < 300 ) {
            iteration++;
            continue;
          }
          if ( (H(0,0)>0)^(H(1,1)>0) ) {
            // Can't be flipped
            iteration++;
            continue;
          }

          // Compute consensuss
          unsigned n_inliers = num_inliers(H, p1, p2);

          // Keep best consensus
          if (n_inliers > inliers_max) {
            inliers_max = n_inliers;
            H_max = H;
          }
        }

        if (inliers_max < m_fitting_func.min_elements_needed_for_fit(p1[0])) {
          vw_throw( RANSACErr() << "RANSAC was unable to find a fit that matched the supplied data." );
        }

        ////////////////////////////////////
        // Second part:
        //    1. find all inliers the best fit
        //    2. re-estimate the fit using all inliers
        //    3. repeat until # of inliers stabilizes
        ///////////////////////////////////
        unsigned int num_old = 0;
        inliers(H_max, p1, p2, try1, try2 );
        while( try1.size() > num_old ){
          num_old = try1.size();
          H = m_fitting_func(try1, try2, H_max); // Seeding with best solution
          inliers( H, p1, p2, try1, try2 );
        }
        // For debugging
        vw_out(InfoMessage, "interest_point") << "\nRANSAC Summary:" << std::endl;
        vw_out(InfoMessage, "interest_point") << "\tFit = " << H << std::endl;
        vw_out(InfoMessage, "interest_point") << "\tInliers / Total  = " << try1.size() << " / " << p1.size() << "\n\n";
        return H;
      }

    };

  }} // namespace vw::math

#endif // __MATH_RANSAC_H__
