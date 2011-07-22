
#ifndef __ANN_MATCHER_H__
#define __ANN_MATCHER_H__

#include <boost/foreach.hpp>
#include <vw/InterestPoint/Matcher.h>
#include <ANN/ANN.h>

namespace vw {

  class InterestPointMatcherANN {
    double m_threshold;

  public:

    InterestPointMatcherANN( double threshold = 0.5 ) : m_threshold(threshold) {}

    /// Given two lists of interest points, this routine returns the two
    /// losts of matching interest points.
    template <class ListT, class MatchListT>
    void operator()( ListT const& ip1, ListT const& ip2,
                     MatchListT& matched_ip1, MatchListT& matched_ip2,
                     bool /*bidirectional*/ = false,
                     const ProgressCallback &progress_callback = ProgressCallback::dummy_instance() ) const {
      typedef typename ListT::const_iterator IterT;

      matched_ip1.clear(); matched_ip2.clear();
      if (!ip1.size() || !ip2.size()) {
        vw_out(InfoMessage,"interest_point") << "No points to match, exiting\n";
        progress_callback.report_finished();
        return;
      }

      // Allocating negative and positive ANN
      double eps = 0.0;
      ANNpointArray ann_pts;
      ann_pts = annAllocPts( ip2.size(), ip2.begin()->size() );
      size_t count = 0;
      for ( IterT ip = ip2.begin(); ip != ip2.end(); ip++ ) {
        std::copy( ip->begin(), ip->end(), ann_pts[count] );
        count++;
      }
      ANNkd_tree* kdtree = new ANNkd_tree( ann_pts, ip2.size(),
                                           ip2.begin()->size() );

      // Making Searches in ANN
      progress_callback.report_progress(0);
      std::vector<size_t> match_index( ip1.size() );
      Vector2 nn_distances;
      Vector2i nn_indexes;
      Vector<float> query( ip1.begin()->size() );
      float inc_amt = 1/float(ip1.size());
      count = 0;
      const size_t FAIL = ip2.size();
      BOOST_FOREACH( ip::InterestPoint const& ip, ip1 ) {
        if (progress_callback.abort_requested())
          vw_throw( Aborted() << "Aborted by ProgressCallback" );
        progress_callback.report_incremental_progress(inc_amt);

        std::copy( ip.begin(), ip.end(), query.begin() );
        kdtree->annkSearch( &query[0], 2,
                            &nn_indexes[0], &nn_distances[0], eps );

        if ( nn_distances[0] > m_threshold * nn_distances[1] )
          match_index[count] = FAIL;
        else
          match_index[count] = nn_indexes[0];
        count++;
      }
      progress_callback.report_finished();

      // Deallocating ANN
      delete kdtree;
      annDeallocPts( ann_pts );
      annClose();

      // Building matched_ip1 & matched ip 2
      for (size_t i = 0; i < ip1.size(); i++ ) {
        if ( match_index[i] < FAIL ) {
          matched_ip1.push_back( ip1[i] );
          matched_ip2.push_back( ip2[match_index[i]] );
        }
      }
    }
  };

}

#endif
