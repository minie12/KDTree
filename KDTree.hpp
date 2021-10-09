#pragma once

/*
 * file: KDTree.hpp
 * author: J. Frederico Carvalho
 *
 * This is an adaptation of the KD-tree implementation in rosetta code
 *  https://rosettacode.org/wiki/K-d_tree
 * It is a reimplementation of the C code using C++.
 * It also includes a few more queries than the original
 *
 */

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

using point_t = std::vector< double >;
using indexArr = std::vector< size_t >;
using pointIndex = typename std::pair< std::vector< double >, size_t >;
// using pointNormalIndex = typename std::pair< pointIndex , std::vector< double > >;
using pointNormalIndex = typename std::tuple< pointIndex , pointIndex, pointIndex >;  // (vertex, sn1, sn2)


class KDNode {
   public:
    using KDNodePtr = std::shared_ptr< KDNode >;
    size_t index;
    point_t x;  // vertex
    pointIndex sn1;  // surface normal
    pointIndex sn2;  // surface normal
    KDNodePtr left;
    KDNodePtr right;

    // initializer
    KDNode();
    KDNode(const point_t &, const size_t &, const KDNodePtr &,
           const KDNodePtr &);
    KDNode(const pointIndex &, const KDNodePtr &, const KDNodePtr &);
    KDNode(const pointNormalIndex &, const KDNodePtr &, const KDNodePtr &, const int);
    ~KDNode();

    // getter
    double coord(const size_t &);

    // conversions
    explicit operator bool();
    explicit operator point_t();
    explicit operator size_t();
    explicit operator pointIndex();
};

using KDNodePtr = std::shared_ptr< KDNode >;

KDNodePtr NewKDNodePtr();

// square euclidean distance
inline double dist2(const point_t &, const point_t &);
inline double dist2(const KDNodePtr &, const KDNodePtr &);

// euclidean distance
inline double dist(const point_t &, const point_t &);
inline double dist(const KDNodePtr &, const KDNodePtr &);

// Need for sorting
class comparer {
   public:
    size_t idx;
    explicit comparer(size_t idx_);
    inline bool compare_idx(
        const std::pair< std::vector< double >, size_t > &,  //
        const std::pair< std::vector< double >, size_t > &   //
    );
    inline bool compare_idx_n(
        const pointNormalIndex &,  //
        const pointNormalIndex &   //
    );
};

using pointIndexArr = typename std::vector< pointIndex >;
using pointNormalIndexArr = typename std::vector< pointNormalIndex >;

inline void sort_on_idx(const pointIndexArr::iterator &,  //
                        const pointIndexArr::iterator &,  //
                        size_t idx);

inline void sort_on_idx_n(const pointNormalIndexArr::iterator &,  //
                        const pointNormalIndexArr::iterator &,  //
                        size_t idx);


using pointVec = std::vector< point_t >;

class KDTree {
    KDNodePtr root;
    KDNodePtr leaf;

    KDNodePtr make_tree(const pointIndexArr::iterator &begin,  //
                        const pointIndexArr::iterator &end,    //
                        const size_t &length,                  //
                        const size_t &level                    //
    );

    KDNodePtr make_tree_n(const pointNormalIndexArr::iterator &begin,  //
                        const pointNormalIndexArr::iterator &end,    //
                        const size_t &length,                  //
                        const size_t &level,                    //
                        const int max
    );

   public:
    KDTree() = default;
    explicit KDTree(pointVec point_array);
    explicit KDTree(pointVec point_array, pointIndexArr surface_n, int max);

   private:
    KDNodePtr nearest_(           //
        const KDNodePtr &branch,  //
        const point_t &pt,        //
        const size_t &level,      //
        const KDNodePtr &best,    //
        const double &best_dist   //
    );

    // default caller
    KDNodePtr nearest_(const point_t &pt);

   public:
    point_t nearest_point(const point_t &pt);
    size_t nearest_index(const point_t &pt);
    pointIndex nearest_pointIndex(const point_t &pt);

   private:
    pointIndexArr neighborhood_(  //
        const KDNodePtr &branch,  //
        const point_t &pt,        //
        const double &rad,        //
        const size_t &level       //
    );

   public:
    pointIndexArr neighborhood(  //
        const point_t &pt,       //
        const double &rad);

    pointVec neighborhood_points(  //
        const point_t &pt,         //
        const double &rad);

    indexArr neighborhood_indices(  //
        const point_t &pt,          //
        const double &rad);

   private:
    void print_node_(KDNodePtr &node);
    KDNodePtr sn_nearest_(const KDNodePtr &branch,  //
                        const point_t &pt,        //
                        const size_t &level,      //
                        const KDNodePtr &best,    //
                        const double &best_dist   //
                        );
   public:
    void print_tree();
    int find_sn_index(const point_t &pt);

};
