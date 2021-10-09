/*
 * file: KDTree.hpp
 * author: J. Frederico Carvalho
 *
 * This is an adaptation of the KD-tree implementation in rosetta code
 * https://rosettacode.org/wiki/K-d_tree
 *
 * It is a reimplementation of the C code using C++.  It also includes a few
 * more queries than the original, namely finding all points at a distance
 * smaller than some given distance to a point.
 *
 */

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <vector>
#include <iostream> // for print_tree()

#include "KDTree.hpp"

KDNode::KDNode() = default;

KDNode::KDNode(const point_t &pt, const size_t &idx_, const KDNodePtr &left_,
               const KDNodePtr &right_) {
    x = pt;
    index = idx_;
    left = left_;
    right = right_;
}

KDNode::KDNode(const pointIndex &pi, const KDNodePtr &left_,
               const KDNodePtr &right_) {
    x = pi.first;
    index = pi.second;
    left = left_;
    right = right_;
}

// for vertex normal
KDNode::KDNode(const pointNormalIndex &pi, const KDNodePtr &left_,
               const KDNodePtr &right_, int max) {
    // pi = { pair(coordinate, idx), pair(sn1, idx), pair(sn2, idx) }
    x = std::get<0>(pi).first;
    sn1 = std::get<1>(pi);
    if(std::get<2>(pi).second != max)
        sn2 = std::get<2>(pi);
    else 
        sn2 = std::get<1>(pi);
    index = std::get<0>(pi).second;
    left = left_;
    right = right_;
}

KDNode::~KDNode() = default;

double KDNode::coord(const size_t &idx) { return x.at(idx); }
KDNode::operator bool() { return (!x.empty()); }
KDNode::operator point_t() { return x; }
KDNode::operator size_t() { return index; }
KDNode::operator pointIndex() { return pointIndex(x, index); }

KDNodePtr NewKDNodePtr() {
    KDNodePtr mynode = std::make_shared< KDNode >();
    return mynode;
}

inline double dist2(const point_t &a, const point_t &b) {
    double distc = 0;
    for (size_t i = 0; i < a.size(); i++) {
        double di = a.at(i) - b.at(i);
        distc += di * di;
    }
    return distc;
}

inline double dist2(const KDNodePtr &a, const KDNodePtr &b) {
    return dist2(a->x, b->x);
}

inline double dist(const point_t &a, const point_t &b) {
    return std::sqrt(dist2(a, b));
}

inline double dist(const KDNodePtr &a, const KDNodePtr &b) {
    return std::sqrt(dist2(a, b));
}

comparer::comparer(size_t idx_) : idx{idx_} {};

inline bool comparer::compare_idx(const pointIndex &a,  //
                                  const pointIndex &b   //
) {
    return (a.first.at(idx) < b.first.at(idx));  //
}

inline bool comparer::compare_idx_n(const pointNormalIndex &a,  //
                                  const pointNormalIndex &b   //
) {
    return (std::get<0>(a).first.at(idx) < std::get<0>(b).first.at(idx));  
}

inline void sort_on_idx(const pointIndexArr::iterator &begin,  //
                        const pointIndexArr::iterator &end,    //
                        size_t idx) {
    comparer comp(idx);
    comp.idx = idx;

    using std::placeholders::_1;
    using std::placeholders::_2;

    std::nth_element(begin, begin + std::distance(begin, end) / 2,
                     end, std::bind(&comparer::compare_idx, comp, _1, _2));
}

inline void sort_on_idx_n(const pointNormalIndexArr::iterator &begin,  //
                        const pointNormalIndexArr::iterator &end,    //
                        size_t idx) {
    comparer comp(idx);
    comp.idx = idx;

    using std::placeholders::_1;
    using std::placeholders::_2;

    std::nth_element(begin, begin + std::distance(begin, end) / 2,
                     end, std::bind(&comparer::compare_idx_n, comp, _1, _2));
}

using pointVec = std::vector< point_t >;

KDNodePtr KDTree::make_tree(const pointIndexArr::iterator &begin,  //
                            const pointIndexArr::iterator &end,    //
                            const size_t &length,                  //
                            const size_t &level                    //
) {
    if (begin == end) {
        return NewKDNodePtr();  // empty tree
    }

    size_t dim = begin->first.size();

    if (length > 1) {
        sort_on_idx(begin, end, level);
    }

    auto middle = begin + (length / 2);

    auto l_begin = begin;
    auto l_end = middle;
    auto r_begin = middle + 1;
    auto r_end = end;

    size_t l_len = length / 2;
    size_t r_len = length - l_len - 1;

    KDNodePtr left;
    if (l_len > 0 && dim > 0) {
        left = make_tree(l_begin, l_end, l_len, (level + 1) % dim);
    } else {
        left = leaf;
    }
    KDNodePtr right;
    if (r_len > 0 && dim > 0) {
        right = make_tree(r_begin, r_end, r_len, (level + 1) % dim);
    } else {
        right = leaf;
    }

    // KDNode result = KDNode();
    return std::make_shared< KDNode >(*middle, left, right);
}

KDNodePtr KDTree::make_tree_n(const pointNormalIndexArr::iterator &begin,  //
                            const pointNormalIndexArr::iterator &end,    //
                            const size_t &length,                  //
                            const size_t &level,                    //
                            const int max
) {
    if (begin == end) {
        return NewKDNodePtr();  // empty tree
    }

    // size_t dim = begin->first.first.size();
    size_t dim = std::get<0>(*begin).first.size();

    if (length > 1) {
        sort_on_idx_n(begin, end, level);
    }

    auto middle = begin + (length / 2);

    auto l_begin = begin;
    auto l_end = middle;
    auto r_begin = middle + 1;
    auto r_end = end;

    size_t l_len = length / 2;
    size_t r_len = length - l_len - 1;

    KDNodePtr left;
    if (l_len > 0 && dim > 0) {
        left = make_tree_n(l_begin, l_end, l_len, (level + 1) % dim, max);
    } else {
        left = leaf;
    }
    KDNodePtr right;
    if (r_len > 0 && dim > 0) {
        right = make_tree_n(r_begin, r_end, r_len, (level + 1) % dim, max);
    } else {
        right = leaf;
    }

    // KDNode result = KDNode();
    return std::make_shared< KDNode >(*middle, left, right, max);
}

KDTree::KDTree(pointVec point_array) {
    leaf = std::make_shared< KDNode >();
    // iterators
    pointIndexArr arr;
    for (size_t i = 0; i < point_array.size(); i++) {
        arr.push_back(pointIndex(point_array.at(i), i));
    }

    auto begin = arr.begin();
    auto end = arr.end();

    size_t length = arr.size();
    size_t level = 0;  // starting

    root = KDTree::make_tree(begin, end, length, level);
}

// KDTree::KDTree(pointVec point_array, pointVec normal_array) {
//     leaf = std::make_shared< KDNode >();
//     // iterators
//     pointIndex pi;
//     pointNormalIndexArr arr;
//     for (size_t i = 0; i < point_array.size(); i++) {
//         pi = pointIndex(point_array.at(i), i);
//         arr.push_back(pointNormalIndex(pi, normal_array.at(i)));
//     }

//     auto begin = arr.begin();
//     auto end = arr.end();

//     size_t length = arr.size();
//     size_t level = 0;  // starting

//     root = KDTree::make_tree_n(begin, end, length, level);
// }

KDTree::KDTree(pointVec point_array, pointIndexArr surface_n, int max){
    leaf = std::make_shared< KDNode >();

    // iterators
    pointIndex pi;
    pointNormalIndexArr arr;
    for (size_t i = 0; i < point_array.size(); i++) {
        pi = pointIndex(point_array.at(i), i);
        arr.push_back(pointNormalIndex(pi, surface_n.at(i*2), surface_n.at(i*2+1)));
    }

    auto begin = arr.begin();
    auto end = arr.end();

    size_t length = arr.size();
    size_t level = 0;  // starting

    root = KDTree::make_tree_n(begin, end, length, level, max);
}


KDNodePtr KDTree::nearest_(   //
    const KDNodePtr &branch,  //
    const point_t &pt,        //
    const size_t &level,      //
    const KDNodePtr &best,    //
    const double &best_dist   //
) {
    double d, dx, dx2;

    if (!bool(*branch)) {
        return NewKDNodePtr();  // basically, null
    }

    point_t branch_pt(*branch);
    size_t dim = branch_pt.size();

    d = dist2(branch_pt, pt);
    dx = branch_pt.at(level) - pt.at(level);
    dx2 = dx * dx;

    KDNodePtr best_l = best;
    double best_dist_l = best_dist;

    if (d < best_dist) {
        best_dist_l = d;
        best_l = branch;
    }

    size_t next_lv = (level + 1) % dim;
    KDNodePtr section;
    KDNodePtr other;

    // select which branch makes sense to check
    if (dx > 0) {
        section = branch->left;
        other = branch->right;
    } else {
        section = branch->right;
        other = branch->left;
    }

    // keep nearest neighbor from further down the tree
    KDNodePtr further = nearest_(section, pt, next_lv, best_l, best_dist_l);
    if (!further->x.empty()) {
        double dl = dist2(further->x, pt);
        if (dl < best_dist_l) {
            best_dist_l = dl;
            best_l = further;
        }
    }
    // only check the other branch if it makes sense to do so
    if (dx2 < best_dist_l) {
        further = nearest_(other, pt, next_lv, best_l, best_dist_l);
        if (!further->x.empty()) {
            double dl = dist2(further->x, pt);
            if (dl < best_dist_l) {
                best_dist_l = dl;
                best_l = further;
            }
        }
    }

    return best_l;
};

// default caller
KDNodePtr KDTree::nearest_(const point_t &pt) {
    size_t level = 0;
    // KDNodePtr best = branch;
    double branch_dist = dist2(point_t(*root), pt);
    return nearest_(root,          // beginning of tree
                    pt,            // point we are querying
                    level,         // start from level 0
                    root,          // best is the root
                    branch_dist);  // best_dist = branch_dist
};

point_t KDTree::nearest_point(const point_t &pt) {
    return point_t(*nearest_(pt));
};
size_t KDTree::nearest_index(const point_t &pt) {
    return size_t(*nearest_(pt));
};

pointIndex KDTree::nearest_pointIndex(const point_t &pt) {
    KDNodePtr Nearest = nearest_(pt);
    return pointIndex(point_t(*Nearest), size_t(*Nearest));
}

pointIndexArr KDTree::neighborhood_(  //
    const KDNodePtr &branch,          //
    const point_t &pt,                //
    const double &rad,                //
    const size_t &level               //
) {
    double d, dx, dx2;

    if (!bool(*branch)) {
        // branch has no point, means it is a leaf,
        // no points to add
        return pointIndexArr();
    }

    size_t dim = pt.size();

    double r2 = rad * rad;

    d = dist2(point_t(*branch), pt);
    dx = point_t(*branch).at(level) - pt.at(level);
    dx2 = dx * dx;

    pointIndexArr nbh, nbh_s, nbh_o;
    if (d <= r2) {
        nbh.push_back(pointIndex(*branch));
    }

    //
    KDNodePtr section;
    KDNodePtr other;
    if (dx > 0) {
        section = branch->left;
        other = branch->right;
    } else {
        section = branch->right;
        other = branch->left;
    }

    nbh_s = neighborhood_(section, pt, rad, (level + 1) % dim);
    nbh.insert(nbh.end(), nbh_s.begin(), nbh_s.end());
    if (dx2 < r2) {
        nbh_o = neighborhood_(other, pt, rad, (level + 1) % dim);
        nbh.insert(nbh.end(), nbh_o.begin(), nbh_o.end());
    }

    return nbh;
};

pointIndexArr KDTree::neighborhood(  //
    const point_t &pt,               //
    const double &rad) {
    size_t level = 0;
    return neighborhood_(root, pt, rad, level);
}

pointVec KDTree::neighborhood_points(  //
    const point_t &pt,                 //
    const double &rad) {
    size_t level = 0;
    pointIndexArr nbh = neighborhood_(root, pt, rad, level);
    pointVec nbhp;
    nbhp.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), nbhp.begin(),
                   [](pointIndex x) { return x.first; });
    return nbhp;
}

indexArr KDTree::neighborhood_indices(  //
    const point_t &pt,                  //
    const double &rad) {
    size_t level = 0;
    pointIndexArr nbh = neighborhood_(root, pt, rad, level);
    indexArr nbhi;
    nbhi.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), nbhi.begin(),
                   [](pointIndex x) { return x.second; });
    return nbhi;
}

void KDTree::print_node_(KDNodePtr &node){
    point_t vertex = node->x;
    std::cout << "vertex(" << node->index << "):";
    for(int i = 0; i < vertex.size(); i++) std::cout << " " << vertex[i];

    pointIndex sn1 = node->sn1;
    std::cout << "\nsn1: ";
    for(int i = 0; i < sn1.first.size(); i++) std::cout << " " << sn1.first[i];
    
    pointIndex sn2 = node->sn2;
    std::cout << "\nsn2: ";
    for(int i = 0; i < sn1.first.size(); i++) std::cout << " " << sn2.first[i];
    std::cout << std::endl;

    
    if(!node->left->x.empty()) print_node_(node->left);
    if(!node->right->x.empty()) print_node_(node->right);
}

void KDTree::print_tree(){
    std::cout << "####Printing KD-Tree: " << std::endl;
    print_node_(root);
} 

KDNodePtr KDTree::sn_nearest_(   //
    const KDNodePtr &branch,  //
    const point_t &pt,        //
    const size_t &level,      //
    const KDNodePtr &best,    //
    const double &best_dist   //
) {
    double d, dx, dx2;
    size_t sn_idx;

    if (!bool(*branch)) {
        return KDNodePtr();  // basically, null
    }

    point_t branch_pt(*branch);
    size_t dim = 2;


    point_t sn = branch->sn1.first;
    double sn_check = branch_pt[0]*sn[0]+branch_pt[1]*sn[1]+branch_pt[2]*sn[2]+sn[3];
    if(-0.0001 < sn_check && sn_check < 0.0001) return branch;

    sn = branch->sn2.first;
    sn_check = branch_pt[0]*sn[0]+branch_pt[1]*sn[1]+branch_pt[2]*sn[2]+sn[3];
    if(-0.0001 < sn_check && sn_check < 0.0001) return branch;

    // if not on the surface, search next point (assumes that finding nearest point would find the face index)
    d = dist2(branch_pt, pt);
    dx = branch_pt.at(level+1) - pt.at(level+1);
    dx2 = dx * dx;

    KDNodePtr best_l = best;
    double best_dist_l = best_dist;

    if (d < best_dist) {
        best_dist_l = d;
        best_l = branch;
    }

    size_t next_lv = (level + 1) % dim;
    KDNodePtr section;
    KDNodePtr other;

    // select which branch makes sense to check
    if (dx > 0) {
        section = branch->left;
        other = branch->right;
    } else {
        section = branch->right;
        other = branch->left;
    }

    // keep nearest neighbor from further down the tree
    KDNodePtr further = sn_nearest_(section, pt, next_lv, best_l, best_dist_l);
    if (!further->x.empty()) {
        double dl = dist2(further->x, pt);
        if (dl < best_dist_l) {
            best_dist_l = dl;
            best_l = further;
        }
        return further;
    }
    // only check the other branch if it makes sense to do so
    if (dx2 < best_dist_l) {
        further = sn_nearest_(other, pt, next_lv, best_l, best_dist_l);
        if (!further->x.empty()) {
            double dl = dist2(further->x, pt);
            if (dl < best_dist_l) {
                best_dist_l = dl;
                best_l = further;
            }

            return further;
        }
    }

    return KDNodePtr();
};

int KDTree::find_sn_index(const point_t &pt){
    size_t level = 0;
    // KDNodePtr best = branch;
    double branch_dist = dist2(point_t(*root), pt);
    KDNodePtr node =  sn_nearest_(root,          // beginning of tree
                                    pt,            // point we are querying
                                    level,         // start from level 0
                                    root,          // best is the root
                                    branch_dist);  // best_dist = branch_dist

    if(!node->x.empty()){
        point_t branch_pt(*node);
        point_t sn = node->sn1.first;
        double sn_check = branch_pt[0]*sn[0]+branch_pt[1]*sn[1]+branch_pt[2]*sn[2]+sn[3];
        if(-0.0001 < sn_check && sn_check < 0.0001) return node->sn1.second;

        sn = node->sn2.first;
        sn_check = branch_pt[0]*sn[0]+branch_pt[1]*sn[1]+branch_pt[2]*sn[2]+sn[3];
        if(-0.0001 < sn_check && sn_check < 0.0001) return node->sn2.second;
    }
    return -1;
}