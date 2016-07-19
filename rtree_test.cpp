#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <stdio.h>
#include <string.h>

#include <unistd.h>


using namespace std;

#define BOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT // solve compile error when compile rtree of boost 1.59
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/timer.hpp>

//#include "RTree.h"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<box, size_t> value;

// (10913.869000 13326.157000, 10914.501000 13326.871000)
// POLYGON((384.54 1490.22,384.54 1491.78,390.66 1491.78,390.66 1490.22,384.54 1490.22)) - 328184
//box query_box(point(10913.869000, 13326.157000), point(10914.501000, 13326.871000));
box query_box(point(380, 1488), point(390, 1498));

std::ofstream ofs;

void test_boost_rtree(const char* bbox_filename)
{
    // create the rtree using default constructor
    // bgi::rtree<value, bgi::linear<500> > rtree;
    // bgi::rtree<value, bgi::linear<500, 150> > rtree;
    // bgi::rtree<value, bgi::quadratic<500, 150> > rtree;
    // bgi::rtree<value, bgi::rstar<32> > rtree;
    // bgi::rtree<value, bgi::dynamic_linear > rtree(bgi::dynamic_linear(500, 150));


    // create some values
    ifstream ifs(bbox_filename);
    if (!ifs.is_open()) return;
    string line;
    std::getline(ifs, line);
    ifs.seekg(0, ifs.end);
    size_t line_num = static_cast<size_t>(ifs.tellg() / line.size() * 1.1);
    std::cout << "reserve " << line_num << " boxes" << std::endl;
    ifs.seekg(0, ifs.beg);

#ifdef NO_PACK_ALGO
    bgi::rtree<value, bgi::linear<500> > rtree;
    std::vector<box> boxes;
    boxes.reserve(line_num);
    
    boost::timer t;
    while(!ifs.eof()) {
        long x0,y0, x1,y1;
        ifs >> x0 >> y0 >> x1 >> y1;
        // create a box
        box b(point(x0 * 0.001, y0 * 0.001), point(x1 * 0.001, y1 * 0.001));
        boxes.push_back(b);
    }
    std::cout << "read bbox file: " << t.elapsed() << " sec." << std::endl;

    t.restart();
    for (size_t i = 0; i < boxes.size(); ++i)
    {
        // insert new value
        rtree.insert(std::make_pair(boxes[i], i));
    }
    std::vector<box>().swap(boxes);  // free memory occupied by boxes.
    std::cout << "build rtree with " << boxes.size() << " boxes in total: " << t.elapsed() << " sec." << std::endl;
#else
    // packing algorithm
    std::vector<value> boxes;
    boxes.reserve(line_num);
    
    boost::timer t;
    int i = 0;
    while(!ifs.eof()) {
        long x0,y0, x1,y1;
        ifs >> x0 >> y0 >> x1 >> y1;
        // create a box
        box b(point(x0 * 0.001, y0 * 0.001), point(x1 * 0.001, y1 * 0.001));
        boxes.push_back(std::make_pair(b, i++));
    }
    std::cout << "read bbox file: " << t.elapsed() << " sec." << std::endl;

    t.restart();
    bgi::rtree<value, bgi::linear<500> > rtree(boxes);
    std::vector<value>().swap(boxes); // free memory occupied by boxes.
    std::cout << "build rtree with " << boxes.size() << " boxes in total: " << t.elapsed() << " sec." << std::endl;
#endif

    // find values intersecting some area defined by a box
    // (10913.869000 13326.157000, 10914.501000 13326.871000)
    std::vector<value> result_s;
    t.restart();
    rtree.query(bgi::intersects(query_box), std::back_inserter(result_s));

    // display results
    std::cout << "spatial query box: " << t.elapsed() << " sec." << std::endl;
    ofs << "spatial query box: " << t.elapsed() << " sec." << std::endl;
    ofs << bg::wkt<box>(query_box) << std::endl;

    ofs << "spatial query result:" << result_s.size() << std::endl;
    BOOST_FOREACH(value const& v, result_s)
    {
        ofs << bg::wkt<box>(v.first) << " - " << v.second << std::endl;
    }

    // find 5 nearest values to a point
    std::vector<value> result_n;
    point k_point(10914, 13326.5);
    t.restart();
    rtree.query(bgi::nearest(k_point, 5), std::back_inserter(result_n));

    std::cout << "knn query point: " << t.elapsed() << " sec." << std::endl;
    ofs << "knn query point: " << t.elapsed() << " sec." << std::endl;
    ofs << bg::wkt<point>(k_point) << std::endl;
    ofs << "knn query result:" << std::endl;
    BOOST_FOREACH(value const& v, result_n)
    {
        ofs << bg::wkt<box>(v.first) << " - " << v.second << std::endl;
    }
}

// used with Superliminar R-tree
bool RTreeSearchCallback(size_t id, void* arg)
{
    std::vector<size_t>* res = static_cast<std::vector<size_t>* >(arg);
    res->push_back(id);
    return true;
}

// http://superliminal.com/sources/sources.htm#C_Code
/*void test_RTree(const char* bbox_filename)
{
    std::cout << "use superliminal.RTree\n";
    namespace bg = boost::geometry;
    namespace bgi = boost::geometry::index;

    typedef bg::model::point<double, 2, bg::cs::cartesian> point;
    typedef bg::model::box<point> box;
    typedef std::pair<box, size_t> value;

    // create the Superliminar R-tree
    RTree<size_t, double, 2, double, 500, 150> rtree;
    
    // create some values
    ifstream ifs(bbox_filename);
    if (!ifs.is_open()) return;

    std::vector<box> boxes;

    boost::timer t;
    while(!ifs.eof()) {
        long x0,y0, x1,y1;
        ifs >> x0 >> y0 >> x1 >> y1;
        // create a box
        box b(point(x0 * 0.001, y0 * 0.001), point(x1 * 0.001, y1 * 0.001));
        boxes.push_back(b);
    }
    std::cout << "read bbox file: " << t.elapsed() << " sec." << std::endl;

    t.restart();
    double pnt0[2], pnt1[2];
    for (size_t i = 0; i < boxes.size(); ++i)
    {
        // insert new value
        const box& b = boxes[i];
        pnt0[0] = bg::get<0>(b.min_corner());
        pnt0[1] = bg::get<1>(b.min_corner());
        pnt1[0] = bg::get<0>(b.max_corner());
        pnt1[1] = bg::get<1>(b.max_corner());

        rtree.Insert(pnt0, pnt1, i);
    }
    std::cout << "build rtree with " << boxes.size() << " boxes in total: " << t.elapsed() << " sec." << std::endl;

    // find values intersecting some area defined by a box
    pnt0[0] = bg::get<0>(query_box.min_corner());
    pnt0[1] = bg::get<1>(query_box.min_corner());
    pnt1[0] = bg::get<0>(query_box.max_corner());
    pnt1[1] = bg::get<1>(query_box.max_corner());
    std::vector<size_t> result_s;
    t.restart();
    rtree.Search(pnt0, pnt1, RTreeSearchCallback, &result_s);

    // display results
    std::cout << "spatial query box: " << t.elapsed() << " sec." << std::endl;
    ofs << "spatial query box: " << t.elapsed() << " sec." << std::endl;
    ofs << bg::wkt<box>(query_box) << std::endl;

    ofs << "spatial query result:" << result_s.size() << std::endl;
    BOOST_FOREACH(size_t const& v, result_s)
    {
        ofs << bg::wkt<box>(boxes[v]) << " - " << v << std::endl;
    }
}*/

int main(int argc, char* argv[])
{
    if (argc > 1) {
        if (access(argv[1], F_OK)) {
            fprintf(stderr, "cannot open file %s\n", argv[1]);
            return 1;
        }

        ofs.open("output.log");
        test_boost_rtree(argv[1]);
        if (argc > 2 && strcmp(argv[2], "compare") == 0) {
            //test_RTree(argv[1]);
        }

    } else {
        fprintf(stderr, "Usage: %s <bbox filename>.\n", argv[0]);
        return 1;
    }
    
    return 0;
}

