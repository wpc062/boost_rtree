#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <stdio.h>
#include <string.h>

#include <unistd.h>


#include <GL/glut.h>

using namespace std;

#define BOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT // solve compile error when compile rtree of boost 1.59
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/ring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>

#include <boost/geometry/index/detail/rtree/utilities/gl_draw.hpp>
#include <boost/geometry/index/detail/rtree/utilities/are_boxes_ok.hpp>
#include <boost/geometry/index/detail/rtree/utilities/are_levels_ok.hpp>
#include <boost/geometry/index/detail/rtree/utilities/statistics.hpp>
#include <boost/geometry/index/detail/rtree/utilities/print.hpp>
//#include "print.hpp"

#include <boost/timer.hpp>
#include <boost/foreach.hpp>
//#include <boost/variant.hpp>
#include <boost/scoped_ptr.hpp>


#define ENABLE_POINTS_AND_SEGMENTS

namespace bg = boost::geometry;
namespace bgi = bg::index;


namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<box, size_t> value;

// used types

typedef point P;
typedef box B;
typedef bg::model::linestring<P> LS;
typedef bg::model::segment<P> S;
typedef bg::model::ring<P> R;
typedef bg::model::polygon<P> Poly;
typedef bg::model::multi_polygon<Poly> MPoly;



double g_bbox[4] = {-150, 150, -150, 150};


typedef bgi::rtree<value, bgi::linear<500> > rtree_t;
boost::scoped_ptr<rtree_t> g_rtree;
std::vector<value> g_result_s;

// (10913.869000 13326.157000, 10914.501000 13326.871000)
// POLYGON((384.54 1490.22,384.54 1491.78,390.66 1491.78,390.66 1490.22,384.54 1490.22)) - 328184
//box g_query_box(point(10913.869000, 13326.157000), point(10914.501000, 13326.871000));
box g_query_box(point(380, 1488), point(390, 1498));

//B search_box;
//P search_point;
template <typename RTree>
inline size_t depth(RTree const& t)
{
    return bgi::detail::rtree::utilities::view<RTree>(t).depth();
}

template <typename RTree>
inline void draw_tree(RTree const& r, int max_level = 1000)
{
    bgi::detail::rtree::utilities::gl_draw(r, 0, max_level);
}

template <typename Values, typename RTree>
inline void draw_result(Values const& v, RTree const& r)
{
    for ( size_t i = 0 ; i < v.size() ; ++i )
    {
        bgi::detail::utilities::gl_draw_indexable(v[i].first, depth(r));
    }
}

// various drawing functions

void draw_point(P const& p)
{
    float x = boost::geometry::get<0>(p);
    float y = boost::geometry::get<1>(p);
    float z = depth(*g_rtree);

    glBegin(GL_QUADS);
    glVertex3f(x+1, y, z);
    glVertex3f(x, y+1, z);
    glVertex3f(x-1, y, z);
    glVertex3f(x, y-1, z);
    glEnd();
}

void draw_segment(S const& s)
{
    float x1 = boost::geometry::get<0, 0>(s);
    float y1 = boost::geometry::get<0, 1>(s);
    float x2 = boost::geometry::get<1, 0>(s);
    float y2 = boost::geometry::get<1, 1>(s);
    float z = depth(*g_rtree);

    glBegin(GL_LINES);
    glVertex3f(x1, y1, z);
    glVertex3f(x2, y2, z);
    glEnd();
}

template <typename Box>
void draw_box(Box const& box)
{
    float x1 = boost::geometry::get<bg::min_corner, 0>(box);
    float y1 = boost::geometry::get<bg::min_corner, 1>(box);
    float x2 = boost::geometry::get<bg::max_corner, 0>(box);
    float y2 = boost::geometry::get<bg::max_corner, 1>(box);
    float z = depth(*g_rtree);

    // search box
    glBegin(GL_LINE_LOOP);
        glVertex3f(x1, y1, z);
        glVertex3f(x2, y1, z);
        glVertex3f(x2, y2, z);
        glVertex3f(x1, y2, z);
    glEnd();
}

template <typename Range>
void draw_ring(Range const& range)
{
    float z = depth(*g_rtree);

    // search box
    glBegin(GL_LINE_LOOP);
    
    BOOST_FOREACH(P const& p, range)
    {
        float x = boost::geometry::get<0>(p);
        float y = boost::geometry::get<1>(p);

        glVertex3f(x, y, z);
    }
    glEnd();
}

template <typename Polygon>
void draw_polygon(Polygon const& polygon)
{
    draw_ring(polygon.outer());
    BOOST_FOREACH(Poly::ring_type const& r, polygon.inners())
        draw_ring(r);
}

template <typename MultiPolygon>
void draw_multi_polygon(MultiPolygon const& multi_polygon)
{
    BOOST_FOREACH(Poly const& p, multi_polygon)
        draw_polygon(p);
}

// render the scene -> tree, if searching data available also the query geometry and result
bool search_valid = false;
int g_max_level_draw = 1;
bool g_depth_test = false;
double g_center[2] = {0};

void render_scene(void)
{
    boost::timer t;
    cout << "render scene\n";

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    draw_tree(*g_rtree, g_max_level_draw);

    if ( search_valid )
    {
        glColor3f(1.0f, 0.25f, 0.0f);

        //draw_knn_area(0, 0);
        draw_box(g_query_box);

        glColor3f(1.0f, 0.5f, 0.0f);

        draw_result(g_result_s, *g_rtree);
    }

    glFlush();
    std::cout << "render scene: " << t.elapsed() << " sec." << std::endl;
}

void resize(int w, int h)
{
    if ( h == 0 )
        h = 1;

    //float ratio = float(w) / h;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glViewport(0, 0, w, h);

    //gluPerspective(45, ratio, 1, 1000);
    box b = g_rtree->bounds();
    cout << "bbox: " << bg::get<bg::min_corner, 0>(b) << ", " << bg::get<bg::min_corner, 1>(b)
        << " -> " << bg::get<bg::max_corner, 0>(b) << ", " << bg::get<bg::max_corner, 1>(b) << endl;
    double box_w =  1.1 * (bg::get<bg::max_corner, 0>(b)) - (bg::get<bg::min_corner, 0>(b));
    double box_h =  1.1 * (bg::get<bg::max_corner, 1>(b)) - (bg::get<bg::min_corner, 1>(b));
    glOrtho(-0.5 * box_w, 0.5 * box_w,
        -0.5 * box_h, 0.5 * box_h,
        -150, 150);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    double xc = 0.5 * (bg::get<bg::min_corner, 0>(b) + bg::get<bg::max_corner, 0>(b)),
        yc = 0.5 * (bg::get<bg::min_corner, 1>(b) + bg::get<bg::max_corner, 1>(b));
    g_center[0] = xc;
    g_center[1] = yc;

    cout << "center: " << xc << ", " << yc << endl;
    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // glCullFace(GL_FRONT);
    if (g_depth_test) {
        gluLookAt(
            xc, yc, -100.0f, 
            xc, yc, 100.0f,
            0.0f, 1.0f, 0.0f);
        glEnable(GL_DEPTH_TEST);    
    } else {
        gluLookAt(
            xc, yc, 100.0f, 
            xc, yc, -100.0f,
            0.0f, 1.0f, 0.0f);
        glDisable(GL_DEPTH_TEST); 
    }
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(1.0f);

    //srand(1);
}

std::ofstream ofs;
void test_boost_rtree(const char* bbox_filename)
{
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
    g_rtree->reset(new rtree_t);
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
        g_rtree->insert(std::make_pair(boxes[i], i));
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
        boxes.push_back(std::make_pair(b, ++i));
    }
    std::cout << "read bbox file: " << t.elapsed() << " sec." << std::endl;

    t.restart();
    g_rtree.reset(new rtree_t(boxes));
    std::vector<value>().swap(boxes); // free memory occupied by boxes.
    std::cout << "build rtree with " << boxes.size() << " boxes in total: " << t.elapsed() << " sec." << std::endl;
#endif

    // find values intersecting some area defined by a box
    // (10913.869000 13326.157000, 10914.501000 13326.871000)
    //std::vector<value> g_result_s;
    t.restart();
    g_rtree->query(bgi::intersects(g_query_box), std::back_inserter(g_result_s));

    // display results
    std::cout << "spatial query box: " << t.elapsed() << " sec." << std::endl;
    ofs << "spatial query box: " << t.elapsed() << " sec." << std::endl;
    ofs << bg::wkt<box>(g_query_box) << std::endl;

    ofs << "spatial query result:" << g_result_s.size() << std::endl;
    BOOST_FOREACH(value const& v, g_result_s)
    {
        ofs << bg::wkt<box>(v.first) << " - " << v.second << std::endl;
    }

    // find 5 nearest values to a point
    std::vector<value> result_n;
    point k_point(10914, 13326.5);
    t.restart();
    g_rtree->query(bgi::nearest(k_point, 5), std::back_inserter(result_n));

    std::cout << "knn query point: " << t.elapsed() << " sec." << std::endl;
    ofs << "knn query point: " << t.elapsed() << " sec." << std::endl;
    ofs << bg::wkt<point>(k_point) << std::endl;
    ofs << "knn query result:" << std::endl;
    BOOST_FOREACH(value const& v, result_n)
    {
        ofs << bg::wkt<box>(v.first) << " - " << v.second << std::endl;
    }
}


// handle mouse input

void mouse(int button, int state, int /*x*/, int /*y*/)
{
    if ( button == GLUT_LEFT_BUTTON && state == GLUT_DOWN )
    {
        //insert_random_value(cont);
        search_valid = false;
        g_max_level_draw++;
    }
    else if ( button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN )
    {
        //remove_random_value(cont);
        search_valid = false;
        if (g_max_level_draw > 0)
            g_max_level_draw--;
    }
    else if ( button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN )
    {
        //search();
        g_depth_test = !g_depth_test;
        if (g_depth_test) {
            gluLookAt(
                g_center[0], g_center[1], -100.0f, 
                g_center[0], g_center[1], 100.0f,
                0.0f, 1.0f, 0.0f);
            glEnable(GL_DEPTH_TEST);    
        } else {
            gluLookAt(
                g_center[0], g_center[1], 100.0f, 
                g_center[0], g_center[1], -100.0f,
                0.0f, 1.0f, 0.0f);
            glDisable(GL_DEPTH_TEST); 
        }
    }

    glutPostRedisplay();
}

// handle keyboard input
void keyboard(unsigned char key, int /*x*/, int /*y*/)
{

}


// main function

int glut_main(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_SINGLE | GLUT_RGBA);
    glutInitWindowPosition(100,100);
    glutInitWindowSize(600, 600);
    glutCreateWindow("boost::geometry::index::rtree GLUT test");

    glutDisplayFunc(render_scene);
    glutReshapeFunc(resize);
    glutMouseFunc(mouse);
    glutKeyboardFunc(keyboard);

    glutMainLoop();

    return 0;
}


int main(int argc, char* argv[])
{
    if (argc > 1) {
        if (access(argv[1], F_OK)) {
            fprintf(stderr, "cannot open file %s\n", argv[1]);
            return 1;
        }

        ofs.open("output.log");
        test_boost_rtree(argv[1]);
        if (argc > 2 && strcmp(argv[2], "view") == 0) {
            argc = 1;
            glut_main(argc, argv);
        }

#if BOOST_VERSION >= 105900
        if (getenv("rtree_print")) {
            ofs << "tree node list:\n";
            /*size_t count = 0;
            for (BOOST_AUTO(it, g_rtree->begin()); it != g_rtree->end(); ++it) {
                const value& v = *it;
                if (count % 100 == 0) ofs << "\n" << (count + 1) << ": ";
                ofs << v.second << ' ';
                ++count;
                
            }*/

            //BOOST_AUTO(re, bgi::detail::rtree::utilities::statistics(rtree));
            //ofs << "\nnode number: " << boost::get<1>(re) << endl;

            bgi::detail::rtree::utilities::print(ofs, *g_rtree);
        }
#endif

    } else {
        fprintf(stderr, "Usage: %s <bbox filename>.\n", argv[0]);
        return 1;
    }
    
    return 0;
}

