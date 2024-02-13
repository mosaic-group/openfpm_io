#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include "Grid/grid_dist_id.hpp"
#include "Vector/vector_dist_subset.hpp"
#include "Grid/map_grid.hpp"

#include "config.h"

#ifdef HAVE_CATALYST
#include <catalyst.hpp>
#include "Catalyst/catalyst_adaptor.hpp"

BOOST_AUTO_TEST_SUITE(catalyst_test_suite)

#ifdef HAVE_PNG
#include <boost/gil/rgb.hpp>
#include <boost/gil/extension/io/png.hpp>

// Test utilities to compare Catalyst-produced image with ground truth.
struct PixelInserter
{
    openfpm::vector<char> &storage;

    PixelInserter(openfpm::vector<char> &s) : storage(s) {}

    void operator()(boost::gil::rgb8_pixel_t p) const
    {
        storage.add(boost::gil::at_c<0>(p));
        storage.add(boost::gil::at_c<1>(p));
        storage.add(boost::gil::at_c<2>(p));
    }
};

void check_png(const std::string &img1_path, const std::string &img2_path)
{
    auto &v_cl = create_vcluster();

    if (v_cl.rank() == 0)
    {
        // Check the file has been generated (we do not check the content)
        boost::gil::rgb8_image_t img1, img2;
        // FIXME: Ensure both images are accessible
        boost::gil::read_image(img1_path.c_str(), img1, boost::gil::png_tag());
        boost::gil::read_image(img2_path.c_str(), img2, boost::gil::png_tag());

        openfpm::vector<char> storage1, storage2;

        for_each_pixel(const_view(img1), PixelInserter(storage1));
        for_each_pixel(const_view(img2), PixelInserter(storage2));

        BOOST_REQUIRE(storage1.size() != 0);
        BOOST_REQUIRE(storage2.size() != 0);

        BOOST_REQUIRE_EQUAL(storage1.size(), storage2.size());

        size_t diff = 0;
        for (int i = 0; i < storage1.size(); i++)
        {
            diff += abs(storage1.get(i) - storage2.get(i));
        }

        BOOST_REQUIRE(diff < 3000000);
    }
}
#endif

BOOST_AUTO_TEST_CASE(catalyst_grid_dist_test)
{
    Vcluster<> &v_cl = create_vcluster();
    size_t sz[3] = {40, 40, 40};
    size_t bc[3] = {NON_PERIODIC, NON_PERIODIC, NON_PERIODIC};
    Box<3, double> domain({0, 0, 0}, {2.0, 2.0, 2.0});
    Ghost<3, double> g(3. * 2. / (sz[0] - 1));
    grid_dist_id<3, double, aggregate<double, VectorS<3, double>, double[3][3]>> grid(sz, domain, g);

    openfpm::vector<std::string> prop_names({"scalar", "vector", "tensor"});
    enum
    {
        SCALAR,
        VECTOR,
        TENSOR
    };
    grid.setPropNames(prop_names);

    auto it = grid.getDomainIterator();
    while (it.isNext())
    {
        auto key = it.get();
        auto pos = it.getGKey(key);

        double x = pos.get(0) * grid.getSpacing()[0];
        double y = pos.get(1) * grid.getSpacing()[1];
        double z = pos.get(2) * grid.getSpacing()[2];

        grid.get<SCALAR>(key) = x + y + z;
        
        grid.get<VECTOR>(key)[0] = sin(x + y);
        grid.get<VECTOR>(key)[1] = cos(x + y);
        grid.get<VECTOR>(key)[2] = 0.0;

        grid.get<TENSOR>(key)[0][0] = x*x;
        grid.get<TENSOR>(key)[0][1] = x*y;
        grid.get<TENSOR>(key)[0][2] = x*z;

        grid.get<TENSOR>(key)[1][0] = y*x;
        grid.get<TENSOR>(key)[1][1] = y*y;
        grid.get<TENSOR>(key)[1][2] = y*z;

        grid.get<TENSOR>(key)[2][0] = z*x;
        grid.get<TENSOR>(key)[2][1] = z*y;
        grid.get<TENSOR>(key)[2][2] = z*z;


        ++it;
    }

    grid.map();
    grid.ghost_get<SCALAR, VECTOR, TENSOR>();
    // Generate representative dataset to create visualization pipeline in ParaView
    //grid.write("grid_dist_test_data");

    // Visualization with Catalyst Adaptor
    // Images are written to ./datasets/ folder by default
    catalyst_adaptor<decltype(grid), vis_props<SCALAR, VECTOR, TENSOR>, catalyst_adaptor_impl::GRID_DIST_IMPL> adaptor;
    openfpm::vector<std::string> scripts({{"./test_data/catalyst_grid_dist_pipeline.py"}});
    adaptor.initialize(scripts);
    adaptor.execute(grid);
    adaptor.finalize();

#ifdef HAVE_PNG
    check_png("./datasets/catalyst_grid_dist.png", "./test_data/catalyst_grid_dist_ground_truth.png");
#endif
}

BOOST_AUTO_TEST_CASE(catalyst_vector_dist_test)
{
    Vcluster<> &v_cl = create_vcluster();
    size_t sz[3] = {40, 40, 40};
    size_t bc[3] = {NON_PERIODIC, NON_PERIODIC, NON_PERIODIC};
    Box<3, double> domain({0, 0, 0}, {2.0, 2.0, 2.0});
    Ghost<3, double> g(3. * 2. / (sz[0] - 1));
    vector_dist_ws<3, double, aggregate<double, VectorS<3, double>, double[3][3]>> particles(0, domain, bc, g);
    openfpm::vector<std::string> prop_names({"scalar", "vector", "tensor"});
    enum
    {
        SCALAR,
        VECTOR,
        TENSOR
    };
    particles.setPropNames(prop_names);

    // Assign scalar = x + y + z, vector = {sin(x + y), cos(x + y), 0}, tensor = {{x^2, xy}, {-xy, y^2}} properties to the particles
    auto it = particles.getGridIterator(sz);
    while (it.isNext())
    {
        particles.add();
        auto key = it.get();
        double x = key.get(0) * it.getSpacing(0);
        particles.getLastPos()[0] = x;
        double y = key.get(1) * it.getSpacing(1);
        particles.getLastPos()[1] = y;
        double z = key.get(2) * it.getSpacing(2);
        particles.getLastPos()[2] = z;

        particles.getLastProp<SCALAR>() = x + y + z;

        particles.getLastProp<VECTOR>()[0] = sin(x + y);
        particles.getLastProp<VECTOR>()[1] = cos(x + y);
        particles.getLastProp<VECTOR>()[2] = 0.0;

        particles.getLastProp<TENSOR>()[0][0] = x*x;
        particles.getLastProp<TENSOR>()[0][1] = x*y;
        particles.getLastProp<TENSOR>()[1][0] = -x*y;
        particles.getLastProp<TENSOR>()[1][1] = y*y;

        ++it;
    }

    particles.map();
    particles.ghost_get<SCALAR, VECTOR, TENSOR>();
    // Generate representative dataset to create visualization pipeline in ParaView
    //particles.write("vector_dist_test_data");

    // Visualization with Catalyst Adaptor
    // Images are written to ./datasets/ folder by default
    catalyst_adaptor<decltype(particles), vis_props<SCALAR, VECTOR, TENSOR>, catalyst_adaptor_impl::VECTOR_DIST_IMPL> adaptor;
    openfpm::vector<std::string> scripts({{"./test_data/catalyst_vector_dist_pipeline.py"}});
    adaptor.initialize(scripts);
    adaptor.execute(particles);
    adaptor.finalize();

#ifdef HAVE_PNG
    check_png("./datasets/catalyst_vector_dist.png", "./test_data/catalyst_vector_dist_ground_truth.png");
#endif
}

BOOST_AUTO_TEST_SUITE_END()

#endif