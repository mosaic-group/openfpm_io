#include "config.h"

#ifdef HAVE_CATALYST
#include "catalyst_adaptor.hpp"

#include <catalyst.hpp>
#include <Vector/map_vector.hpp>
#include <boost/mp11.hpp>

/**
 * @brief Catalyst Adaptor for structured particles mesh, stored in `grid_dist`-like data structures.
 * 
 * @tparam grid_type Type of particles data structure.
 * @tparam props Indices of properties to be visualized.
 */
template <typename grid_type, unsigned int... props>
class catalyst_adaptor<grid_type, vis_props<props...>, catalyst_adaptor_impl::GRID_DIST_IMPL>
{
    // Create typename for chosen properties.
    typedef object<typename object_creator<typename grid_type::value_type::type, props...>::type> prop_object;
    // SoA storage for chosen properties, split across local sub-grids.
    openfpm::vector<openfpm::vector_soa<prop_object>> prop_storage;
    // Type of particles coordinates - float/double.
    typedef typename grid_type::stype stype;

public:
    catalyst_adaptor()
    {
    }

    void initialize(const openfpm::vector<std::string> &scripts)
    {
        conduit_cpp::Node node;

        for (int i = 0; i < scripts.size(); i++)
        {
            node["catalyst/scripts/script" + std::to_string(i)].set_string(scripts.get(i));
        }

        catalyst_status err = catalyst_initialize(conduit_cpp::c_node(&node));
        if (err != catalyst_status_ok)
        {
            std::cerr << "Failed to initialize Catalyst: " << err << std::endl;
        }
    }

    void finalize()
    {
        conduit_cpp::Node node;
        catalyst_status err = catalyst_finalize(conduit_cpp::c_node(&node));
        if (err != catalyst_status_ok)
        {
            std::cerr << "Failed to finalize Catalyst: " << err << std::endl;
        }
    }

    void execute(grid_type &grid, size_t cycle = 0, double time = 0, const std::string &channel_name = "grid")
    {
        // Simulation state
        conduit_cpp::Node exec_params;
        auto state = exec_params["catalyst/state"];
        state["timestep"].set(cycle);
        state["time"].set(time);
        
        // Mesh
        auto channel = exec_params["catalyst/channels/" + channel_name];
        channel["type"].set("mesh");
        auto mesh = channel["data"];
        mesh["coordsets/coords/type"].set("uniform");

        // Particles coordinates and properties
        // Iterate over local sub-grids stored on current processor and fill data about coordinates and properties.
        prop_storage.resize(grid.getN_loc_grid());

        for (size_t grid_num = 0; grid_num < grid.getN_loc_grid(); grid_num++)
        {
            auto box = grid.getLocalGridsInfo().get(grid_num).Dbox;
            auto offset = grid.getLocalGridsInfo().get(grid_num).origin;

            // FIXME: What is happening here?
            for (int dim = 0; dim < grid_type::dims; dim++)
            {
                if (offset.get(dim) + box.getHigh(dim) + 1 != grid.size(dim))
                {
                    box.setHigh(dim, box.getHigh(dim) + 1);
                }
            }

            // Extract local sub-grid, iterate over its particles and copy (subset of) their properties into SoA storage.
            auto &grid_loc = grid.get_loc_grid(grid_num);
            prop_storage.get(grid_num).resize(box.getVolumeKey());
            auto &prop_storage_loc = prop_storage.get(grid_num);

            auto it = grid_loc.getIterator(box.getKP1(), box.getKP2());
            size_t particle_idx = 0;

            while (it.isNext())
            {
                auto particle = it.get();
                object_s_di<decltype(grid_loc.template get_o(particle)), decltype(prop_storage_loc.template get(particle_idx)), OBJ_ENCAP, props...>(grid_loc.template get_o(particle), prop_storage_loc.template get(particle_idx));
                ++it;
                ++particle_idx;
            }

            // Set size of sub-grid along XYZ axes.
            mesh["coordsets/coords/dims/i"].set(box.getHigh(0) - box.getLow(0) + 1);
            mesh["coordsets/coords/dims/j"].set(box.getHigh(1) - box.getLow(1) + 1);
            mesh["coordsets/coords/dims/k"].set(box.getHigh(2) - box.getLow(2) + 1);

            // FIXME: What is happening here?
            Point<grid_type::dims, stype> poffset;
            for (int i = 0; i < grid_type::dims; i++)
            {
                poffset.get(i) = (offset.get(i) + box.getLow(i)) * grid.getSpacing()[i];
            }

            // Set origin of local sub-grid.
            mesh["coordsets/coords/origin/x"].set(poffset.get(0));
            mesh["coordsets/coords/origin/y"].set(poffset.get(1));
            mesh["coordsets/coords/origin/z"].set(poffset.get(2));

            // Set spacing of local sub-grid.
            mesh["coordsets/coords/spacing/dx"].set(grid.getSpacing()[0]);
            mesh["coordsets/coords/spacing/dy"].set(grid.getSpacing()[1]);
            mesh["coordsets/coords/spacing/dz"].set(grid.getSpacing()[2]);

            // Topology
            mesh["topologies/mesh/type"].set("uniform");
            mesh["topologies/mesh/coordset"].set("coords");

            // Fields
            auto fields = mesh["fields"];
            auto &prop_names = grid.getPropNames();
            set_prop_val_functor<grid_type, openfpm::vector_soa<prop_object>, props...> prop_setter(prop_names, fields, prop_storage_loc);
            boost::mpl::for_each_ref<boost::mpl::range_c<int, 0, sizeof...(props)>>(prop_setter);
        }

        // Execute
        catalyst_status err = catalyst_execute(conduit_cpp::c_node(&exec_params));
        if (err != catalyst_status_ok)
        {
            std::cerr << "Failed to execute Catalyst: " << err << std::endl;
        }
    }
};

#endif /* HAVE_CATALYST */