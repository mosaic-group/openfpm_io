#include "config.h"
#include "insitu_catalyst.hpp"

#ifdef HAVE_CATALYST

#include <catalyst.hpp>
#include <Vector/map_vector.hpp>

template<typename T>
struct attr_type_set_impl
{
    void set()
    {
        // unsupported we skip
    }
};

template<>
struct attr_type_set_impl<double>
{
    template<typename T,typename fields_type>
    void set(size_t offset, size_t stride, size_t n_pnt, void * buffer, std::string & base, fields_type & fields)
    {
        fields[base + "/values"].set_external((conduit_float64 *)buffer, n_pnt, offset,stride);
    }
};

template<>
struct attr_type_set_impl<float>
{
    template<typename T, typename fields_type>
    void set(size_t offset, size_t stride, size_t n_pnt, void * buffer, std::string & base, fields_type & fields)
    {
        fields[base + "/values"].set_external((conduit_float32 *)buffer, n_pnt, offset,stride);
    }
};

template<unsigned int N>
struct attr_type_set_impl<double[N]>
{
    template<typename T, typename fields_type>
    void set(size_t offset, size_t stride, size_t n_pnt, void * buffer, std::string & base, fields_type & fields)
    {
        if (N == 3)
        {
            // velocity is stored in non-interlaced form (unlike points).
            fields[base + "/values/x"].set_external((conduit_float64 *)buffer, n_pnt, offset,stride);
            fields[base + "/values/y"].set_external((conduit_float64 *)buffer, n_pnt, offset+sizeof(double),stride);
            fields[base + "/values/z"].set_external((conduit_float64 *)buffer, n_pnt, offset+2*sizeof(double),stride);
        }
    }
};

template<unsigned int N>
struct attr_type_set_impl<float[N]>
{
    template<typename T, typename fields_type>
    void set(size_t offset, size_t stride, size_t n_pnt, void * buffer, std::string & base, fields_type & fields)
    {
        if (N == 3)
        {
            // velocity is stored in non-interlaced form (unlike points).
            fields[base + "/values/x"].set_external((conduit_float32 *)buffer, n_pnt, offset,stride);
            fields[base + "/values/y"].set_external((conduit_float32 *)buffer, n_pnt, offset+sizeof(float),stride);
            fields[base + "/values/z"].set_external((conduit_float32 *)buffer, n_pnt, offset+2*sizeof(float),stride);
        }
    }
};

/*! \brief this class is a functor for "for_each" algorithm
 *
 * This class is a functor for "for_each" algorithm. For each
 * element of the boost::vector the operator() is called.
 * Is mainly used to process broadcast request for each buffer
 *
 */
template<typename field_type, typename vector_prop_type>
struct insitu_attr_out
{
    openfpm::vector<std::string> & attr;

    field_type & fields;

    size_t offset = 0;

    size_t stride = 0;

    size_t n_pnt = 0;

    vector_prop_type props;

	/*! \brief constructor
	 *
	 * \param v set of pointer buffers to set
	 *
	 */
	inline insitu_attr_out(openfpm::vector<std::string> & attr, field_type & fields)
	:attr(attr),fields(fields)
	{};

	//! It call the copy function for each property
	template<typename T>
	inline void operator()(T& t)
	{
        fields[attr.get(T::value) + "/association"].set("vertex");
        fields[attr.get(T::value) + "/topology"].set("mesh");
        fields[attr.get(T::value) + "/volume_dependent"].set("false");

        typedef typename boost::mpl::at<typename vector_prop_type::value_type::type,T>::type wtype;

        void * buffer = props.template getPointer<T::value>();

        attr_type_set_impl<wtype>::set(offset,stride,n_pnt,props.template getPointer<T::value>(),attr.get(T::value),fields);
	}
};

/*! \brief
 *
 * Implement insitu visualization with catalyst
 * 
 */
template<unsigned int dim , typename St>
class insitu_viscatalyst<dim,St,insity_catalyst_implementation::GRID>
{

    openfpm::vector<Point<dim,St>> points;
    openfpm::vector<unsigned int> cells;
    openfpm::vector<double> data;

public:

    insitu_viscatalyst()
    {}


    void initialize(const std::string & script)
    {
        conduit_cpp::Node node;

        node["catalyst/scripts/script0"].set_string(script);

        catalyst_initialize(conduit_cpp::c_node(&node));
    }

    void finalize()
    {
        conduit_cpp::Node node;
        catalyst_finalize(conduit_cpp::c_node(&node));
    }

    template<typename T>
    void execute(grid_cpu<dim,T> & grid, 
            Point<dim,St> & offset, 
            Box<dim,St> & cellbox, 
            Box<dim,int> box,
            openfpm::vector<std::string> & props,
            long int cycle = 0, 
            double time = 0)
    {
        conduit_cpp::Node exec_params;

        // add time/cycle information
        auto state = exec_params["catalyst/state"];
        state["timestep"].set(cycle);
        state["time"].set(time);

        //////////////////////// DO FOR ALL PROPERTIES

        // Add channels.
        // We only have 1 channel here. Let's name it 'grid'.
        auto channel = exec_params["catalyst/channels/grid"];

        ///////////////////////////////////////////////////////////

        // Since this example is using Conduit Mesh Blueprint to define the mesh,
        // we set the channel's type to "mesh".
        channel["type"].set("mesh");

        // now create the mesh.
        auto mesh = channel["data"];

        // start with coordsets (of course, the sequence is not important, just make
        // it easier to think in this order).
        mesh["coordsets/coords/type"].set("explicit");

        size_t n_pnt = 0;
        auto box2 = box;
        box2.shrinkP2(1);

        points.resize(box.getVolumeKey());
        data.resize(box.getVolumeKey());
        cells.resize(box2.getVolumeKey()*openfpm::math::pow(2,dim));

        auto it2 = grid.getIterator(box2.getKP1(),box2.getKP2());

        size_t cell_c = 0;

        while (it2.isNext())
        {
            auto p = it2.get();

            if (dim == 3)
            {
                int i = p.get(0) - box.getLow(0);
                int j = p.get(1) - box.getLow(1);
                int k = p.get(2) - box.getLow(2);

                long int numPoints[3] = {box.getHigh(0) - box.getLow(0) + 1,box.getHigh(1) - box.getLow(1) + 1,box.getHigh(2) - box.getLow(2) + 1};

                cells.get(cell_c) = i * numPoints[1] * numPoints[2] + j * numPoints[2] + k;
                cells.get(cell_c + 1) = (i + 1) * numPoints[1] * numPoints[2] + j * numPoints[2] + k;
                cells.get(cell_c + 2) = (i + 1) * numPoints[1] * numPoints[2] + (j + 1) * numPoints[2] + k;
                cells.get(cell_c + 3) = i * numPoints[1] * numPoints[2] + (j + 1) * numPoints[2] + k;
                cells.get(cell_c + 4) = i * numPoints[1] * numPoints[2] + j * numPoints[2] + k + 1;
                cells.get(cell_c + 5) = (i + 1) * numPoints[1] * numPoints[2] + j * numPoints[2] + k + 1;
                cells.get(cell_c + 6) = (i + 1) * numPoints[1] * numPoints[2] + (j + 1) * numPoints[2] + k + 1;
                cells.get(cell_c + 7) = i * numPoints[1] * numPoints[2] + (j + 1) * numPoints[2] + k + 1;
            }

            cell_c += 8;

            ++it2;
        }

        auto it = grid.getIterator(box.getKP1(),box.getKP2());

        size_t pnt_c = 0;

        while (it.isNext())
        {
            auto p = it.get();

            for (int j = 0 ; j < dim ; j++)
            {
                points.template get<0>(pnt_c)[j] = p.get(j)*cellbox.getHigh(j) + offset.get(j);
                data.template get<0>(pnt_c) = grid.template get<0>(p);
            }

            pnt_c++;

            ++it;
        }

        mesh["coordsets/coords/values/x"].set_external((conduit_float64*)&points.template get<0>(0), points.size(), 0, 3 * sizeof(St));
        mesh["coordsets/coords/values/y"].set_external((conduit_float64*)&points.template get<0>(0), points.size(), sizeof(St), 3 * sizeof(St));
        mesh["coordsets/coords/values/z"].set_external((conduit_float64*)&points.template get<0>(0), points.size(), 2 * sizeof(St), 3 * sizeof(St));

        // Next, add topology
        mesh["topologies/mesh/type"].set("unstructured");
        mesh["topologies/mesh/coordset"].set("coords");
        mesh["topologies/mesh/elements/shape"].set("hex");
        mesh["topologies/mesh/elements/connectivity"].set_external(&cells.template get<0>(0), cells.size());

        // Finally, add fields.

        auto fields = mesh["fields"];

        // pressure is cell-data.
        fields["pressure/association"].set("vertex");
        fields["pressure/topology"].set("mesh");
        fields["pressure/volume_dependent"].set("false");
        fields["pressure/values"].set_external((conduit_float64 *)&data.template get<0>(0),points.size(),0,sizeof(T));

        //exec_params.print();

        catalyst_execute(conduit_cpp::c_node(&exec_params));
    }

};

#endif