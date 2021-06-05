#ifndef INSITU_CATALYST
#define INSITU_CATALYST

#include "config.h"

#ifdef HAVE_CATALYST

#include <catalyst.hpp>
#include <Vector/map_vector.hpp>

template<typename T>
struct attr_type_set_impl
{
    template<typename fields_type>
    static void set(size_t offset, size_t n_pnt, void * buffer, const std::string & base, fields_type & fields)
    {
        // unsupported we skip
    }
};

template<>
struct attr_type_set_impl<double>
{
    template<typename fields_type>
    static void set(size_t offset, size_t n_pnt, double & buffer, const std::string & base, fields_type & fields)
    {
        fields[base + "/values"].set_external((conduit_float64 *)&buffer, n_pnt, offset,sizeof(double));
    }
};

template<>
struct attr_type_set_impl<float>
{
    template<typename fields_type>
    static void set(size_t offset, size_t n_pnt, float & buffer, const std::string & base, fields_type & fields)
    {
        fields[base + "/values"].set_external((conduit_float32 *)&buffer, n_pnt, offset,sizeof(float));
    }
};

template<unsigned int N>
struct attr_type_set_impl<double[N]>
{
    template<typename fields_type, typename encap_array>
    static void set(size_t offset, size_t n_pnt, encap_array buffer, const std::string & base, fields_type & fields)
    {
        if (N == 3)
        {
            //std::cout << "AAAAAAAAAAAAAAAAAAAAAA " << (conduit_float64 *)((char *)buffer+stride*sizeof(double)) << std::endl;

            // velocity is stored in non-interlaced form (unlike points).
            fields[base + "/values/x"].set_external((conduit_float64 *)&buffer[0], n_pnt,0,sizeof(double));
            fields[base + "/values/y"].set_external((conduit_float64 *)&buffer[1], n_pnt,0,sizeof(double));
            fields[base + "/values/z"].set_external((conduit_float64 *)&buffer[2], n_pnt,0,sizeof(double));
        }
    }
};

template<unsigned int N>
struct attr_type_set_impl<float[N]>
{
    template<typename fields_type, typename encap_array>
    static void set(size_t offset, size_t stride, size_t n_pnt, encap_array buffer, const std::string & base, fields_type & fields)
    {
        if (N == 3)
        {
            // velocity is stored in non-interlaced form (unlike points).
            fields[base + "/values/x"].set_external((conduit_float32 *)&buffer[0], n_pnt,0,sizeof(float));
            fields[base + "/values/y"].set_external((conduit_float32 *)&buffer[1], n_pnt,0,sizeof(float));
            fields[base + "/values/z"].set_external((conduit_float32 *)&buffer[2], n_pnt,0,sizeof(float));
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

enum insity_catalyst_implementation
{
    GRID = 1,
    VECTOR = 2,
    GRID_DIST = 3
};

/*! \brief
 *
 * Implement insitu visualization with catalyst
 * 
 */
template<typename grid_type, typename vis_props, unsigned int impl>
class insitu_viscatalyst
{

public:

    insitu_viscatalyst()
    {}

    void initialize()
    {
        std::cout << __FILE__ << ":" << __LINE__ << " The implementetion has not been selected or is unknown " << std::endl;
    }

    void finalize()
    {
        std::cout << __FILE__ << ":" << __LINE__ << " The implementetion has not been selected or is unknown " << std::endl;
    }

    template<typename TypeToVis>
    void execute(TypeToVis & data)
    {
        std::cout << __FILE__ << ":" << __LINE__ << " The implementetion has not been selected or is unknown " << std::endl;
    }

};


#include "insitu_catalyst_grid_dist.hpp"

#endif

#endif