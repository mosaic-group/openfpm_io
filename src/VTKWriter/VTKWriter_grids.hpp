/*
 * VTKWriter_grids.hpp
 *
 *  Created on: May 5, 2015
 *      Author: Pietro Incardona
 */

#ifndef VTKWRITER_GRIDS_HPP_
#define VTKWRITER_GRIDS_HPP_

#include <boost/mpl/pair.hpp>
#include "VTKWriter_grids_util.hpp"
#include "is_vtk_writable.hpp"

/*! \brief It store one grid
 *
 * \tparam Grid type of grid
 * \tparam St type of space where the grid is defined
 *
 */
template <typename Grid, typename St>
class ele_g
{
public:

	typedef Grid value_type;

	ele_g(const Grid & g, const Point<Grid::dims,St> & offset, const Point<Grid::dims,St> & spacing, const Box<Grid::dims,St> & dom)
	:g(g),offset(offset),spacing(spacing),dom(dom)
	{}

	//! Dataset name
	std::string dataset;
	//! Grid
	const Grid & g;
	//! offset where it start
	Point<Grid::dims,St> offset;
	// spacing of the grid
	Point<Grid::dims,St> spacing;
	// Part of the grid that is real domain
	Box<Grid::dims,size_t> dom;
};



/*! \brief this class is a functor for "for_each" algorithm
 *
 * This class is a functor for "for_each" algorithm. For each
 * element of the boost::vector the operator() is called.
 * Is mainly used to produce at output for each property
 *
 * \tparam ele_g element that store the grid and its attributes
 * \param St type of space where the grid live
 *
 */

template<typename ele_g, typename St>
struct prop_out_g
{
	//! property output string
	std::string & v_out;

	//! grid that we are processing
	const openfpm::vector_std< ele_g > & vg;

	//! File type
	file_type ft;

	//! list of names for the properties
	const openfpm::vector<std::string> & prop_names;

	/*! \brief constructor
	 *
	 * \param v_out string to fill with the vertex properties
	 * \param vg vector of elements to write
	 * \param prop_names properties name
	 * \param ft file type
	 *
	 */
	prop_out_g(std::string & v_out, const openfpm::vector_std< ele_g > & vg, const openfpm::vector<std::string> & prop_names ,file_type ft)
	:v_out(v_out),vg(vg),ft(ft),prop_names(prop_names)
	{};

	/*! It produce an output for each propert
	 *
	 * \param t prop-id
	 *
	 */
    template<typename T>
    void operator()(T& t) const
    {
    	typedef typename boost::mpl::at<typename ele_g::value_type::value_type::type,boost::mpl::int_<T::value>>::type ptype;
    	typedef typename std::remove_all_extents<ptype>::type base_ptype;

    	meta_prop_new<boost::mpl::int_<T::value> ,ele_g,St, ptype, is_vtk_writable<base_ptype>::value > m(vg,v_out,prop_names,ft);
    }

    /*! \brief Write the last property
     *
     *
     *
     */
    void lastProp()
	{
        std::string v_outToEncode,v_Encoded;
        // Create point data properties
		//v_out += "SCALARS domain float\n";
		// Default lookup table
		//v_out += "LOOKUP_TABLE default\n";
        v_out += "        <DataArray type=\"Float32\" Name=\"domain\"";
        if (ft == file_type::ASCII) {
            v_out += " format=\"ascii\">\n";
        }
        else {
            v_out += " format=\"binary\">\n";
        }

        if (ft == file_type::BINARY) {
            v_outToEncode.append(8,0);
        }
		// Produce point data
		for (size_t k = 0 ; k < vg.size() ; k++)
		{
			//! Get a vertex iterator
			auto it = vg.get(k).g.getIterator();

			// if there is the next element
			while (it.isNext())
			{
				if (ft == file_type::ASCII)
				{
		 			if (vg.get(k).dom.isInside(it.get().toPoint()) == true)
		 			{v_outToEncode += "1.0\n";}
					else
					{v_outToEncode += "0.0\n";}
				}
				else
				{
					if (vg.get(k).dom.isInside(it.get().toPoint()) == true)
					{
						float one = 1;
                        v_outToEncode.append((const char *)&one,sizeof(int));
					}
					else
					{
						float zero = 0;
                        v_outToEncode.append((const char *)&zero,sizeof(int));
					}
				}

				// increment the iterator and counter
				++it;
			}
		}
        if (ft == file_type::BINARY)
        {
            *(size_t *) &v_outToEncode[0] = v_outToEncode.size()-sizeof(size_t);
            v_Encoded.resize(v_outToEncode.size()/3*4+4);
            size_t sz=EncodeToBase64((const unsigned char*)&v_outToEncode[0],v_outToEncode.size(),(unsigned char *)&v_Encoded[0],0);
            v_Encoded.resize(sz);
            v_out += v_Encoded + "\n";
        }
        else{
            v_out += v_outToEncode;
        };
		v_out+="        </DataArray>\n";
	}
};

/*!
 *
 * It write a VTK format file in case of grids defined on a space
 *
 * \tparam boost::mpl::pair<G,S>
 *
 * where G is the type of grid S is the type of space, float, double ...
 *
 */
template <typename pair>
class VTKWriter<pair,VECTOR_GRIDS>
{
	//! Vector of grids

	openfpm::vector< ele_g<typename pair::first,typename pair::second> > vg;
	/*! \brief Get the total number of points
	 *
	 * \return the total number
	 *
	 */
	size_t get_total()
	{
		size_t tot = 0;

		//! Calculate the full number of vertices
		for (size_t i = 0 ; i < vg.size() ; i++)
		{
			tot += vg.get(i).g.size();
		}
		return tot;
	}

	/*! \brief It get the vertex properties list
	 *
	 * It get the vertex properties list of the vertex defined as VTK header
	 *
	 * \return a string that define the vertex properties in graphML format
	 *
	 */

	std::string get_vertex_properties_list(file_type & opt)
	{
		//! vertex property output string
		std::string v_out;

		v_out += "      <Verts>\n";
        if (opt == file_type::ASCII)

        {
            v_out+="        <DataArray type=\"Int64\" Name=\"connectivity\" format=\"ascii\">\n";
        }
        else
        {
            v_out+="        <DataArray type=\"Int64\" Name=\"connectivity\" format=\"binary\">\n";
        }

        // write the number of vertex
		//v_out += "VERTICES " + std::to_string(get_total()) + " " + std::to_string(get_total() * 2) + "\n";
		// return the vertex properties string
		return v_out;
	}

	/*! \brief It get the vertex properties list
	 *
	 * It get the vertex properties list of the vertex defined as a VTK header
	 *
	 * \return a string that define the vertex properties in graphML format
	 *
	 */
	std::string get_point_properties_list()
	{
		//! vertex property output string
		std::string v_out;

		// write the number of vertex

		v_out += "    <Piece NumberOfPoints=\"" + std::to_string(get_total()) + "\" " +"NumberOfVerts=\"" + std::to_string(get_total()) + "\">\n";

		// return the vertex properties string
		return v_out;
	}

	/*! \brief Create the VTK point definition
	 *
	 * \param ft file type
	 *
	 * \return the string with the point list
	 *
	 */
	std::string get_point_list(file_type & opt)
	{
		//! vertex node output string
		std::stringstream v_out;

		v_out<<"      <Points>\n";

        std::stringstream binaryToEncode;
        if (std::is_same<typename pair::second,float>::value == true)
        {
			binaryToEncode << std::setprecision(7);
			if (opt == file_type::ASCII)
			{v_out<<"        <DataArray type=\"Float32\" Name=\"Points\" NumberOfComponents=\"3\" format=\"ascii\">\n";}
			else
			{v_out<<"        <DataArray type=\"Float32\" Name=\"Points\" NumberOfComponents=\"3\" format=\"binary\">\n";}
		}
        else
        {
			binaryToEncode << std::setprecision(16);
			if (opt == file_type::ASCII)
			{v_out<<"        <DataArray type=\"Float64\" Name=\"Points\" NumberOfComponents=\"3\" format=\"ascii\">\n";}
			else
			{v_out<<"        <DataArray type=\"Float64\" Name=\"Points\" NumberOfComponents=\"3\" format=\"binary\">\n";}
		}

		//! For each defined grid
        if (opt == file_type::BINARY)
        {
            size_t tmp=0;
            binaryToEncode.write((const char *)&tmp,sizeof(tmp));
        }

        for (size_t i = 0 ; i < vg.size() ; i++)
		{
			//! write the particle position
			auto it = vg.get(i).g.getIterator();

			// if there is the next element
			while (it.isNext())
			{
				Point<pair::first::dims,typename pair::second> p;
		 		p = it.get().toPoint();
		 		p = pmul(p,vg.get(i).spacing) + vg.get(i).offset;

				output_point_new<pair::first::dims,typename pair::second>(p,binaryToEncode,opt);

				// increment the iterator and counter
				++it;
			}
		}
		//! In case of binary we have to add a new line at the end of the list
		if (opt == file_type::BINARY){
		    std::string buffer_out,buffer_bin;
            buffer_bin=binaryToEncode.str();
            *(size_t *)&buffer_bin[0]=buffer_bin.size()-8;
		    buffer_out.resize(buffer_bin.size()/3*4+4);
			unsigned long sz = EncodeToBase64((const unsigned char*)&buffer_bin[0],buffer_bin.size(),(unsigned char*)&buffer_out[0],0);
			buffer_out.resize(sz);
		    v_out << buffer_out<<std::endl;
        }
		else
		{
		    v_out<<binaryToEncode.str();
		}
        v_out<<"        </DataArray>\n";
        v_out<<"      </Points>\n";
		// return the vertex list
		return v_out.str();
	}

	/*! \brief Create the VTK vertex definition
	 *
	 * \param ft file type
	 *
	 */
	std::string get_vertex_list(file_type ft)
	{
		// vertex node output string
		std::string v_out,v_outToEncode,v_Encoded;

        size_t k = 0;
        if (ft == file_type::BINARY) {
            v_outToEncode.append(8,0);
        }
		for (size_t i = 0 ; i < vg.size() ; i++)
		{
			//! For each grid point create a vertex
			auto it = vg.get(i).g.getIterator();

			while (it.isNext())
			{
				output_vertex_new(k,v_outToEncode,ft);

				++k;
				++it;
			}
		}
		//! In case of binary we have to add a new line at the end of the list
		if (ft == file_type::BINARY)
		{
            *(size_t *) &v_outToEncode[0] = v_outToEncode.size()-sizeof(size_t);
		    v_Encoded.resize(v_outToEncode.size()/3*4+4);
		    size_t sz=EncodeToBase64((const unsigned char*)&v_outToEncode[0],v_outToEncode.size(),(unsigned char *)&v_Encoded[0],0);
		    v_Encoded.resize(sz);
		    v_out += v_Encoded + "\n";
		}
		else{
            v_out += v_outToEncode;
		};
        v_out += "        </DataArray>\n";
        v_out += "                <DataArray type=\"Int64\" Name=\"offsets\" ";

        if (ft == file_type::ASCII)
        {
            v_out += "format=\"ascii\">\n";
        }
        else{
            v_out += "format=\"binary\">\n";
        }

        k=0;
        v_outToEncode.clear();
        if (ft == file_type::BINARY) {
            v_outToEncode.append(8,0);
        }

        for (size_t i = 0 ; i < vg.size() ; i++)
        {
            //! For each grid point create a vertex
            auto it = vg.get(i).g.getIterator();
            while (it.isNext())
            {
                output_vertex_new(k+1,v_outToEncode,ft);

                ++k;
                ++it;
            }
        }
        if (ft == file_type::BINARY)
        {
            *(size_t *) &v_outToEncode[0] = v_outToEncode.size()-sizeof(size_t);
            v_Encoded.resize(v_outToEncode.size()/3*4+4);
            size_t sz=EncodeToBase64((const unsigned char*)&v_outToEncode[0],v_outToEncode.size(),(unsigned char *)&v_Encoded[0],0);
            v_Encoded.resize(sz);
            v_out += v_Encoded + "\n";
        }
        else{
            v_out += v_outToEncode;
        };
        v_out += "        </DataArray>\n";
        v_out += "      </Verts>\n";
		// return the vertex list
		return v_out;
	}

	/*! \brief Get the point data header
	 *
	 * \return a string with the point data header for VTK format
	 *
	 */

	std::string get_point_data_header()
	{
		std::string v_out;

		v_out += "      <PointData>\n";

		return v_out;
	}

public:

	/*!
	 *
	 * VTKWriter constructor
	 *
	 */
	VTKWriter()
	{}

	/*! \brief Add grid dataset
	 *
	 * \param g Grid to add
	 * \param offset grid offset
	 * \param spacing spacing of the grid
	 * \param dom part of the space that is the domain
	 *
	 */
	void add(const typename pair::first & g,
			 const Point<pair::first::dims,typename pair::second> & offset,
			 const Point<pair::first::dims,typename pair::second> & spacing,
			 const Box<pair::first::dims,typename pair::second> & dom)
	{
		ele_g<typename pair::first,typename pair::second> t(g,offset,spacing,dom);

		vg.add(t);
	}

	/*! \brief It write a VTK file from a graph
	 *
	 * \tparam prp_out which properties to output [default = -1 (all)]
	 *
	 * \param file path where to write
	 * \param name of the graph
	 * \param prop_names properties name (can also be a vector of size 0)
	 * \param ft specify if it is a VTK BINARY or ASCII file [default = ASCII]
	 *
	 * \return true if the function write successfully
	 *
	 */
	template<int prp = -1> bool write(std::string file,
									  const openfpm::vector<std::string> & prop_names,
									  std::string f_name = "grids",
									  file_type ft = file_type::ASCII)
	{
		// Header for the vtk
		std::string vtk_header;
		// Point list of the VTK
		std::string point_list;
		// Vertex list of the VTK
		std::string vertex_list;
		// Graph header
		std::string vtk_binary_or_ascii;
		// vertex properties header
		std::string point_prop_header;
		// edge properties header
		std::string vertex_prop_header;
		// Data point header
		std::string point_data_header;
		// Data point
		std::string point_data;

		// VTK header
		vtk_header = "<VTKFile type=\"PolyData\" version=\"1.0\" byte_order=\"LittleEndian\" header_type=\"UInt64\">\n";

        vtk_header +="  <PolyData>\n";

		// point properties header
		point_prop_header = get_point_properties_list();

		// Get point list
		point_list = get_point_list(ft);

		// vertex properties header
		vertex_prop_header = get_vertex_properties_list(ft);

		// Get vertex list
		vertex_list = get_vertex_list(ft);

		// Get the point data header
		point_data_header = get_point_data_header();

		// For each property in the vertex type produce a point data

		prop_out_g< ele_g<typename pair::first,typename pair::second>, typename pair::second > pp(point_data, vg, prop_names, ft);

		if (prp == -1)
		{boost::mpl::for_each< boost::mpl::range_c<int,0, pair::first::value_type::max_prop> >(pp);}
		else
		{boost::mpl::for_each< boost::mpl::range_c<int,prp, prp> >(pp);}

		// Add the last property
		pp.lastProp();

		std::string closingFile="      </PointData>\n    </Piece>\n  </PolyData>\n</VTKFile>";

		// write the file
		std::ofstream ofs(file);

		// Check if the file is open
		if (ofs.is_open() == false)
		{std::cerr << "Error cannot create the VTK file: " + file + "\n";}

		ofs << vtk_header << point_prop_header << point_list <<
				vertex_prop_header << vertex_list << point_data_header << point_data << closingFile;

		// Close the file

		ofs.close();

		// Completed succefully
		return true;
	}
};


#endif /* VTKWRITER_GRAPH_HPP_ */
