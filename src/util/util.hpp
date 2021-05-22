/*
 * util.hpp
 *
 *  Created on: May 7, 2015
 *      Author: Pietro Incardona
 */

#include "config.h"

#include "util/SimpleRNG.hpp"

#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <boost/iostreams/device/mapped_file.hpp>


/*! \brief Compare two files, return true if they match
 *
 * \param file1 path1
 * \param file2 path2
 *
 * \return true if they match
 *
 */
static inline bool compare(std::string file1, std::string file2)
{
    boost::iostreams::mapped_file_source f1(file1);
    boost::iostreams::mapped_file_source f2(file2);

    if( f1.size() == f2.size() && std::equal(f1.data(), f1.data() + f1.size(), f2.data()) )
        return true;
    else
        return false;
}

//! RGB color struct
struct RGB
{
	//! Red
	float R;

	//! Green
	float G;

	//! Blue
	float B;

	//! Return the color as string
	std::string toString()
	{
		return std::to_string(R) + " " + std::to_string(G) + " " + std::to_string(B);
	}
};

/*! \brief Return the color sampled from one group
 *
 * groups:
 *
 * 0: Red
 * 1: Green
 * 2: Blue
 * 3: Yellow
 * 4: Cyan
 * 5: Magenta
 * 6: Orange
 * 7: Chartreuse-Green
 * 8: Spring Green
 * 9: Azure
 * 10: Violet
 * 11: Rose
 *
 * \param group
 *
 */

static inline struct RGB getColor(int group, SimpleRNG & d)
{
	struct RGB col;

	float s = (float)d.GetUniform();

	group = group % 12;

#ifdef ON_IO_UNIT_TESTS
	s = 0.5;
#endif

	if (group == 0)
	{
		col.R = s/2 + 0.5;
		col.G = 0.0;
		col.B = 0.0;
	}
	else if (group == 1)
	{
		col.R = 0.0;
		col.G = s/2 + 0.5;
		col.B = 0.0;
	}
	else if (group == 2)
	{
		col.R = 0.0;
		col.G = 0.0;
		col.B = s;
	}
	else if (group == 3)
	{
		col.R = s/2 + 0.5;
		col.G = s/2 + 0.5;
		col.B = 0.0;
	}
	else if (group == 4)
	{
		col.R = s/2 + 0.5;
		col.G = 0.0;
		col.B = s/2 + 0.5;
	}
	else if (group == 5)
	{
		col.R = 0.0;
		col.G = s/2 + 0.5;
		col.B = s/2 + 0.5;
	}
	else if (group == 6)
	{
		col.R = s/2 + 0.5;
		col.G = s/4 + 0.5;
		col.B = 0.0;
	}
	else if (group == 7)
	{
		col.R = s/4 + 0.5;
		col.G = s/2 + 0.5;
		col.B = 0.0;
	}
	else if (group == 8)
	{
		col.R = 0.0;
		col.G = s/2 + 0.5;
		col.B = s/4 + 0.5;
	}
	else if (group == 9)
	{
		col.R = 0.0;
		col.G = s/4 + 0.5;
		col.B = s/2 + 0.5;
	}
	else if (group == 10)
	{
		col.R = s/4 + 0.5;
		col.G = 0.0;
		col.B = s/2 + 0.5;
	}
	else
	{
		col.R = s/2 + 0.5;
		col.G = 0.0;
		col.B = s/4 + 0.5;
	}

	return col;
}

/*! \brief Check if one string end with a particular string
 *
 * \param fullString string to check
 * \param ending ending string to check
 *
 */
static inline bool hasEnding (std::string const &fullString, std::string const &ending)
{
    if (fullString.length() >= ending.length())
    {return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));}
    else
    {return false;}
}

static const std::string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";


static inline bool is_base64(unsigned char c) {
    return (isalnum(c) || (c == '+') || (c == '/'));
}



static const unsigned char vtkBase64UtilitiesEncodeTable[65] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";

//----------------------------------------------------------------------------
inline static unsigned char vtkBase64UtilitiesEncodeChar(unsigned char c)
{
    assert( c < 65 );
    return vtkBase64UtilitiesEncodeTable[c];
}

//----------------------------------------------------------------------------
static void EncodeTriplet(unsigned char i0,
                                       unsigned char i1,
                                       unsigned char i2,
                                       unsigned char *o0,
                                       unsigned char *o1,
                                       unsigned char *o2,
                                       unsigned char *o3)
{
    *o0 = vtkBase64UtilitiesEncodeChar((i0 >> 2) & 0x3F);
    *o1 = vtkBase64UtilitiesEncodeChar(((i0 << 4) & 0x30)|((i1 >> 4) & 0x0F));
    *o2 = vtkBase64UtilitiesEncodeChar(((i1 << 2) & 0x3C)|((i2 >> 6) & 0x03));
    *o3 = vtkBase64UtilitiesEncodeChar(i2 & 0x3F);
}

//----------------------------------------------------------------------------
static void EncodePair(unsigned char i0,
                                    unsigned char i1,
                                    unsigned char *o0,
                                    unsigned char *o1,
                                    unsigned char *o2,
                                    unsigned char *o3)
{
    *o0 = vtkBase64UtilitiesEncodeChar((i0 >> 2) & 0x3F);
    *o1 = vtkBase64UtilitiesEncodeChar(((i0 << 4) & 0x30)|((i1 >> 4) & 0x0F));
    *o2 = vtkBase64UtilitiesEncodeChar(((i1 << 2) & 0x3C));
    *o3 = '=';
}

//----------------------------------------------------------------------------
static void EncodeSingle(unsigned char i0,
                                      unsigned char *o0,
                                      unsigned char *o1,
                                      unsigned char *o2,
                                      unsigned char *o3)
{
    *o0 = vtkBase64UtilitiesEncodeChar((i0 >> 2) & 0x3F);
    *o1 = vtkBase64UtilitiesEncodeChar(((i0 << 4) & 0x30));
    *o2 = '=';
    *o3 = '=';
}

//----------------------------------------------------------------------------
static unsigned long EncodeToBase64(const unsigned char *input,
                                         unsigned long length,
                                         unsigned char *output,
                                         int mark_end)
{

    const unsigned char *ptr = input;
    const unsigned char *end = input + length;
    unsigned char *optr = output;

    // Encode complete triplet

    while ((end - ptr) >= 3)
    {
        EncodeTriplet(ptr[0], ptr[1], ptr[2],
                                          &optr[0], &optr[1], &optr[2], &optr[3]);
        ptr += 3;
        optr += 4;
    }

    // Encodes a 2-byte ending into 3 bytes and 1 pad byte and writes.

    if (end - ptr == 2)
    {
        EncodePair(ptr[0], ptr[1],
                                       &optr[0], &optr[1], &optr[2], &optr[3]);
        optr += 4;
    }

        // Encodes a 1-byte ending into 2 bytes and 2 pad bytes

    else if (end - ptr == 1)
    {
        EncodeSingle(ptr[0],
                                         &optr[0], &optr[1], &optr[2], &optr[3]);
        optr += 4;
    }

        // Do we need to mark the end

    else if (mark_end)
    {
        optr[0] = optr[1] = optr[2] = optr[3] = '=';
        optr += 4;
    }

    return optr - output;
}


#endif /* UTIL_HPP_ */
