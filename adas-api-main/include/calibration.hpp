#ifndef CALIBRATION_HPP

#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <sstream>
#include <cmath>

std::vector<float> read_file( std::ifstream &file )
{
    std::vector<float> data;

    std::copy( std::istream_iterator<float>(file), 
               std::istream_iterator<float>(), 
               std::back_inserter(data) );

    return data;
}


float calibration( const std::vector<float> &calib, const float input )
{
    float output = 0.0f;
    for( size_t i=0; i<calib.size(); ++i )
        output += calib[i] * powf( input, i );
    return output;
}

#endif // CALIBRATION_HPP
