#include "road.h"

#include <string>
#include <algorithm>


// Start to migrate functions

void Road::
    toUpperCaseInPlace(std::string &str)
{
    for (auto &c : str)
    {
        // Convert each character to uppercase
        c = std::toupper(static_cast<unsigned char>(c));
    }
}

void Road::
    trimInPlace(std::string &str)
{
    // Lambda function to check if a character is not a whitespace
    auto notSpace = [](int ch)
    { return !std::isspace(static_cast<unsigned char>(ch)); };

    // Find the first non-whitespace character
    auto start = std::find_if(str.begin(), str.end(), notSpace);
    // Erase leading whitespace
    str.erase(str.begin(), start);

    // Find the last non-whitespace character
    auto end = std::find_if(str.rbegin(), str.rend(), notSpace).base();
    // Erase trailing whitespace
    str.erase(end, str.end());
}
