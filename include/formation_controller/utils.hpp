#ifndef UTILS_HPP
#define UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace std;

inline string replace_ns_prefix(string &ns, const string &input, bool remove_leading_slash = true)
{
    if (input.rfind("NS", 0) != 0) return input; // Return the original string if "NS" is not at the start

    // Remove leading "/" if present
    if (remove_leading_slash && ns.rfind("/", 0) == 0) {
        ns = ns.substr(1);
    }
    string resultant_string = ns + input.substr(2); // Replace "NS" with the namespace
    return resultant_string; // Replace "NS" with the namespace
}

#endif // UTILS_HPP