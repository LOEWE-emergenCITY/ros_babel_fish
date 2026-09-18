// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_TOOLS_STATS_COMMON_HPP
#define ROS_BABEL_FISH_TOOLS_STATS_COMMON_HPP

// Helpers shared by the stats and service_stats tools: metric accumulation, output formatting and
// command line handling.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace ros_babel_fish_tools
{

//! Tracks min/max/sum/count of a single metric over a reporting window.
struct Accumulator {
  uint64_t count = 0;
  int64_t sum = 0;
  int64_t min = 0;
  int64_t max = 0;

  void add( int64_t value )
  {
    if ( count == 0 ) {
      min = value;
      max = value;
    } else {
      min = std::min( min, value );
      max = std::max( max, value );
    }
    sum += value;
    ++count;
  }

  double avg() const { return count == 0 ? 0.0 : static_cast<double>( sum ) / count; }

  void reset() { *this = Accumulator{}; }
};

//! Formats "min/avg/max" for @p acc (each value divided by @p divisor) as fixed-point numbers with
//! @p precision decimals, or "n/a" if no samples were collected.
inline std::string format_min_avg_max( const Accumulator &acc, double divisor, int precision )
{
  if ( acc.count == 0 )
    return "n/a";
  return std::format( "{0:.{3}f}/{1:.{3}f}/{2:.{3}f}", acc.min / divisor, acc.avg() / divisor,
                      acc.max / divisor, precision );
}

//! Formats a byte count with a binary unit (B, KiB, MiB, GiB), e.g. "1.50 KiB".
inline std::string format_bytes( double bytes, const char *suffix = "" )
{
  const char *units[] = { "B", "KiB", "MiB", "GiB" };
  int unit = 0;
  while ( bytes >= 1024.0 && unit < 3 ) {
    bytes /= 1024.0;
    ++unit;
  }
  return std::format( "{:.{}f} {}{}", bytes, unit == 0 ? 0 : 2, units[unit], suffix );
}

//! Formats a bandwidth with a binary unit, e.g. "1.50 KiB/s".
inline std::string format_bytes_per_sec( double bytes_per_sec )
{
  return format_bytes( bytes_per_sec, "/s" );
}

//! Parses a strictly positive, finite number. @return False on junk ("5x"), empty parses or
//! non-positive values.
inline bool parse_positive_number( const std::string &value, double &out )
{
  char *end = nullptr;
  out = std::strtod( value.c_str(), &end );
  return end != value.c_str() && *end == '\0' && std::isfinite( out ) && out > 0.0;
}

//! Parses a strictly positive, finite number of seconds whose nanosecond representation fits in an
//! int64_t (so it can be used as a timer period or timeout without overflowing).
inline bool parse_positive_seconds( const std::string &value, double &out )
{
  return parse_positive_number( value, out ) &&
         out <= static_cast<double>( std::numeric_limits<int64_t>::max() ) / 1e9;
}

//! Fetches the value following the option at @p i in @p args and advances @p i past it.
//! @return False (after printing an error) if the option is the last argument.
inline bool take_option_value( const std::vector<std::string> &args, size_t &i, std::string &out )
{
  if ( i + 1 >= args.size() ) {
    std::cerr << "Missing value for " << args[i] << std::endl;
    return false;
  }
  out = args[++i];
  return true;
}

//! Opens @p path for writing CSV measurements, asking for confirmation on stdin before overwriting
//! an existing file. @return False (after printing an error) if the file was not opened.
inline bool open_csv_output( const std::string &path, std::ofstream &csv )
{
  if ( std::filesystem::exists( path ) ) {
    std::cout << "Output file '" << path << "' already exists. Overwrite? [y/N]: " << std::flush;
    std::string answer;
    // A non-interactive stdin (EOF) leaves answer empty, i.e. defaults to not overwriting.
    std::getline( std::cin, answer );
    if ( answer != "y" && answer != "Y" && answer != "yes" ) {
      std::cerr << "Aborting; output file not overwritten." << std::endl;
      return false;
    }
  }
  csv.open( path );
  if ( !csv.is_open() ) {
    std::cerr << "Failed to open output file: " << path << std::endl;
    return false;
  }
  return true;
}

} // namespace ros_babel_fish_tools

#endif // ROS_BABEL_FISH_TOOLS_STATS_COMMON_HPP
