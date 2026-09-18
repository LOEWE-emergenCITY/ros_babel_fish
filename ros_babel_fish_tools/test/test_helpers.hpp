// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_TOOLS_TEST_TEST_HELPERS_HPP
#define ROS_BABEL_FISH_TOOLS_TEST_TEST_HELPERS_HPP

// Helpers shared by the tool tests: running one of the CLI tools as a child process and capturing
// its output, parsing that output, temporary files and the checks every tool has in common.
// The tools are plain executables whose logic lives in main(), so they are tested end-to-end by
// spawning them against publishers/servers created by the test.

#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <filesystem>
#include <initializer_list>
#include <mutex>
#include <optional>
#include <regex>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace ros_babel_fish_tools_test
{

class ToolProcess
{
public:
  ToolProcess() = default;
  ToolProcess( const ToolProcess & ) = delete;
  ToolProcess &operator=( const ToolProcess & ) = delete;

  ~ToolProcess()
  {
    if ( pid_ > 0 && !exit_code_ )
      kill();
    join();
  }

  //! Starts @p executable with @p args. @p stdin_content is written to the child's stdin, after
  //! which stdin is closed (an empty string means the child sees EOF right away).
  bool start( const std::string &executable, const std::vector<std::string> &args,
              const std::string &stdin_content = "" )
  {
    // Close-on-exec so the child does not inherit descriptors of other test processes (the parent
    // is multi-threaded). dup2 clears the flag on the three standard descriptors.
    int out_pipe[2] = { -1, -1 }, err_pipe[2] = { -1, -1 }, in_pipe[2] = { -1, -1 };
    if ( pipe2( out_pipe, O_CLOEXEC ) != 0 || pipe2( err_pipe, O_CLOEXEC ) != 0 ||
         pipe2( in_pipe, O_CLOEXEC ) != 0 ) {
      close_all( { out_pipe[0], out_pipe[1], err_pipe[0], err_pipe[1], in_pipe[0], in_pipe[1] } );
      return false;
    }

    std::vector<std::string> argv_storage;
    argv_storage.push_back( executable );
    argv_storage.insert( argv_storage.end(), args.begin(), args.end() );
    std::vector<char *> argv;
    for ( auto &s : argv_storage ) argv.push_back( s.data() );
    argv.push_back( nullptr );
    // Prepared before fork; the child may only use async-signal-safe calls (no allocation).
    const std::string exec_error = "Failed to execute '" + executable + "'\n";

    pid_ = fork();
    if ( pid_ < 0 ) {
      close_all( { out_pipe[0], out_pipe[1], err_pipe[0], err_pipe[1], in_pipe[0], in_pipe[1] } );
      return false;
    }
    if ( pid_ == 0 ) {
      dup2( in_pipe[0], STDIN_FILENO );
      dup2( out_pipe[1], STDOUT_FILENO );
      dup2( err_pipe[1], STDERR_FILENO );
      execv( executable.c_str(), argv.data() );
      // Only reached if exec failed; make that visible in the captured stderr.
      ssize_t written = write( STDERR_FILENO, exec_error.data(), exec_error.size() );
      (void)written;
      _exit( 127 );
    }
    close_all( { in_pipe[0], out_pipe[1], err_pipe[1] } );

    if ( !stdin_content.empty() ) {
      const char *data = stdin_content.data();
      size_t remaining = stdin_content.size();
      while ( remaining > 0 ) {
        const ssize_t n = write( in_pipe[1], data, remaining );
        if ( n <= 0 )
          break;
        data += n;
        remaining -= static_cast<size_t>( n );
      }
    }
    close( in_pipe[1] );

    stdout_reader_ = std::thread( [this, fd = out_pipe[0]]() { read_all( fd, stdout_ ); } );
    stderr_reader_ = std::thread( [this, fd = err_pipe[0]]() { read_all( fd, stderr_ ); } );
    return true;
  }

  //! Waits up to @p timeout for the process to exit. @return The exit code or nullopt on timeout.
  //! A process killed by a signal reports 128 + signal number (as a shell would).
  std::optional<int> wait( std::chrono::milliseconds timeout )
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while ( !try_reap() ) {
      if ( std::chrono::steady_clock::now() >= deadline )
        return std::nullopt;
      std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
    }
    join();
    return exit_code_;
  }

  bool running() { return !try_reap(); }

  //! Sends SIGINT (Ctrl-C), which the tools handle via rclcpp's signal handler.
  void interrupt()
  {
    if ( pid_ > 0 && !exit_code_ )
      ::kill( pid_, SIGINT );
  }

  void kill()
  {
    if ( pid_ > 0 && !exit_code_ ) {
      ::kill( pid_, SIGKILL );
      int status = 0;
      waitpid( pid_, &status, 0 );
      exit_code_ = -1;
    }
  }

  //! Output captured so far (complete once wait() returned an exit code).
  std::string stdout_text()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return stdout_;
  }

  std::string stderr_text()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return stderr_;
  }

  //! Polls until @p needle shows up in stdout or @p timeout expires.
  bool wait_for_stdout( const std::string &needle, std::chrono::milliseconds timeout )
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while ( std::chrono::steady_clock::now() < deadline ) {
      if ( stdout_text().find( needle ) != std::string::npos )
        return true;
      if ( !running() ) // flush the readers before the final check
        break;
      std::this_thread::sleep_for( std::chrono::milliseconds( 20 ) );
    }
    join();
    return stdout_text().find( needle ) != std::string::npos;
  }

private:
  static void close_all( std::initializer_list<int> fds )
  {
    for ( int fd : fds )
      if ( fd >= 0 )
        close( fd );
  }

  static int decode_status( int status )
  {
    if ( WIFEXITED( status ) )
      return WEXITSTATUS( status );
    if ( WIFSIGNALED( status ) )
      return 128 + WTERMSIG( status );
    return -1;
  }

  //! Non-blocking check that records the exit code once the process has exited.
  //! @return True if the process has exited.
  bool try_reap()
  {
    if ( exit_code_ )
      return true;
    int status = 0;
    const pid_t ret = waitpid( pid_, &status, WNOHANG );
    if ( ret == 0 )
      return false;
    exit_code_ = ret == pid_ ? decode_status( status ) : -1;
    return true;
  }

  void read_all( int fd, std::string &target )
  {
    char buffer[4096];
    while ( true ) {
      const ssize_t n = read( fd, buffer, sizeof( buffer ) );
      if ( n <= 0 )
        break;
      std::lock_guard<std::mutex> lock( mutex_ );
      target.append( buffer, static_cast<size_t>( n ) );
    }
    close( fd );
  }

  void join()
  {
    if ( stdout_reader_.joinable() )
      stdout_reader_.join();
    if ( stderr_reader_.joinable() )
      stderr_reader_.join();
  }

  pid_t pid_ = -1;
  std::optional<int> exit_code_;
  std::thread stdout_reader_;
  std::thread stderr_reader_;
  std::mutex mutex_;
  std::string stdout_;
  std::string stderr_;
};

//! Splits @p text into lines, dropping a trailing empty line.
inline std::vector<std::string> split_lines( const std::string &text )
{
  std::vector<std::string> lines;
  size_t start = 0;
  while ( start < text.size() ) {
    size_t end = text.find( '\n', start );
    if ( end == std::string::npos )
      end = text.size();
    lines.push_back( text.substr( start, end - start ) );
    start = end + 1;
  }
  return lines;
}

//! Splits a CSV line into its fields (no quoting is used by the tools).
inline std::vector<std::string> split_csv( const std::string &line )
{
  std::vector<std::string> fields;
  size_t start = 0;
  while ( true ) {
    const size_t end = line.find( ',', start );
    fields.push_back( line.substr( start, end == std::string::npos ? end : end - start ) );
    if ( end == std::string::npos )
      break;
    start = end + 1;
  }
  return fields;
}

//! A file path in a fresh temporary directory that is removed with this object.
class TempDir
{
public:
  TempDir()
  {
    std::string tmpl = ( std::filesystem::temp_directory_path() / "rbf_tools_test_XXXXXX" ).string();
    const char *created = mkdtemp( tmpl.data() );
    path_ = created ? std::filesystem::path( created ) : std::filesystem::path();
  }

  ~TempDir()
  {
    std::error_code ec;
    if ( !path_.empty() )
      std::filesystem::remove_all( path_, ec );
  }

  std::string file( const std::string &name ) const { return ( path_ / name ).string(); }

private:
  std::filesystem::path path_;
};

// Regex fragments for the numbers in the tools' report lines.
//! A number as printed by the tools: fixed-point ("12.34", "-0.50") or integer ("12").
inline const std::string kNum = R"(-?\d+(?:\.\d+)?)";
inline const std::string kMinAvgMax = kNum + "/" + kNum + "/" + kNum;
//! Output of format_bytes, e.g. "512 B" or "1.50 KiB"
inline const std::string kBytes = R"(\d+(?:\.\d\d)? (?:B|KiB|MiB|GiB))";
inline const std::string kBandwidth = kBytes + "/s";

//! A line matched by a regex: the full line followed by its capture groups.
using Match = std::vector<std::string>;

//! Collects the lines of @p text matching @p re.
inline std::vector<Match> matching_lines( const std::string &text, const std::regex &re )
{
  std::vector<Match> result;
  for ( const std::string &line : split_lines( text ) ) {
    std::smatch m;
    if ( !std::regex_match( line, m, re ) )
      continue;
    Match groups;
    for ( const auto &sub : m ) groups.push_back( sub.str() );
    result.push_back( std::move( groups ) );
  }
  return result;
}

//! Runs @p tool until at least @p min_reports lines matching @p report_line were printed (or
//! @p timeout expired), then interrupts it. @return The exit code or nullopt if it did not exit.
inline std::optional<int>
run_until_reports( ToolProcess &tool, const std::regex &report_line, size_t min_reports,
                   std::chrono::milliseconds timeout = std::chrono::seconds( 20 ) )
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while ( std::chrono::steady_clock::now() < deadline && tool.running() ) {
    if ( matching_lines( tool.stdout_text(), report_line ).size() >= min_reports )
      break;
    std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
  }
  tool.interrupt();
  return tool.wait( std::chrono::seconds( 10 ) );
}

//! Size of @p msg in its serialized (CDR) form, as the tools report it.
template<typename T>
size_t serialized_size( const T &msg )
{
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<T>().serialize_message( &msg, &serialized );
  return serialized.size();
}

//! Checks that @p executable prints its usage (mentioning @p expected_option) to stderr and exits
//! successfully for both --help and -h, leaving stdout empty.
inline void expect_help_on_stderr( const std::string &executable, const std::string &expected_option )
{
  for ( const std::string flag : { "--help", "-h" } ) {
    ToolProcess tool;
    ASSERT_TRUE( tool.start( executable, { flag } ) );
    const auto code = tool.wait( std::chrono::seconds( 10 ) );
    ASSERT_TRUE( code.has_value() ) << flag;
    EXPECT_EQ( *code, 0 ) << flag;
    EXPECT_NE( tool.stderr_text().find( "Usage:" ), std::string::npos ) << tool.stderr_text();
    EXPECT_NE( tool.stderr_text().find( expected_option ), std::string::npos ) << tool.stderr_text();
    EXPECT_TRUE( tool.stdout_text().empty() ) << "help goes to stderr, stdout stays clean";
  }
}

struct InvalidArgumentCase {
  std::vector<std::string> args;
  std::string expected_error;
};

//! Checks that @p executable exits with code 1, prints the expected error to stderr and nothing to
//! stdout for each of @p cases.
inline void expect_rejects_arguments( const std::string &executable,
                                      const std::vector<InvalidArgumentCase> &cases )
{
  for ( const InvalidArgumentCase &c : cases ) {
    ToolProcess tool;
    ASSERT_TRUE( tool.start( executable, c.args ) );
    const auto code = tool.wait( std::chrono::seconds( 10 ) );
    ASSERT_TRUE( code.has_value() )
        << "did not exit for args: " << ::testing::PrintToString( c.args );
    EXPECT_EQ( *code, 1 ) << ::testing::PrintToString( c.args );
    EXPECT_NE( tool.stderr_text().find( c.expected_error ), std::string::npos )
        << ::testing::PrintToString( c.args ) << "\nstderr:\n"
        << tool.stderr_text();
    EXPECT_TRUE( tool.stdout_text().empty() ) << tool.stdout_text();
  }
}

} // namespace ros_babel_fish_tools_test

#endif // ROS_BABEL_FISH_TOOLS_TEST_TEST_HELPERS_HPP
