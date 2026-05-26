#ifndef FSD_COMMON_LOGGING_HPP_
#define FSD_COMMON_LOGGING_HPP_

#include <cstdarg>
#include <cstdio>

namespace fsd_common {

/// Log level enum
enum class LogLevel { DEBUG, INFO, WARN, ERROR };

/// Log callback type
using LogCallback = void (*)(LogLevel level, const char* fmt, va_list args);

/// Set the global log callback. If not set, defaults to fprintf(stderr).
inline void SetLogCallback(LogCallback cb);

/// Get the current log callback
inline LogCallback GetLogCallback();

/// Log a message at the specified level
inline void Log(LogLevel level, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  LogCallback cb = GetLogCallback();
  if (cb) {
    cb(level, fmt, args);
  } else {
    // Default: print to stderr
    const char* prefix = "";
    switch (level) {
      case LogLevel::DEBUG: prefix = "[DEBUG] "; break;
      case LogLevel::INFO:  prefix = "[INFO] "; break;
      case LogLevel::WARN:  prefix = "[WARN] "; break;
      case LogLevel::ERROR: prefix = "[ERROR] "; break;
    }
    fprintf(stderr, "%s", prefix);
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\n");
  }
  va_end(args);
}

/// Convenience macros (use global namespace to avoid ADL issues)
#define FSD_LOG_DEBUG(...) ::fsd_common::Log(::fsd_common::LogLevel::DEBUG, __VA_ARGS__)
#define FSD_LOG_INFO(...) ::fsd_common::Log(::fsd_common::LogLevel::INFO, __VA_ARGS__)
#define FSD_LOG_WARN(...) ::fsd_common::Log(::fsd_common::LogLevel::WARN, __VA_ARGS__)
#define FSD_LOG_ERROR(...) ::fsd_common::Log(::fsd_common::LogLevel::ERROR, __VA_ARGS__)

// Implementation of global callback storage
namespace detail {
inline LogCallback g_log_callback = nullptr;
}  // namespace detail

inline void SetLogCallback(LogCallback cb) {
  detail::g_log_callback = cb;
}

inline LogCallback GetLogCallback() {
  return detail::g_log_callback;
}

}  // namespace fsd_common

#endif  // FSD_COMMON_LOGGING_HPP_
