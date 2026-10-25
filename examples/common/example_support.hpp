#pragma once

// Small helpers shared by the examples: optional CSV logging and a uniform
// pass/fail report. They are deliberately kept out of the SDK headers.

#include <cstdio>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <string>
#include <string_view>

namespace example {

/**
 * Writes a CSV file when the example is started with `--csv <path>`.
 * Otherwise every call is a no-op, so examples can log unconditionally.
 */
class CsvLog {
  public:
    CsvLog(int argc, char** argv, std::initializer_list<std::string_view> columns) {
        for (int i = 1; i + 1 < argc; ++i) {
            if (std::string_view{argv[i]} == "--csv") {
                file_.open(argv[i + 1]);
                break;
            }
        }
        if (!file_) {
            return;
        }
        bool first = true;
        for (const auto column : columns) {
            file_ << (first ? "" : ",") << column;
            first = false;
        }
        file_ << '\n';
    }

    void row(std::initializer_list<double> values) {
        if (!file_.is_open()) {
            return;
        }
        bool first = true;
        for (const double value : values) {
            file_ << (first ? "" : ",") << value;
            first = false;
        }
        file_ << '\n';
    }

  private:
    std::ofstream file_;
};

/// Prints a labelled value with a unit.
inline void print_value(std::string_view label, double value, std::string_view unit = "") {
    std::printf("  %-28.*s %12.5f %.*s\n", static_cast<int>(label.size()), label.data(), value,
                static_cast<int>(unit.size()), unit.data());
}

/// Checks an acceptance criterion and prints PASS/FAIL. Returns `ok`.
inline bool check(bool ok, std::string_view description) {
    std::printf("  [%s] %.*s\n", ok ? "PASS" : "FAIL", static_cast<int>(description.size()),
                description.data());
    return ok;
}

inline void title(std::string_view text) {
    std::cout << "\n== " << text << " ==\n";
}

inline int finish(bool ok) {
    std::cout << (ok ? "\nresult: OK\n" : "\nresult: FAILED\n");
    return ok ? 0 : 1;
}

} // namespace example
