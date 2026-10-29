#include <string_view>

#include "msim/version.hpp"

int main() { return std::string_view(msim::version()) == "0.1.0" ? 0 : 1; }
