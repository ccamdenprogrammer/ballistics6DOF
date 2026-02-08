#pragma once

#include "solver.hpp"
#include <string>

void write_report(const Solver6DOF& solver, const std::string& filename);
void write_csv(const Solver6DOF& solver, const std::string& filename);
