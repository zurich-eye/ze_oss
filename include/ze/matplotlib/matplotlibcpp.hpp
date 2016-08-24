// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential


// Modified to accept eigen types as primary input. Exposes wrappers for
// std::vector

#pragma once

#include <vector>
#include <map>
#include <numeric>
#include <stdexcept>
#include <iostream>
#include <ze/common/types.h>
#include <initializer_list>


namespace ze {
namespace plt {

//! Enable interactive mode.
bool ion();

//! Create a new figure.
bool figure(std::string i = "");

//! Histogram.
bool hist(
    const Eigen::Ref<const VectorX>& x,
    const double bins = 10,
    const std::string histtype = "bar");

//! Every row of X is the data for a box.
bool boxplot(
    const Eigen::Ref<const MatrixX>& x,
    const std::vector<std::string>& labels);
bool boxplot(
    const Eigen::Ref<const MatrixX>& x,
    std::initializer_list<const std::string> labels);
bool boxplot(
    const Eigen::Ref<const MatrixX>& x);

//! Create a subplot.
bool subplot(const size_t nrows, const size_t ncols, const size_t plot_number);

//! Create an x/y plot with properties as map.
bool plot(
    const Eigen::Ref<const MatrixX>& x,
    const Eigen::Ref<const MatrixX>& y,
    const std::map<std::string, std::string>& keywords);

//! Create an x/y plot with properties in string.
bool plot(
    const Eigen::Ref<const MatrixX>& x,
    const Eigen::Ref<const MatrixX>& y,
    const std::string& s = "");

//! Create an x/y plot with name as label.
bool labelPlot(
    const std::string& name,
    const Eigen::Ref<const MatrixX>& x,
    const Eigen::Ref<const MatrixX>& y,
    const std::string& format = "");
bool labelPlot(
    const std::string& name,
    const Eigen::Ref<const MatrixX>& y,
    const std::string& format = "");

bool plot(
    const Eigen::Ref<const MatrixX>& x,
    const std::string& format = "");

// -----------------------------------------------------------------------------
//! @name std::vector wrappers.
//! @{
bool plot(
    const std::vector<real_t>& y,
    const std::string& format = "");

bool plot(
    const std::vector<real_t>& x,
    const std::vector<real_t>& y,
    const std::map<std::string, std::string>& keywords);

bool plot(
    const std::vector<real_t>& x,
    const Eigen::Ref<const MatrixX>& y,
    const std::map<std::string, std::string>& keywords);

bool plot(
    const std::vector<real_t>& x,
    const std::vector<real_t>& y,
    const std::string& s = "");
bool plot(
    const std::vector<real_t>& x,
    const Eigen::Ref<const MatrixX>& y,
    const std::string& s = "");

bool labelPlot(
    const std::string& name,
    const std::vector<real_t>& x,
    const std::vector<real_t>& y,
    const std::string& format = "");

bool labelPlot(
    const std::string& name,
    const std::vector<real_t>& x,
    const Eigen::Ref<const MatrixX>& y,
    const std::string& format = "");
//! @}

// -----------------------------------------------------------------------------
//! @name Plot settings.
//! @{
void legend();

void ylim(real_t min, real_t max);

void xlim(real_t xmin, real_t xmax);

void title(const std::string &titlestr);

void axis(const std::string &axisstr);

void xlabel(const std::string &str);

void ylabel(const std::string &str);

void grid(bool flag);

void show(bool block = true);

void save(const std::string& filename);
//! @}

} // namespace plt
} // namespace ze
