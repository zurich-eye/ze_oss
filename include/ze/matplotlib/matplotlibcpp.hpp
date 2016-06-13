// The MIT License (MIT)
//
// Copyright (c) 2014 Benno Evers
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Modified to accept eigen types as primary input. Exposes wrappers for
// std::vector

#pragma once

#include <vector>
#include <map>
#include <numeric>
#include <stdexcept>
#include <iostream>
#include <ze/common/types.h>

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

//! Create a subplot.
bool subplot(const size_t nrows, const size_t ncols, const size_t plot_number);

//! Create an x/y plot with properties as map.
bool plot(
    const Eigen::Ref<const VectorX>& x,
    const Eigen::Ref<const VectorX>& y,
    const std::map<std::string, std::string>& keywords);

//! Create an x/y plot with properties in string.
bool plot(
    const Eigen::Ref<const VectorX>& x,
    const Eigen::Ref<const VectorX>& y,
    const std::string& s = "");

//! Create an x/y plot with name as label.
bool labelPlot(
    const std::string& name,
    const Eigen::Ref<const VectorX>& x,
    const Eigen::Ref<const VectorX>& y,
    const std::string& format = "");


bool plot(
    const Eigen::Ref<const VectorX>& x,
    const std::string& format = "");

// -----------------------------------------------------------------------------
//! @name std::vector wrappers.
//! @{
bool plot(
    const std::vector<FloatType>& y,
    const std::string& format = "");

bool plot(
    const std::vector<FloatType>& x,
    const std::vector<FloatType>& y,
    const std::map<std::string, std::string>& keywords);

bool plot(
    const std::vector<FloatType>& x,
    const std::vector<FloatType>& y,
    const std::string& s = "");

bool labelPlot(
    const std::string& name,
    const std::vector<FloatType>& x,
    const std::vector<FloatType>& y,
    const std::string& format = "");
//! @}

// -----------------------------------------------------------------------------
//! @name Plot settings.
//! @{
void legend();

void ylim(FloatType min, FloatType max);

void xlim(FloatType xmin, FloatType xmax);

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
