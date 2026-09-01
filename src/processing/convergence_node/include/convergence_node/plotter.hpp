#ifndef PLOTTER_HPP
#define PLOTTER_HPP

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

class Plotter
{
public:
    Plotter() {
        // Nada aqui
    }

    void add_data(int window_id, const std::string& series_name, double x, double y) {
        data_[window_id][series_name].x.push_back(x);
        data_[window_id][series_name].y.push_back(y);
    }

    void plot() {
        plt::ion(); 

        for (auto& [window_id, series_map] : data_) {
            plt::figure(window_id);
            plt::clf();

            for (auto& [name, series] : series_map) {
                plt::named_plot(name, series.x, series.y);
            }

            plt::legend();

            std::string title = window_names_[window_id];
            if (title.empty())
                title = "Window " + std::to_string(window_id);

            plt::title(title);

            plt::draw();
        }

        plt::pause(0.001);
    }

    void set_window_name(int window_id, const std::string& name) {
        window_names_[window_id] = name;
    }

    void show() {
        plt::show();
    }

private:
    struct Series {
        std::vector<double> x;
        std::vector<double> y;
    };

    std::map<int, std::map<std::string, Series>> data_;
    std::map<int, std::string> window_names_;
};

#endif
