#ifndef PLOTTER_HPP
#define PLOTTER_HPP

#include <string>
#include <vector>
#include <map>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

class Plotter
{
public:
    Plotter() = default;

    // adiciona um ponto à série
    void add_data(const std::string& series_name, double x, double y)
    {
        xs_[series_name].push_back(x);
        ys_[series_name].push_back(y);
    }

    // atualiza o gráfico
    void plot()
    {
        plt::clf(); // limpa figura
        for (auto& [name, x_vec] : xs_)
        {
            plt::named_plot(name, xs_[name], ys_[name]);
        }
        plt::legend();
        plt::xlabel("Mensagem #");
        plt::ylabel("Tempo do tópico (us)");
        plt::pause(0.01); // atualização em tempo quase real
    }

private:
    std::map<std::string, std::vector<double>> xs_;
    std::map<std::string, std::vector<double>> ys_;
};

#endif // PLOTTER_HPP