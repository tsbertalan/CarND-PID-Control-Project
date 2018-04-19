#ifndef VECTOR_UTILS_H
#define VECTOR_UTILS_H

#include <vector>
#include <cmath>
#include <iostream>

// PRINTING
template <typename T>
void vec_print(std::vector<T> v) {
    std::cout << "[";
    for(auto & x : v) {
        std::cout << x << ", ";
    }
    std::cout << "]";
}

template <typename T>
void vec_print(std::vector<T> v, std::string name) {
    std::cout << name << " = ";
    vec_print(v);
    std::cout << std::endl;
}

// STATISTICS
template <typename T>
T vec_sum(std::vector<T> v) {
    T out = 0;
    for(auto & x : v) {
        out += x;
    }
    return out;
}

// TODO: Deal with zero-length vectors reasonably.
template<typename T>
double vec_mean(std::vector<T> v) {
    T sum = vec_sum(v);
    return (double) sum / v.size();
}

template<typename T>
double vec_stdd(std::vector<T> v, double mean) {
    double sum_squared_differences = 0;
    for(auto & x : v) {
        sum_squared_differences += std::pow(x - mean, 2);
    }
    return sum_squared_differences / (v.size() - 1);
}

template<typename T>
double vec_stdd(std::vector<T> v) {
    double mean = vec_mean(v);
    return vec_stdd(v, mean);
}

#endif /* VECTOR_UTILS_H */