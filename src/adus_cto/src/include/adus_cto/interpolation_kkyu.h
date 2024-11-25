#ifndef __INTERPOLATION_KKYU_H__
#define __INTERPOLATION_KKYU_H__

// #include "interpolation_kkyu.h"

#include "def.h"

class interpolation_kkyu
{
private:
    /* data */

public:
    interpolation_kkyu(/* args */);
    ~interpolation_kkyu();

public:
    // hermite
    int sign_decision(double k);
    int32_t find_arr_index(std::vector<int32_t> tmp_arr, int32_t value);
    void arr_min_max(std::vector<int32_t> tmp_arr, int32_t &min, int32_t &max);
    bool ramp_pattern(double sample_num, double arr_size, std::vector<double> &xi_arr);
    void chk_monotonic(std::vector<double> y, std::vector<double> &x_out, std::vector<double> &y_out);
    int32_t interpol_prepare(std::vector<double> y, std::vector<double> xi_arr, std::vector<double> &x_out, std::vector<double> &y_out, std::vector<double> &xi_out);
    double one_side_slope(std::vector<double> delta, std::vector<double> h);
    double interior_slope(std::vector<double> delta, std::vector<double> h);
    void y_derivative(std::vector<double> y, std::vector<double> x, std::vector<double> &y_derivate);
    void hermite_polynomial(std::vector<double> y, std::vector<double> y_deriv, std::vector<double> x, int32_t index, std::vector<double> &hermite);
    double polynomial_calc(std::vector<double> Px, double a);
    void seq_0(double xi_each_var, std::vector<double> x_out, int32_t x_out_size1, bool ascending_bool, int32_t start_index_new, int32_t &sq0_out0, int32_t &sq0_out1, bool &sq0_out2);
    void seq_1(double xi_each_var, std::vector<double> x_out, bool ascending_bool, int32_t sq0_out0, int32_t sq0_out1, bool sq0_out2, int32_t &sq1_out0);
    double seq_2(double xi_each_var, std::vector<double> x_out, int32_t sq1_out0);
    void hermite_method(std::vector<double> x_out, std::vector<double> y_out, std::vector<double> xi_out, std::vector<double> y_derivate, std::vector<double> &yi);
    void hermite_interpolation(std::vector<double> origin_vector, int32_t sample_num, std::vector<double> &interpolation_vector);
    void path_hermite_inter(std::vector<waypoints> origin_vec, int sample_num, std::vector<waypoints> &hermite_vec);

    // lerp
    void linear_interpolation(std::vector<double> origin_vector, int32_t sample_num, std::vector<double> &interpolation_vector);

    // relocate
    void relocate_calc(double locate_dist, int n0, std::vector<waypoints> interpol_path, double &calc_dist, int &id_n);
    void re_locate(double locate_dist, std::vector<waypoints> interpol_path, std::vector<waypoints> &relocate_path);
};

#endif
