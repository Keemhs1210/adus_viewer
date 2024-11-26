#include "interpolation_kkyu.h"

interpolation_kkyu::interpolation_kkyu(/* args */)
{
}

interpolation_kkyu::~interpolation_kkyu()
{
}

int interpolation_kkyu::sign_decision(double k)
{
    int sign_k;

    if (k == 0)
    {
        sign_k = 0;
    }
    else
    {
        if (k / abs(k) == 1)
        {
            sign_k = 1;
        }
        else
        {
            sign_k = -1;
        }
    }
    return sign_k;
}

int32_t interpolation_kkyu::find_arr_index(std::vector<int32_t> tmp_arr, int32_t value)
{
    int32_t ele;
    int i;

    for (i = 0; i < tmp_arr.size(); i++)
    {
        if (value == tmp_arr[i])
            break;
    }

    if (i < tmp_arr.size())
    {
        ele = i;
    }
    else
    {
        ele = -1;
    }
    return ele;
}

void interpolation_kkyu::arr_min_max(std::vector<int32_t> tmp_arr, int32_t &min, int32_t &max)
{
    int32_t max_t = tmp_arr[0];
    int32_t min_t = tmp_arr[0];

    for (int i = 0; i < tmp_arr.size() - 1; i++)
    {
        if (tmp_arr[i] > max_t)
            max_t = tmp_arr[i];
        if (tmp_arr[i] < min_t)
            min_t = tmp_arr[i];
    }

    max = max_t;
    min = min_t;
}

bool interpolation_kkyu::ramp_pattern(double sample_num, double arr_size, std::vector<double> &xi_arr)
{
    double start_p = 0;
    double end_p = arr_size - 1;
    double delta = 1 / (sample_num + 1);
    double inf = std::numeric_limits<double>::infinity();

    if (delta == 0)
    {
        if (start_p == end_p)
        {
            xi_arr.push_back(start_p);
        }
        else
        {
            return -1; // error
        }
    }
    else
    {
        if ((abs((end_p - start_p) / delta) == inf) || (((end_p - start_p) / delta) < 0))
        {
            return -1; // error
        }
        else
        {
            double q = abs((abs(end_p) + abs(start_p)) / delta) * eps;
            double w = abs(ceil(((end_p - start_p) / delta) + 1) - (((end_p - start_p) / delta) + 1));

            int32_t e = (w <= q) ? ceil(((end_p - start_p) / delta) + 1) : floor(((end_p - start_p) / delta) + 1);

            for (int i = 0; i <= e - 1; i++)
            {
                double r = start_p + (delta * i);
                xi_arr.push_back(r);
            }
        }
    }
    return 0;
}

void interpolation_kkyu::chk_monotonic(std::vector<double> y, std::vector<double> &x_out, std::vector<double> &y_out)
{
    for (int i = 0; i <= y.size() - 1; i++)
    {
        double q = (double)i;
        x_out.push_back(q);
    }

    y_out = y;
}

int32_t interpolation_kkyu::interpol_prepare(std::vector<double> y, std::vector<double> xi_arr, std::vector<double> &x_out, std::vector<double> &y_out, std::vector<double> &xi_out)
{
    std::vector<double> x_chk_out, y_chk_out;

    chk_monotonic(y, x_chk_out, y_chk_out);

    int32_t ntimes = 1;
    int32_t ntimes_rmin;

    x_out = x_chk_out;
    y_out = y_chk_out;

    if (xi_arr.size() == 0)
    {
        if (x_chk_out.size() > 1)
        {
            ntimes_rmin = ntimes;

            if (ntimes >= 63)
            {
                ntimes = 63;
            }
            else if (ntimes <= 0)
            {
                ntimes = 0;
            }
            else
            {
                ntimes = ntimes;
            }
        }
        else
        {
            ntimes_rmin = 0;
        }
        xi_out = x_chk_out;
    }
    else
    {
        xi_out = xi_arr;
        ntimes_rmin = 0;
    }

    return ntimes_rmin;
}

double interpolation_kkyu::one_side_slope(std::vector<double> delta, std::vector<double> h)
{
    double one_slope;

    double q = ((((h[0] * 2) + h[1]) * delta[0]) - (h[0] * delta[1])) / (h[0] + h[1]);

    if (sign_decision(q) != sign_decision(delta[0]))
    {
        one_slope = 0;
    }
    else
    {
        if ((abs(q) > abs(3 * delta[0])) && (sign_decision(delta[0]) != sign_decision(delta[1])))
        {
            one_slope = 3 * delta[0];
        }
        else
        {
            one_slope = q;
        }
    }
    return one_slope;
}

double interpolation_kkyu::interior_slope(std::vector<double> delta, std::vector<double> h)
{
    double in_slope;
    int q, w;

    if (delta[0] >= eps1 || delta[0] <= (-1 * eps1))
    {
        q = 0;
    }
    else
    {
        q = 1;
    }

    if (delta[1] >= eps1 || delta[1] <= (-1 * eps1))
    {
        w = 0;
    }
    else
    {
        w = 1;
    }

    if ((q == 1 || w == 1) || (sign_decision(delta[1]) != sign_decision(delta[0])))
    {
        in_slope = 0;
    }
    else
    {
        if (h[0] == h[1])
        {
            in_slope = 2 / (1 / delta[0] + 1 / delta[1]);
        }
        else
        {
            in_slope = ((2 * h[1] + h[0]) + (2 * h[0] + h[1])) / ((2 * h[1] + h[0] / delta[0]) + (2 * h[0] + h[1] / delta[1]));
        }
    }
    return in_slope;
}

void interpolation_kkyu::y_derivative(std::vector<double> y, std::vector<double> x, std::vector<double> &y_derivate)
{
    std::vector<double> delta, h;
    std::vector<int32_t> tmp_arr;
    double q, w;
    int32_t e = 0;
    double slope;

    for (int i = 0; i <= y.size() - 2; i++)
    {
        q = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
        w = x[i + 1] - x[i];

        delta.push_back(q);
        h.push_back(w);
    }
    delta.pop_back();
    h.pop_back();

    tmp_arr.push_back(0);
    tmp_arr.push_back(y.size() - 1);
    e = y.size();

    std::vector<double> delta_tmp_0, h_tmp_0, delta_tmp_1, h_tmp_1, delta_tmp_2, h_tmp_2;

    for (int j = 0; j < e; j++)
    {
        if (find_arr_index(tmp_arr, j) == 0)
        {
            delta_tmp_0.push_back(delta[0]);
            delta_tmp_0.push_back(delta[1]);

            h_tmp_0.push_back(h[0]);
            h_tmp_0.push_back(h[1]);

            slope = one_side_slope(delta_tmp_0, h_tmp_0);
            delta_tmp_0.clear();
            h_tmp_0.clear();
        }
        else if (find_arr_index(tmp_arr, j) == 1)
        {

            delta_tmp_1.push_back(delta[j - 1]);
            delta_tmp_1.push_back(delta[j - 2]);

            h_tmp_1.push_back(h[j - 1]);
            h_tmp_1.push_back(h[j - 2]);

            slope = one_side_slope(delta_tmp_1, h_tmp_1);
            delta_tmp_1.clear();
            h_tmp_1.clear();
        }
        else
        {
            delta_tmp_2.push_back(delta[j - 1]);
            delta_tmp_2.push_back(delta[j]);

            h_tmp_2.push_back(h[j - 1]);
            h_tmp_2.push_back(h[j]);

            slope = interior_slope(delta_tmp_2, h_tmp_2);
            delta_tmp_2.clear();
            h_tmp_2.clear();
        }
        y_derivate.push_back(slope);
    }
}

void interpolation_kkyu::hermite_polynomial(std::vector<double> y, std::vector<double> y_deriv, std::vector<double> x, int32_t index, std::vector<double> &hermite)
{
    double p_0, p_1, p_2, p_3;
    double q;

    q = (y_deriv[index + 1] - y_deriv[index]) - ((((y[index + 1] - y[index]) * (1 / (x[index + 1] - x[index]))) - y_deriv[index]) + (((y[index + 1] - y[index]) * (1 / (x[index + 1] - x[index]))) - y_deriv[index]));

    p_0 = y[index];
    hermite.push_back(p_0);

    p_1 = y_deriv[index];
    hermite.push_back(p_1);

    p_2 = ((((y[index + 1] - y[index]) * (1 / (x[index + 1] - x[index]))) - y_deriv[index]) - q) * (1 / (x[index + 1] - x[index]));
    hermite.push_back(p_2);

    p_3 = q * pow((1 / (x[index + 1] - x[index])), 2);
    hermite.push_back(p_3);
}

double interpolation_kkyu::polynomial_calc(std::vector<double> Px, double a)
{
    double Pa;
    std::vector<double> Px_out;

    if (Px.size() == 0)
    {
        Pa = 0;
    }
    else
    {
        double px_var;
        double ref_var = 0;

        if (Px.size() == 0)
        {
            Px_out = {};
        }
        else
        {
            int32_t shift_n = -1;
            int32_t out_n;

            for (int i = 0; i <= Px.size() - 1; i++)
            {
                px_var = abs(Px[i]);

                if (px_var > ref_var)
                {
                    out_n = i;
                }
                else
                {
                    out_n = shift_n;
                }
                shift_n = out_n;
            }

            int32_t max_n;

            if ((out_n + 1) > 1)
            {
                max_n = out_n + 1;
            }
            else if (((out_n + 1) == 1))
            {
                max_n = 1;
            }
            else
            {
                max_n = 1;
            }

            for (int j = 0; j <= max_n - 1; j++)
            {
                Px_out.push_back(Px[j]);
            }
            std::reverse(Px_out.begin(), Px_out.end());

            double shift_d = 0.0;
            for (int n = 0; n <= Px_out.size() - 1; n++)
            {
                Pa = a * shift_d + Px_out[n];
                shift_d = Pa;
            }
        }
    }
    return Pa;
}

void interpolation_kkyu::seq_0(double xi_each_var, std::vector<double> x_out, int32_t x_out_size1, bool ascending_bool, int32_t start_index_new, int32_t &sq0_out0, int32_t &sq0_out1, bool &sq0_out2)
{
    if ((xi_each_var >= x_out[start_index_new]) == ascending_bool)
    {
        if (x_out_size1 == start_index_new)
        {
            sq0_out0 = x_out_size1;
            sq0_out1 = x_out_size1;
            sq0_out2 = false;
        }
        else
        {
            int32_t n0 = start_index_new;
            int32_t n1 = start_index_new + 1;
            int32_t n2 = 1;
            bool b0 = false;

            while (((xi_each_var >= x_out[n1]) == ascending_bool) & ~b0)
            {
                b0 = ((n1 + n2 + n2) > x_out_size1) ? true : false;
                n0 = n1;
                n1 = n1 + n2 + n2;
                n2 = n2 + n2;
            }

            if ((n1 + n2 + n2) > x_out_size1)
            {
                sq0_out1 = x_out_size1 + 1;
            }
            else
            {
                sq0_out1 = n1;
            }
            sq0_out0 = n0;
            sq0_out2 = true;
        }
    }
    else
    {
        if (start_index_new == 0)
        {
            sq0_out0 = -1;
            sq0_out1 = -1;
            sq0_out2 = false;
        }
        else
        {
            int32_t m0 = start_index_new - 1;
            int32_t m1 = start_index_new;
            int32_t m2 = 1;
            bool v0 = false;

            while (((xi_each_var < x_out[m0]) == ascending_bool) & ~v0)
            {
                int32_t m0_tmp = m0;

                v0 = ((m1 - m2 + m2) < 0) ? true : false;
                m0 = m1 - m2 + m2;
                m1 = m0_tmp;
                m2 = m2 + m2;
            }

            if ((m1 - m2 + m2) < 0)
            {
                sq0_out0 = 0;
            }
            else
            {
                sq0_out0 = m1;
            }
            sq0_out1 = m1;
            sq0_out2 = true;
        }
    }
}

void interpolation_kkyu::seq_1(double xi_each_var, std::vector<double> x_out, bool ascending_bool, int32_t sq0_out0, int32_t sq0_out1, bool sq0_out2, int32_t &sq1_out0)
{
    if (sq0_out2 == true)
    {
        int32_t n0 = sq0_out0;
        int32_t n1 = sq0_out1;

        while ((n1 - n0) > 1)
        {
            if ((xi_each_var >= x_out[(n1 + n0) * pow(2, -1)]) == ascending_bool)
            {
                n1 = n1;
                n0 = (n1 + n0) * pow(2, -1);
            }
            else
            {
                n0 = n0;
                n1 = (n1 + n0) * pow(2, -1);
            }
        }
        sq1_out0 = n0;
    }
    else
    {
        sq1_out0 = sq0_out0;
    }
}

double interpolation_kkyu::seq_2(double xi_each_var, std::vector<double> x_out, int32_t sq1_out0)
{
    double fractional_index;

    fractional_index = ((xi_each_var - x_out[sq1_out0]) / (x_out[sq1_out0 + 1] - x_out[sq1_out0])) + (double)sq1_out0;

    return fractional_index;
}

void interpolation_kkyu::hermite_method(std::vector<double> x_out, std::vector<double> y_out, std::vector<double> xi_out, std::vector<double> y_derivate, std::vector<double> &yi)
{
    // prepare for loop
    std::vector<double> tmp_arr = {};
    double a_pre = 0;
    int32_t pre_index = -1;
    int32_t x_out_size2 = x_out.size() - 2; //-2
    int32_t start_index = 0;

    int x_out_sz = x_out.size();
    int y_out_sz = y_out.size();
    int xi_out_sz = xi_out.size();
    int y_derivate_sz = y_derivate.size();

    // for loop
    for (int i = 0; i <= xi_out.size() - 1; i++)
    {
        double xi_each_var = xi_out[i];

        int32_t x_out_size1 = x_out.size() - 1; //-1
        bool ascending_bool = (x_out[0] <= x_out[x_out_size1]) ? true : false;
        bool nearest_bool;
        int32_t start_index_new;

        // starting index!!!
        if (start_index >= x_out_size1)
        {
            start_index_new = start_index;
        }
        else if (start_index <= 0)
        {
            start_index_new = 0;
        }
        else
        {
            start_index_new = start_index;
        }
        // starting index!!!

        // ascending bool!!!
        if (ascending_bool == true)
        {
            if ((xi_each_var > x_out[0]) && (xi_each_var < x_out[x_out_size1]))
            {
                nearest_bool = true;
            }
            else
            {
                nearest_bool = false;
            }
        }
        else
        {
            if ((xi_each_var < x_out[0]) && (xi_each_var > x_out[x_out_size1]))
            {
                nearest_bool = true;
            }
            else
            {
                nearest_bool = false;
            }
        }
        // ascending bool!!!

        // fractional_index!!!
        double fractional_index;

        if (nearest_bool == true)
        {
            int32_t sq0_out0;
            int32_t sq0_out1;
            bool sq0_out2;

            int32_t sq1_out0;

            seq_0(xi_each_var, x_out, x_out_size1, ascending_bool, start_index_new, sq0_out0, sq0_out1, sq0_out2);
            seq_1(xi_each_var, x_out, ascending_bool, sq0_out0, sq0_out1, sq0_out2, sq1_out0);
            fractional_index = seq_2(xi_each_var, x_out, sq1_out0);
        }
        else
        {
            if (ascending_bool == true)
            {
                fractional_index = (xi_each_var <= x_out[0]) ? 0 : start_index_new;
            }
            else
            {
                fractional_index = (xi_each_var >= x_out[0]) ? 0 : start_index_new;
            }
            fractional_index = (double)fractional_index;
        }
        // fractional_index!!!

        // min index!!!
        int32_t min_index;
        if (x_out_size2 < (int32_t)floor(fractional_index))
        {
            min_index = x_out_size2;
        }
        else if (x_out_size2 == (int32_t)floor(fractional_index))
        {
            min_index = x_out_size2;
        }
        else
        {
            min_index = (int32_t)floor(fractional_index);
        }
        // min index!!!

        // polynomial out!!!
        std::vector<double> tmp_arr_out;
        double a_pre_out;
        int32_t pre_index_out;

        if (pre_index == min_index)
        {
            tmp_arr_out = tmp_arr;
            a_pre_out = a_pre;
            pre_index_out = pre_index;
        }
        else
        {
            hermite_polynomial(y_out, y_derivate, x_out, min_index, tmp_arr_out);
            tmp_arr_out = tmp_arr_out;
            a_pre_out = x_out[min_index];
            pre_index_out = min_index;
        }
        // polynomial out!!!

        yi.push_back(polynomial_calc(tmp_arr_out, (xi_each_var - a_pre_out)));

        tmp_arr = tmp_arr_out;
        a_pre = a_pre_out;
        pre_index = pre_index_out;
        start_index = min_index;
    }
}

void interpolation_kkyu::hermite_interpolation(std::vector<double> origin_vector, int32_t sample_num, std::vector<double> &interpolation_vector)
{
    std::vector<double> xi;
    ramp_pattern(sample_num, origin_vector.size(), xi);

    std::vector<double> x_out, y_out, xi_out;
    interpol_prepare(origin_vector, xi, x_out, y_out, xi_out);

    std::vector<double> y_derivate;
    y_derivative(y_out, x_out, y_derivate);

    hermite_method(x_out, y_out, xi_out, y_derivate, interpolation_vector);
}


void interpolation_kkyu::path_hermite_inter(std::vector<waypoints> origin_vec, int sample_num, std::vector<waypoints> &hermite_vec)
{
    std::vector<double> origin_x_vec;
    std::vector<double> origin_y_vec;

    std::vector<double> hermite_x_vec;
    std::vector<double> hermite_y_vec;

    for (int i = 0; i < origin_vec.size(); i++)
    {
        double x, y;

        x = origin_vec[i].x_point;
        y = origin_vec[i].y_point;

        origin_x_vec.push_back(x);
        origin_y_vec.push_back(y);
    }

    hermite_interpolation(origin_x_vec, sample_num, hermite_x_vec);
    hermite_interpolation(origin_y_vec, sample_num, hermite_y_vec);
    for (int i = 0; i < hermite_x_vec.size(); i++)
    {
        waypoints hermite_pt;

        hermite_pt.x_point = hermite_x_vec[i];
        hermite_pt.y_point = hermite_y_vec[i];

        hermite_vec.push_back(hermite_pt);
    }
}

void interpolation_kkyu::linear_interpolation(std::vector<double> origin_vector, int32_t sample_num, std::vector<double> &interpolation_vector)
{
    double lerp_xi = 0;
    std::vector<double> lerp_xi_vec;

    for (int i = 0; i <= sample_num + 1; i++)
    {
        lerp_xi_vec.push_back(lerp_xi);
        lerp_xi += 1 / (double)(sample_num + 1);
    }

    for (int i = 0; i < origin_vector.size() - 1; i++)
    {
        if (i == 0)
        {
            for (int j = 0; j < lerp_xi_vec.size(); j++)
            {
                interpolation_vector.push_back(((origin_vector[i + 1] - origin_vector[i]) * lerp_xi_vec[j]) + origin_vector[i]);
            }
        }
        else
        {
            for (int j = 1; j < lerp_xi_vec.size(); j++)
            {
                interpolation_vector.push_back(((origin_vector[i + 1] - origin_vector[i]) * lerp_xi_vec[j]) + origin_vector[i]);
            }
        }
    }
}

void interpolation_kkyu::relocate_calc(double locate_dist, int n0, std::vector<waypoints> interpol_path, double &calc_dist, int &id_n)
{
    int ni = 0;
    int i = 0;

    double x, y, k;

    bool b_tmp0 = true;
    bool b_tmp1 = true;
    bool s0 = true;

    double loc_dist = (locate_dist <= 0) ? eps1 : locate_dist;
    loc_dist = loc_dist * 0.99;
    int path_size = interpol_path.size();

    while (s0)
    {
        ni = n0 + i;
        x = interpol_path[n0].x_point - interpol_path[ni].x_point;
        y = interpol_path[n0].y_point - interpol_path[ni].y_point;

        k = sqrt(pow(x, 2) + pow(y, 2));

        b_tmp0 = (k >= loc_dist) ? true : false;
        b_tmp1 = ((path_size - 1) == ni) ? true : false;

        s0 = !(b_tmp0 || b_tmp1);

        i++;
    }
    calc_dist = k;
    id_n = ni;
}

void interpolation_kkyu::re_locate(double locate_dist, std::vector<waypoints> interpol_path, std::vector<waypoints> &relocate_path)
{
    double calc_dist;
    int id_n;
    int n0 = 0;
    int i = 0;

    bool s0 = true;
    int path_size = interpol_path.size();

    waypoints wp;

    wp.x_point = interpol_path[0].x_point;
    wp.y_point = interpol_path[0].y_point;
    wp.index = 0;
    relocate_path.push_back(wp);

    while (s0)
    {
        i++;
        relocate_calc(locate_dist, n0, interpol_path, calc_dist, id_n);
        s0 = ((path_size - 1) == id_n) ? false : true;
        n0 = id_n;

        wp.index = i;
        wp.x_point = interpol_path[n0].x_point;
        wp.y_point = interpol_path[n0].y_point;

        relocate_path.push_back(wp);
    }
}
