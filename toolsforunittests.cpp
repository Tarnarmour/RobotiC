#include "Eigen/Dense"
#include <iostream>


template <typename T, int m, int n>
bool compare_eigen_matrices(const Eigen::Matrix<T, m, n>& m1, const Eigen::Matrix<T, m, n>& m2, double prec = 1e-10, bool verb = true)
{
    bool equal = true;

    for (int i = 0; i < m * n; i++)
    {
        if ((std::abs(m1.reshaped()(i) - m2.reshaped()(i)) > prec))
        {
            if (verb)
            {
                std::cout << "At index " << i << " m1 has value: " << m1(i) << " but m2 has value: " << m2(i) << std::endl;
            }
            equal = false;
        }

    }

    return equal;
}

