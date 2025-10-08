#pragma once
#include <Eigen/Dense>
#include <vector>


namespace vamp::planning {
    using row_matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign | Eigen::RowMajor>;
    using state = Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::AutoAlign | Eigen::RowMajor>;

    inline static int comb(int n, int k) {
        int n_fact = 1;
        int k_fact = 1;
        int n_k_fact = 1;

        for (int i = 1; i <= n; i++) {
            if (i <= k) {
                k_fact *= i;
            }

            if (i <= (n - k)) {
                n_k_fact *= i;
            }
            n_fact *= i;
        }
        return n_fact / (k_fact * n_k_fact);
    }

    class Bezier {
        public:
            row_matrix anchors;
            int degree;

            Bezier(row_matrix anchors) {
                this->anchors = anchors;
                this->degree = anchors.rows() - 1;
            }

            std::vector<state> generate_trajectory(int T) {
                std::vector<state> traj;
                for (int t = 0; t <= T; t++) {
                    state P(1, this->anchors.rows());
                    for (int i = 0; i <= this->degree; i++) {
                        P(0, i) = ((comb(this->degree, i) * 
                        (pow(1.0 - t * 1.0 / T, this->degree - i)) * 
                        (pow(1.0 * t / T, i))));
                    }
                    state s = P * this->anchors;
                    traj.push_back(s);
                }
                return traj;
            }
            
            state evaluate(int t) {
                state P(1, this->anchors.rows());
                for (int i = 0; i <= this->degree; i++) {
                    P(0, i) = ((comb(this->degree, i) * 
                    (pow(1.0 - t * 1.0, this->degree - i)) * 
                    (pow(1.0 * t, i))));
                }
                state s = P * this->anchors;
                return s;
            }
    };
}