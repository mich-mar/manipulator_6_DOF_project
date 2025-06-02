#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>

/**
 * @struct DHParams
 * @brief Struktura przechowująca parametry Denavita-Hartenberga
 */
struct DHParams {
    double theta;   ///< Kąt obrotu wokół osi z [rad]
    double d;       ///< Przesunięcie wzdłuż osi z [m]
    double a;       ///< Długość członu wzdłuż osi x [m]
    double alpha;   ///< Kąt skręcenia wokół osi x [rad]

    /**
     * @brief Konstruktor parametrów D-H
     * @param t Kąt theta [rad]
     * @param d_val Przesunięcie d [m]
     * @param a_val Długość a [m]
     * @param alpha_val Kąt alpha [rad]
     */
    DHParams(double t, double d_val, double a_val, double alpha_val)
            : theta(t), d(d_val), a(a_val), alpha(alpha_val) {}
};


/**
 * @brief Oblicza macierz transformacji jednorodnej na podstawie parametrów D-H
 * @param theta Kąt obrotu wokół osi z [rad]
 * @param d Przesunięcie wzdłuż osi z [m]
 * @param a Długość członu [m]
 * @param alpha Kąt skręcenia [rad]
 * @return Macierz transformacji jednorodnej 4x4
 */
Eigen::Matrix4d dh_transform(double theta, double d, double a, double alpha) {
    Eigen::Matrix4d T;
    double ct = cos(theta), st = sin(theta);
    double ca = cos(alpha), sa = sin(alpha);

    T << ct, -st*ca,  st*sa, a*ct,
            st,  ct*ca, -ct*sa, a*st,
            0,      sa,     ca,    d,
            0,       0,      0,    1;
    return T;
}

/**
 * @brief Implementacja kinematyki prostej dla robota 6-DOF
 * @param q Wektor konfiguracji przegubów [rad]
 * @param params Wektor parametrów długości członów [m]
 * @return Macierz transformacji końcówki robota
 *
 * @details Parametry długości:
 * - l1: Pierwsza długość (podstawa do pierwszego przegubu)
 * - l2: Druga długość (pierwszy do drugiego przegubu)
 * - l3: Trzecia długość
 * - l4: Czwarta długość
 * - l5: Piąta długość (do końcówki)
 */
Eigen::Matrix4d forward_kinematics(const Eigen::VectorXd &q, const std::vector<double> &params) {
    // Pobranie parametrów długości
    double l1 = params[0], l2 = params[1], l3 = params[2], l4 = params[3], l5 = params[4];

    // Definicja tablicy parametrów D-H dla każdego połączenia
    std::vector<DHParams> dh_table = {
            DHParams(q[0], l1, 0, M_PI/2),        // A0-1: θ=q1, d=l1, a=0, α=90°
            DHParams(q[1] + M_PI/2, 0, l2, 0),    // A1-2: θ=q2+90°, d=0, a=l2, α=0°
            DHParams(q[2], 0, 0, M_PI/2),         // A2-3: θ=q3, d=0, a=0, α=90°
            DHParams(q[3], l3, 0, M_PI/2),        // A3-4: θ=q4, d=l3, a=0, α=90°
            DHParams(q[4], l4, 0, -M_PI/2),       // A4-5: θ=q5, d=l4, a=0, α=-90°
            DHParams(q[5], l5, 0, 0)              // A5-6: θ=q6, d=l5, a=0, α=0°
    };

    Eigen::Matrix4d T_final = Eigen::Matrix4d::Identity();
    for (const auto &dh : dh_table) {
        T_final *= dh_transform(dh.theta, dh.d, dh.a, dh.alpha);
    }
    return T_final;
}

/**
 * @brief Oblicza macierz Jacobiego dla aktualnej konfiguracji robota
 * @param q Wektor konfiguracji przegubów [rad]
 * @param params Wektor parametrów długości członów [m]
 * @return Macierz Jacobiego 6x6
 */
Eigen::MatrixXd compute_jacobian(const Eigen::VectorXd &q, const std::vector<double> &params) {
    int dof = 6;
    Eigen::MatrixXd J(6, dof);
    std::vector<Eigen::Matrix4d> Ts;

    double l1 = params[0], l2 = params[1], l3 = params[2], l4 = params[3], l5 = params[4];

    // Spójne parametry D-H z forward_kinematics
    std::vector<DHParams> dh_table = {
            DHParams(q[0], l1, 0, M_PI/2),        // A0-1
            DHParams(q[1] + M_PI/2, 0, l2, 0),    // A1-2
            DHParams(q[2], 0, 0, M_PI/2),         // A2-3
            DHParams(q[3], l3, 0, M_PI/2),        // A3-4
            DHParams(q[4], l4, 0, -M_PI/2),       // A4-5
            DHParams(q[5], l5, 0, 0)              // A5-6
    };

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Ts.push_back(T);

    // Oblicz transformacje kumulatywne
    for (int i = 0; i < dof; ++i) {
        const auto &dh = dh_table[i];
        Eigen::Matrix4d Ti = dh_transform(dh.theta, dh.d, dh.a, dh.alpha);
        T *= Ti;
        Ts.push_back(T);
    }

    Eigen::Vector3d pe = T.block<3,1>(0,3);

    // Oblicz kolumny Jakobiego
    for (int i = 0; i < dof; ++i) {
        Eigen::Vector3d zi = Ts[i].block<3,1>(0,2);
        Eigen::Vector3d pi = Ts[i].block<3,1>(0,3);

        // Część liniowa i kątowa Jakobiego
        Eigen::Vector3d Jv = zi.cross(pe - pi);
        Eigen::Vector3d Jw = zi;

        J.block<3,1>(0,i) = Jv;
        J.block<3,1>(3,i) = Jw;
    }

    return J;
}


/**
 * @brief Implementacja kinematyki odwrotnej z adaptacyjnym sterowaniem
 * @param T_target Docelowa macierz transformacji
 * @param q Początkowa (i końcowa) konfiguracja przegubów
 * @param params Parametry długości członów
 * @param max_iter Maksymalna liczba iteracji
 * @param tol Tolerancja błędu
 * @return true jeśli znaleziono rozwiązanie, false w przeciwnym razie
 */
bool inverse_kinematics(
        const Eigen::Matrix4d &T_target,
        Eigen::VectorXd &q,
        const std::vector<double> &params,
        int max_iter = 300,
        double tol = 1e-5
) {
    double alpha = 1.0;  // Początkowa wartość kroku
    double damping = 1e-4;  // Zwiększone tłumienie
    double prev_error = std::numeric_limits<double>::max();
    int stagnation_count = 0;

    // Wagi dla błędu pozycji vs rotacji
    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(6, 6);
    W.block<3,3>(0,0) *= 1.0;  // Waga pozycji
    W.block<3,3>(3,3) *= 0.5;  // Mniejsza waga rotacji

    for (int iter = 0; iter < max_iter; ++iter) {
        Eigen::Matrix4d T = forward_kinematics(q, params);

        // Błąd pozycji
        Eigen::Vector3d dp = T_target.block<3,1>(0,3) - T.block<3,1>(0,3);

        // Błąd rotacji - stabilniejsza metoda
        Eigen::Matrix3d R_current = T.block<3,3>(0,0);
        Eigen::Matrix3d R_target = T_target.block<3,3>(0,0);
        Eigen::Matrix3d R_err = R_target * R_current.transpose();

        // Konwersja na wektor kątowy (bardziej stabilna)
        double trace = R_err.trace();
        double angle = acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0)));

        Eigen::Vector3d dr = Eigen::Vector3d::Zero();
        if (angle > 1e-6) {
            Eigen::Vector3d axis;
            axis << R_err(2,1) - R_err(1,2),
                    R_err(0,2) - R_err(2,0),
                    R_err(1,0) - R_err(0,1);
            dr = (angle / (2.0 * sin(angle))) * axis;
        }

        // Błąd końcowy z wagami
        Eigen::VectorXd error(6);
        error << dp, dr;
        Eigen::VectorXd weighted_error = W * error;

        double current_error = weighted_error.norm();

        // Wypisuj co 10 iteracji żeby nie zaśmiecać
        if (iter % 10 == 0 || iter < 20) {
            std::cout << "Iter " << iter << ", błąd: " << current_error
                      << ", pos: " << dp.norm() << ", rot: " << dr.norm() << std::endl;
        }

        if (current_error < tol) {
            std::cout << "Zbieżność osiągnięta w " << iter << " iteracjach\n";
            return true;
        }

        // Detekcja stagnacji
        if (abs(current_error - prev_error) < 1e-8) {
            stagnation_count++;
        } else {
            stagnation_count = 0;
        }

        // Jeśli utknęliśmy, spróbuj z innym punktem startowym
        if (stagnation_count > 20) {
            std::cout << "Wykryto stagnację, restartowanie z losowego punktu...\n";
            for (int i = 0; i < 6; ++i) {
                q[i] = ((double)rand() / RAND_MAX - 0.5) * M_PI;
            }
            stagnation_count = 0;
            alpha = 1.0;
            continue;
        }

        Eigen::MatrixXd J = compute_jacobian(q, params);

        // Sprawdź singularność
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
        double min_sv = svd.singularValues().minCoeff();

        if (min_sv < 1e-6) {
            std::cout << "Singularność wykryta (min SV: " << min_sv << "), zwiększam tłumienie\n";
            damping = std::max(damping * 2.0, 1e-2);
        }

        // Damped least squares z wagami
        Eigen::MatrixXd JW = J * W.inverse();
        Eigen::MatrixXd JtJ = JW.transpose() * JW;
        Eigen::MatrixXd JtJ_damped = JtJ + damping * Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd J_inv = JtJ_damped.inverse() * JW.transpose();

        // Aktualizuj konfigurację z adaptacyjnym krokiem
        Eigen::VectorXd dq = alpha * J_inv * weighted_error;

        // Ograniczenia na wielkość kroku
        double max_dq = dq.cwiseAbs().maxCoeff();
        if (max_dq > 0.5) {
            dq *= 0.5 / max_dq;
        }

        q += dq;

        // Ograniczenia na kąty przegubów
        for (int i = 0; i < 6; ++i) {
            while (q[i] > M_PI) q[i] -= 2*M_PI;
            while (q[i] < -M_PI) q[i] += 2*M_PI;
        }

        // Adaptacyjna kontrola kroku
        if (current_error < prev_error) {
            alpha = std::min(alpha * 1.1, 1.5);  // Zwiększ krok
            damping *= 0.95;  // Zmniejsz tłumienie
        } else {
            alpha *= 0.5;  // Zmniejsz krok
            damping *= 1.1;  // Zwiększ tłumienie
        }

        prev_error = current_error;
    }

    return false;
}

/**
 * @brief Sprawdza czy zadana pozycja jest osiągalna dla robota
 * @param target_pos Wektor pozycji docelowej
 * @param params Wektor parametrów długości członów
 * @return true jeśli pozycja jest osiągalna, false w przeciwnym razie
 */
bool check_reachability(const Eigen::Vector3d &target_pos, const std::vector<double> &params) {
    // Użyj wszystkich 5 parametrów
    double l1 = params[0], l2 = params[1], l3 = params[2], l4 = params[3], l5 = params[4];
    double max_reach = l1 + l2 + l3 + l4 + l5; // Maksymalny zasięg (uwzględniamy wszystkie długości)
    double min_reach = abs(l1 - l2 - l3 - l4 - l5); // Minimalny zasięg

    double distance = target_pos.norm();
    return (distance >= min_reach && distance <= max_reach);
}


/**
 * @brief Główna funkcja programu
 *
 * Demonstruje działanie kinematyki odwrotnej dla robota 6-DOF.
 * Próbuje osiągnąć zadaną pozycję i orientację końcówki z różnych
 * konfiguracji początkowych.
 */
int main() {
    srand(time(nullptr));

    // Parametry konstrukcyjne robota [m]
    std::vector<double> params = {0.016, 0.067, 0.050, 0.015, 0.035}; // l1, l2, l3, l4, l5

    // Zadana transformacja końcowa
    Eigen::Matrix4d T_goal = Eigen::Matrix4d::Identity();
    T_goal(0,3) = 0.0;
    T_goal(1,3) = 0.005;
    T_goal(2,3) = 0.166;

    // Prostsze zadanie rotacyjne
    T_goal.block<3,3>(0,0) = Eigen::AngleAxisd(M_PI/8, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    std::cout << "Cel:\n" << T_goal << std::endl;

    // Sprawdź osiągalność
    Eigen::Vector3d target_pos = T_goal.block<3,1>(0,3);
    if (!check_reachability(target_pos, params)) {
        std::cout << "UWAGA: Pozycja może być poza zasięgiem robota!\n";
        std::cout << "Odległość: " << target_pos.norm() << std::endl;
        double max_reach = params[0] + params[1] + params[2] + params[3] + params[4]; // Uwzględnij wszystkie długości
        std::cout << "Maksymalny zasięg: " << max_reach << std::endl;
    }

    // Spróbuj z kilku różnych punktów startowych
    std::vector<Eigen::VectorXd> initial_configs;

    // Konfiguracja zerowa
    Eigen::VectorXd q1(6); q1.setZero();
    initial_configs.push_back(q1);

    // Kilka losowych konfiguracji
    for (int i = 0; i < 3; ++i) {
        Eigen::VectorXd q_rand(6);
        for (int j = 0; j < 6; ++j) {
            q_rand[j] = ((double)rand() / RAND_MAX - 0.5) * M_PI;
        }
        initial_configs.push_back(q_rand);
    }

    // Próbuj z każdą konfiguracją startową
    for (int attempt = 0; attempt < initial_configs.size(); ++attempt) {
        std::cout << "\n=== Próba " << attempt + 1 << " ===\n";
        Eigen::VectorXd q_init = initial_configs[attempt];

        if (inverse_kinematics(T_goal, q_init, params)) {
            std::cout << "SUKCES! Rozwiązanie IK [rad]:\n" << q_init.transpose() << std::endl;
            std::cout << "Rozwiązanie IK [deg]:\n" << (q_init * 180.0 / M_PI).transpose() << std::endl;

            // Szczegółowa weryfikacja
            Eigen::Matrix4d T_result = forward_kinematics(q_init, params);
            std::cout << "\nWeryfikacja - osiągnięta transformacja:\n" << T_result << std::endl;

            Eigen::Vector3d pos_error = T_goal.block<3,1>(0,3) - T_result.block<3,1>(0,3);
            std::cout << "Błąd pozycji: " << pos_error.norm() << std::endl;

            Eigen::Matrix3d rot_error = T_goal.block<3,3>(0,0).transpose() * T_result.block<3,3>(0,0);
            Eigen::AngleAxisd aa_error(rot_error);
            std::cout << "Błąd rotacji [rad]: " << aa_error.angle() << std::endl;
            std::cout << "Błąd rotacji [deg]: " << aa_error.angle() * 180.0 / M_PI << std::endl;

            return 0; // Sukces!

        } else {
            std::cout << "Próba " << attempt + 1 << " nie powiodła się.\n";
        }
    }

    std::cout << "\nWszystkie próby nie powiodły się :((
    return 0;
}