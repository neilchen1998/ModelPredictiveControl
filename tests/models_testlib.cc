#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <models/mass_spring_damper.hpp>

namespace models
{
    template <typename T>
    inline unsigned int GetTimeStamp(const T& adaptor)
    {
        struct Printer : T // to access protected Adaptor::t (protected inheritance)
        {
            unsigned int GetTimeStamp() const { return this->t; }
        };

        return static_cast<const Printer&>(adaptor).GetTimeStamp();
    }

    template <typename T>
    inline Eigen::MatrixXd GetMatrixAd(const T& adaptor)
    {
        struct Printer : T // to access protected Adaptor::t (protected inheritance)
        {
            Eigen::MatrixXd GetMatrixAd() const { return this->Ad; }
        };

        return static_cast<const Printer&>(adaptor).GetMatrixAd();
    }
}

TEST_CASE( "MassSpringDamper 1", "[main]" ) 
{
    constexpr int n = 4;
    constexpr int m = 1;
    constexpr float k1 = 1000;
    constexpr float k2 = 2000;
    constexpr float d1 = 1;
    constexpr float d2 = 5;
    constexpr float m1 = 20;
    constexpr float m2 = 20;
    constexpr float T = 0.05;

    Eigen::MatrixXd A(n, n);
    A << 0, 1, 0, 0, 
        -(k1 + k2) / m1, -(d1 + d2) / m1, k2/m1, d2/m1, 
        0, 0, 0, 1, 
        k2/m2, d2/m2, -k2/m2, -d2/m2;
    Eigen::MatrixXd B(n, m);
    B << 0, 0, 
        0, 1/m2;
    Eigen::MatrixXd C(1, n);
    C << 1, 0, 0, 0;

    auto msd = models::MassSpringDamper(A, B, C, Eigen::MatrixXd::Zero(m, m), Eigen::MatrixXd::Zero(n, 1), T);

    SECTION("Convert Matrix A to Matrix Ad", "[constructor]")
    {
        Eigen::Matrix4d A_true;
        A_true << 8.36383509e-01,  4.55860203e-02,  1.06633966e-01,  2.93243405e-03,
            -6.54465964,  8.23440811e-01,  4.26535862,  1.17297362e-01,
            1.06434027e-01,  2.93243405e-03,  8.89900431e-01,  4.69722618e-02,
            4.25736108,  1.17297362e-01, -4.40398278e+00,  8.78890474e-01;

        const auto Ad = models::GetMatrixAd(msd);

        REQUIRE( Ad.isApprox(A_true, 0.05) );
    }

    SECTION("Time Stamp", "[functionality]")
    {
        constexpr int T = 10;
        for (int i = 0; i < T; i++)
        {
            msd.Step();
        }
        
        auto t = models::GetTimeStamp(msd);
        REQUIRE (t == T);
    }
}