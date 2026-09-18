#pragma once
#ifndef __Sim3_H__
#define __Sim3_H__

#include "util/NumType.h"
#include <g2o/core/base_vertex.h>
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/sim3/sim3.h"
#include "util/globalFuncs.h"

namespace HSLAM
{
    class VertexXYZPt : public g2o::BaseVertex<3, Vec3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        VertexXYZPt(){}
        virtual bool read(std::istream &is) override
        {
            return readVector(is, _estimate);
        }
        virtual bool write(std::ostream &os) const
        {
            return writeVector(os, estimate());
        }
        virtual void setToOriginImpl()
        {
            _estimate.fill(0);
        }

        virtual void oplusImpl(const number_t *update)
        {
            Eigen::Map<const Vec3> v(update);
            _estimate += v;
        }
    };

    class Sim3Vertex : public g2o::BaseVertex<7, g2o::Sim3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Sim3Vertex() : g2o::BaseVertex<7, g2o::Sim3>() {}

        void setData(double _fx, double _fy, double _cx, double _cy, bool _fixScale = false)
        {
            fx = _fx;
            fy = _fy;
            cx = _cx;
            cy = _cy;
            fixScale = _fixScale;
            _marginalized = false;
        }

        void setData2(double _fx, double _fy, double _cx , double _cy)
        {
            fx2 = _fx;
            fy2 = _fy;
            cx2 = _cx;
            cy2 = _cy;
        }

        virtual bool read(std::istream &is) override
        {
            Vec7 cam2world;
            for (int i = 0; i < 7; i++)
                is >> cam2world[i];
            is >> fx; is >> fy; is >> cx; is >> cy; is >> fixScale;
            is >> fx2; is >> fy2; is >> cx2; is >> cy2;
            setEstimate(g2o::Sim3(cam2world).inverse());
            return true;
        }

        virtual bool write(std::ostream &os) const
        {
            g2o::Sim3 cam2world(estimate().inverse()); //    estimate().inverse());
            Vec7 lv = cam2world.log();
            for (int i = 0; i < 7; i++)
                os << lv[i] << " ";
            os << fx << " " << fy << " " << cx << " " << cy << " " << fixScale << " ";
            os << fx2 << " " << fy2 << " " << cx2 << " " << cy2 << " ";
            return os.good();
        }

        virtual void setToOriginImpl() override
        {
            _estimate = g2o::Sim3();
        }

        virtual void oplusImpl(const double *update_) override
        {
            Eigen::Map<Vec7> update(const_cast<double*>(update_));
            if (fixScale)
                update[6] = 0;
            // // std::cout<<update[6]<<std::endl;
            // if(update[6] < -1e-3){
            //     _is_invalid = true;
            //     update[6] = 0.;
            // }
            _estimate = g2o::Sim3(update) * _estimate;
        }

        Vec2 cam_map(const Vec2 &v) const
        {
            return Vec2(v[0] * fx + cx, v[1] * fy + cy);
        }

        Vec2 cam_map2(const Vec2 &v) const
        {
            return Vec2(v[0] * fx2 + cx2, v[1] * fy2 + cy2);
        }

        bool fixScale = false;
        bool _is_invalid = false;
        double cx, cy, fx, fy;
        double cx2, cy2, fx2, fy2;

    };

    class EdgeSim3ProjectXYZ : public g2o::BaseBinaryEdge<2, Vec2, VertexXYZPt, Sim3Vertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeSim3ProjectXYZ(){};
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        void computeError()
        {
            const Sim3Vertex *v1 = static_cast<const Sim3Vertex *>(_vertices[1]);
            const VertexXYZPt *v2 = static_cast<const VertexXYZPt *>(_vertices[0]);

            Vec2 obs(_measurement);
            _error = obs - v1->cam_map( project( v1->estimate().map(v2->estimate())));
        }
        // virtual void linearizeOplus();
    };


    bool EdgeSim3ProjectXYZ::read(std::istream &is)
    {
        for (int i = 0; i < 2; i++)
            is >> _measurement[i];
        is >> information()(0, 0);
        is >> information()(0, 1);
        is >> information()(1, 1);
        information()(1, 0) = information()(0, 1);
        return true;
    }

    bool EdgeSim3ProjectXYZ::write(std::ostream &os) const
    {
        for (int i = 0; i < 2; i++)
        {
            os << _measurement[i] << " ";
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++)
            {
                os << " " << information()(i, j);
            }
        return os.good();
    }

    class  EdgeInverseSim3ProjectXYZ : public g2o::BaseBinaryEdge<2, Vec2,  VertexXYZPt, Sim3Vertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeInverseSim3ProjectXYZ(){};
        virtual bool read(std::istream &is) override
        {
            for (int i = 0; i < 2; i++)
                is >> _measurement[i];
            is >> information()(0, 0);
            is >> information()(0, 1);
            is >> information()(1, 1);
            information()(1, 0) = information()(0, 1);
            return true;
        }
        virtual bool write(std::ostream &os) const
        {
            for (int i = 0; i < 2; i++)
            {
                os << _measurement[i] << " ";
            }

            for (int i = 0; i < 2; i++)
                for (int j = i; j < 2; j++)
                {
                    os << " " << information()(i, j);
                }
            return os.good();
        }

        void computeError()
        {
            const Sim3Vertex *v1 = static_cast<const Sim3Vertex *>(_vertices[1]);
            const VertexXYZPt *v2 = static_cast<const VertexXYZPt *>(_vertices[0]);

            Vec2 obs(_measurement);
            _error = obs - v1->cam_map2(project(v1->estimate().inverse().map(v2->estimate())));
        }
        // virtual void linearizeOplus();
    };


    class EdgeSim3 : public g2o::BaseBinaryEdge<7, g2o::Sim3, Sim3Vertex, Sim3Vertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeSim3(){}
        virtual bool read(std::istream &is) override
        {
            Vec7 v7;
            for (int i = 0; i < 7; i++)
                is >> v7[i];
            setMeasurement(g2o::Sim3(v7).inverse());

            for (int i = 0; i < 7; i++)
                for (int j = i; j < 7; j++)
                {
                    is >> information()(i, j);
                    if (i != j)
                        information()(j, i) = information()(i, j);
                }
            return true;
        }
        virtual bool write(std::ostream &os) const
        {
            g2o::Sim3 cam2world(measurement().inverse());
            Vec7 v7 = cam2world.log();
            for (int i = 0; i < 7; i++)
            {
                os << v7[i] << " ";
            }
            for (int i = 0; i < 7; i++)
                for (int j = i; j < 7; j++)
                {
                    os << " " << information()(i, j);
                }
            return os.good();
        }

        void computeError()
        {
            const Sim3Vertex *v1 = static_cast<const Sim3Vertex *>(_vertices[0]);
            const Sim3Vertex *v2 = static_cast<const Sim3Vertex *>(_vertices[1]);

            g2o::Sim3 C(_measurement);
            g2o::Sim3 error_ = C * v1->estimate() * v2->estimate().inverse();
            _error = error_.log();
        }

        virtual number_t initialEstimatePossible(const g2o::OptimizableGraph::VertexSet &, g2o::OptimizableGraph::Vertex *) { return 1.; }
        virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet &from, g2o::OptimizableGraph::Vertex * /*to*/)
        {
            Sim3Vertex *v1 = static_cast<Sim3Vertex *>(_vertices[0]);
            Sim3Vertex *v2 = static_cast<Sim3Vertex *>(_vertices[1]);
            if (from.count(v1) > 0)
                v2->setEstimate(measurement() * v1->estimate());
            else
                v1->setEstimate(measurement().inverse() * v2->estimate());
        }
        // virtual void linearizeOplus();
    };

    // ----------------------------------------------------------------------------
    // Indirect.H3 (May 8, 2026): ML-derived scale priors on Sim3 vertices in
    // OptimizeEssentialGraph. Two arms, both shipped as separate edges:
    //
    //   H3-abs (EdgeSim3ScalePrior, unary):  e = log(scale_v) - log(s_target)
    //       — anchors a single KF's Sim3 scale toward an ML-derived target.
    //         Requires bias-correction on s_target (Metric3D 0.55 outdoor)
    //         to avoid pulling the gauge to the biased floor.
    //
    //   H3-rel (EdgeSim3RelScalePrior, binary):
    //       e = log(scale_v_i) - log(scale_v_j) - log(s_target_ratio)
    //       — anchors the relative scale change between two KFs to the ratio
    //         of their independent ML estimates. Bias cancels by construction
    //         (Metric3D bias is multiplicative and ratio-invariant), so no
    //         per-regime bias correction is needed.
    //
    // Sim3Vertex parameterization: 7-DOF tangent [omega(3), upsilon(3), sigma],
    // with sigma = log(s) and oplusImpl: _estimate = g2o::Sim3(update) * _estimate.
    // After update, scale_new = exp(sigma_update) * scale_old, so
    // d(log(scale))/d(update) = [0,0,0,0,0,0,1] regardless of estimate.
    // ----------------------------------------------------------------------------

    class EdgeSim3ScalePrior : public g2o::BaseUnaryEdge<1, double, Sim3Vertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeSim3ScalePrior() {}

        virtual bool read(std::istream &is) override
        {
            double m; is >> m; setMeasurement(m);
            is >> information()(0, 0);
            return true;
        }
        virtual bool write(std::ostream &os) const override
        {
            os << measurement() << " " << information()(0, 0);
            return os.good();
        }

        // Measurement is the *target* scale s_target (NOT log). Stored as-is;
        // we take logs in computeError so callers can pass a natural scalar.
        void computeError() override
        {
            const Sim3Vertex *v = static_cast<const Sim3Vertex *>(_vertices[0]);
            const double s_est = v->estimate().scale();
            const double s_meas = _measurement;
            // Guard against pathological state: if a Sim3Vertex's scale ever
            // hits 0/negative we'd hit log(0)/log(<0). HSLAM has historically
            // crashed on Sophus::ScaleNotPositive in this regime; emit a large
            // but finite error rather than NaN-poisoning the optimizer.
            if (s_est <= 0.0 || s_meas <= 0.0) {
                _error[0] = 1e6;
                return;
            }
            _error[0] = std::log(s_est) - std::log(s_meas);
        }

        void linearizeOplus() override
        {
            // d(log(s))/d(tangent) = [0,0,0,0,0,0,1] in the [omega, upsilon, sigma] order.
            _jacobianOplusXi.setZero();
            _jacobianOplusXi(0, 6) = 1.0;
        }
    };

    class EdgeSim3RelScalePrior : public g2o::BaseBinaryEdge<1, double, Sim3Vertex, Sim3Vertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeSim3RelScalePrior() {}

        virtual bool read(std::istream &is) override
        {
            double m; is >> m; setMeasurement(m);
            is >> information()(0, 0);
            return true;
        }
        virtual bool write(std::ostream &os) const override
        {
            os << measurement() << " " << information()(0, 0);
            return os.good();
        }

        // Measurement is the *target* ratio s_target_i / s_target_j (NOT log).
        // Stored as-is; logs are taken in computeError. Ordering: vertex 0 = i,
        // vertex 1 = j; error = log(s_i / s_j) - log(s_target_ratio).
        void computeError() override
        {
            const Sim3Vertex *vi = static_cast<const Sim3Vertex *>(_vertices[0]);
            const Sim3Vertex *vj = static_cast<const Sim3Vertex *>(_vertices[1]);
            const double s_i = vi->estimate().scale();
            const double s_j = vj->estimate().scale();
            const double s_meas = _measurement;
            if (s_i <= 0.0 || s_j <= 0.0 || s_meas <= 0.0) {
                _error[0] = 1e6;
                return;
            }
            _error[0] = (std::log(s_i) - std::log(s_j)) - std::log(s_meas);
        }

        void linearizeOplus() override
        {
            _jacobianOplusXi.setZero(); _jacobianOplusXi(0, 6) = +1.0;
            _jacobianOplusXj.setZero(); _jacobianOplusXj(0, 6) = -1.0;
        }
    };

} // namespace HSLAM
#endif